//! A minimal HID keyboard on the SN32F290, driven by this crate.
//!
//! # What this proves
//!
//! That the `usb-device` stack, a real HID class and this `UsbBus`
//! implementation compile and link together into a Cortex-M0 image - and, when
//! flashed, that the device enumerates as a boot keyboard and emits a keypress.
//!
//! It is deliberately not a keyboard firmware. There is no matrix scan, no
//! keymap, no debounce: it presses the `A` key once a second, forever. The point
//! is to exercise the whole USB path with the smallest possible amount of
//! unrelated code, so that if it does not enumerate the fault is in the USB
//! driver and nowhere else.
//!
//! # Timing without a known clock
//!
//! The SN32F290's internal RC oscillator frequency is not established, so a
//! calibrated delay is not available. Rather than guess, this uses the USB frame
//! number register: `FRMNO` increments once per millisecond while the bus is
//! active. That gives an exact 1 ms tick with no assumption about the core clock
//! at all.
//!
//! The consequence is that if the bus is idle or the device is suspended, the
//! frame number stops and so does the demo. That is acceptable here and it fails
//! visibly (nothing is typed) rather than silently.
//!
//! # Why the interrupt handler is not `#[interrupt]`
//!
//! `cortex_m_rt::interrupt` expands to a path `interrupt::<NAME>`, expecting a
//! module named `interrupt` to be in scope at the call site. The svd2rust PAC in
//! `../pac` does not provide one - it emits `__INTERRUPTS` and a `device.x` full
//! of `PROVIDE(NAME = DefaultHandler)`, which is the other half of the same
//! mechanism. Defining the symbol directly is what those two halves expect, and
//! it needs no macro machinery.
//!
//! # Building
//!
//! ```text
//! cargo build --release --target thumbv6m-none-eabi --example hid_keyboard
//! ```
//!
//! Then convert the ELF with `../tools/elf2fw.py`, which verifies the vector
//! table before writing a raw image. Do not flash this without a proven recovery
//! path: it starts the USB peripheral and enables interrupts, so a bug can leave
//! the part needing the ISP handshake to recover.

#![no_std]
#![no_main]
// The USB allocator, the device and the class all have to live in statics for
// the interrupt handler to reach them, which is the normal shape for embedded
// Rust singletons. Every access is on a single thread of control or inside the
// ISR, and the driver masks interrupts around its own shared state.
#![allow(static_mut_refs)]

use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;
use panic_halt as _;

use usb_device::class::{ControlIn, ControlOut, UsbClass};
use usb_device::class_prelude::*;
use usb_device::control::{Request, RequestType};
use usb_device::prelude::*;

use sn32f290_usb::regs::{verify_pac_layout, OFF_FRMNO};
use sn32f290_usb::{InterruptHandler, Sn32UsbBus, SnUsbBus, UsbInterrupt};

// ---------------------------------------------------------------------------
// HID descriptor data
// ---------------------------------------------------------------------------

/// HID class-specific request codes. These are not in `usb_device::control`,
/// which only covers the standard requests from USB 2.0 chapter 9.
mod hid_request {
    /// Get Report.
    pub const GET_REPORT: u8 = 0x01;
    /// Get Idle.
    pub const GET_IDLE: u8 = 0x02;
    /// Get Protocol.
    pub const GET_PROTOCOL: u8 = 0x03;
    /// Set Report.
    pub const SET_REPORT: u8 = 0x09;
    /// Set Idle.
    pub const SET_IDLE: u8 = 0x0A;
    /// Set Protocol.
    pub const SET_PROTOCOL: u8 = 0x0B;
}

/// HID report descriptor for a boot-protocol keyboard: an 8-byte input report
/// (modifier byte, reserved byte, six keycodes) and a 1-byte LED output report.
const REPORT_DESCRIPTOR: [u8; 63] = [
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x05, 0x07, //   Usage Page (Key Codes)
    0x19, 0xE0, //   Usage Minimum (224)
    0x29, 0xE7, //   Usage Maximum (231)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data, Variable, Absolute) - modifier byte
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input (Constant)                - reserved byte
    0x95, 0x05, //   Report Count (5)
    0x75, 0x01, //   Report Size (1)
    0x05, 0x08, //   Usage Page (LEDs)
    0x19, 0x01, //   Usage Minimum (1)
    0x29, 0x05, //   Usage Maximum (5)
    0x91, 0x02, //   Output (Data, Variable, Absolute) - LED report
    0x95, 0x01, //   Report Count (1)
    0x75, 0x03, //   Report Size (3)
    0x91, 0x01, //   Output (Constant)                 - LED padding
    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x65, //   Logical Maximum (101)
    0x05, 0x07, //   Usage Page (Key Codes)
    0x19, 0x00, //   Usage Minimum (0)
    0x29, 0x65, //   Usage Maximum (101)
    0x81, 0x00, //   Input (Data, Array)               - six keycodes
    0xC0, // End Collection
];

/// The seven bytes that follow `bDescriptorType` in a HID descriptor.
///
/// `wDescriptorLength` is derived from `REPORT_DESCRIPTOR`, so the two cannot
/// drift apart.
const HID_DESCRIPTOR_BODY: [u8; 7] = [
    0x11, 0x01, // bcdHID 1.11
    0x00, // bCountryCode
    0x01, // bNumDescriptors
    0x22, // bDescriptorType: report
    (REPORT_DESCRIPTOR.len() & 0xFF) as u8,
    (REPORT_DESCRIPTOR.len() >> 8) as u8,
];

/// HID usage code for the `A` key.
const KEY_A: u8 = 0x04;

// ---------------------------------------------------------------------------
// HID class
// ---------------------------------------------------------------------------

/// A boot-protocol keyboard with one interrupt IN endpoint.
///
/// Written by hand rather than using `usbd-hid`, because that crate needs
/// `usbd-hid-macros` - a proc-macro helper that is not available in this offline
/// registry. It is a small class; the trait surface is the interesting part.
struct HidKeyboard<'a> {
    iface: InterfaceNumber,
    ep_in: EndpointIn<'a, Sn32UsbBus>,
    /// The report waiting to be sent.
    report: [u8; 8],
    pending: bool,
}

impl<'a> HidKeyboard<'a> {
    fn new(alloc: &'a SnUsbBus) -> Self {
        HidKeyboard {
            iface: alloc.interface(),
            // 8-byte reports, polled every 1 ms. The SN32 has no per-endpoint
            // interval register, so the interval is advertised to the host and
            // otherwise ignored - see `EpConfig::interval`.
            ep_in: alloc.interrupt(8, 1),
            report: [0; 8],
            pending: false,
        }
    }

    /// Queues a report. It goes out on the next `poll`.
    fn send(&mut self, report: [u8; 8]) {
        self.report = report;
        self.pending = true;
    }
}

impl UsbClass<Sn32UsbBus> for HidKeyboard<'_> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        // Interface 0: HID, boot subclass, keyboard protocol.
        writer.interface(self.iface, 0x03, 0x01, 0x01)?;
        writer.write(0x21, &HID_DESCRIPTOR_BODY)?;
        writer.endpoint(&self.ep_in)?;
        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<Sn32UsbBus>) {
        let req = *xfer.request();

        if req.request_type != RequestType::Class {
            return;
        }

        // Class requests arrive in two shapes: the standard GET_DESCRIPTOR with
        // a HID descriptor type in the high byte of wValue, and the HID-specific
        // requests, which have their own request codes.
        if req.request == Request::GET_DESCRIPTOR {
            match req.value >> 8 {
                // Report descriptor.
                0x22 => {
                    let _ = xfer.accept_with_static(&REPORT_DESCRIPTOR);
                }
                // HID descriptor, this time with its own length and type bytes.
                0x21 => {
                    let mut buf = [0u8; 9];
                    buf[0] = 9;
                    buf[1] = 0x21;
                    buf[2..].copy_from_slice(&HID_DESCRIPTOR_BODY);
                    let _ = xfer.accept_with(&buf);
                }
                _ => {
                    let _ = xfer.reject();
                }
            }
            return;
        }

        match req.request {
            // Idle rate 0: the host polls, we do not resend unchanged reports.
            hid_request::GET_IDLE => {
                let _ = xfer.accept_with(&[0u8]);
            }
            // Boot protocol. Report the requested protocol back.
            hid_request::GET_PROTOCOL => {
                let _ = xfer.accept_with(&[(req.value & 0xFF) as u8]);
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<Sn32UsbBus>) {
        let req = *xfer.request();

        if req.request_type != RequestType::Class {
            return;
        }

        match req.request {
            // SET_IDLE, SET_PROTOCOL and SET_REPORT (the LED byte). All accepted
            // and ignored: this demo never lights an LED and never resends an
            // unchanged report.
            hid_request::SET_IDLE
            | hid_request::SET_PROTOCOL
            | hid_request::SET_REPORT => {
                let _ = xfer.accept();
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn endpoint_in_complete(&mut self, _addr: EndpointAddress) {
        self.pending = false;
    }

    fn poll(&mut self) {
        // `endpoint_in_complete` clears `pending` once the host has taken the
        // report. Until then `write` returns `WouldBlock` and this retries.
        if self.pending {
            let _ = self.ep_in.write(&self.report);
        }
    }

    fn reset(&mut self) {
        self.report = [0; 8];
        self.pending = false;
    }
}

// ---------------------------------------------------------------------------
// Singletons
// ---------------------------------------------------------------------------

static mut USB_ALLOC: Option<SnUsbBus> = None;
static mut USB_DEV: Option<UsbDevice<'static, Sn32UsbBus>> = None;
static mut HID: Option<HidKeyboard<'static>> = None;
static mut HANDLER: Option<InterruptHandler> = None;

/// The USB interrupt handler, IRQ 1.
///
/// The symbol name is what matters: the PAC's `__INTERRUPTS` vector table
/// references `USB`, and the PAC's `device.x` says `PROVIDE(USB = DefaultHandler)`.
/// A strong definition here overrides that default.
///
/// # Safety
///
/// Called by hardware on the USB interrupt. It only touches the driver's own
/// state, which is synchronised against the main loop by critical sections
/// inside the driver.
#[no_mangle]
#[allow(non_snake_case)]
extern "C" fn USB() {
    // SAFETY: `HANDLER` is written once during start-up, before the NVIC line is
    // unmasked by `UsbBus::enable`, and is never written again.
    if let Some(h) = unsafe { HANDLER.as_ref() } {
        h.handle();
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut core = CorePeripherals::take().unwrap();

    // Check that this crate's hand-written register offsets agree with the
    // generated PAC before touching the peripheral. A mismatch means the driver
    // would be poking the wrong registers, which on this board shows up as an
    // unexplained hang and a trip through the ISP recovery handshake. Stop
    // before writing anything instead.
    if verify_pac_layout().is_err() {
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }

    // 2 NVIC priority bits on this part, so priorities are 0..=3. Keep USB below
    // the highest so a future 1 ms tick handler can pre-empt it.
    //
    // SAFETY: IRQ 1, owned by this driver, configured before it can fire.
    unsafe {
        core.NVIC.set_priority(UsbInterrupt::Usb, 1);
    }

    // SAFETY: single-threaded start-up. The allocator, the device and the class
    // each live in a static so the references handed to the interrupt handler
    // outlive this function - which is exactly why they are statics and not
    // locals.
    unsafe {
        USB_ALLOC = Some(UsbBusAllocator::new(Sn32UsbBus::new()));
        let alloc: &'static SnUsbBus = USB_ALLOC.as_ref().unwrap();

        HID = Some(HidKeyboard::new(alloc));

        // 0x0C45 / 0x800A is Sonix's vendor ID and the product ID both the stock
        // firmware and the ISP boot ROM present. Using it means a host that
        // already knows this device will match, and it is the pair `sonix-rs`
        // looks for.
        USB_DEV = Some(
            UsbDeviceBuilder::new(alloc, UsbVidPid(0x0C45, 0x800A))
                .strings(&[StringDescriptors::default()
                    .manufacturer("Whale-chan")
                    .product("AULA S98Pro (Rust USB driver)")
                    .serial_number("0001")])
                .unwrap()
                // 64-byte control endpoint. `usb-device` asks for this, and the
                // SN32's endpoint 0 buffer is exactly 64 bytes at packet RAM
                // offset 0.
                .max_packet_size_0(64)
                .unwrap()
                .build(),
        );

        let dev: &'static UsbDevice<'static, Sn32UsbBus> = USB_DEV.as_ref().unwrap();
        HANDLER = Some(InterruptHandler::new(dev.bus()));
    }

    // `UsbDeviceBuilder::build` calls `UsbBus::enable`, which has already
    // started the peripheral, run the post-reset initialisation and unmasked the
    // NVIC line. There is nothing left to do but poll.

    // Timing: FRMNO counts milliseconds while the bus is active. No assumption
    // about the core clock, which is unresolved for this part.
    let mut last_frame = 0u16;
    let mut ms = 0u32;
    let mut phase = 0u8;

    loop {
        let (dev, hid) = unsafe { (USB_DEV.as_mut().unwrap(), HID.as_mut().unwrap()) };

        dev.poll(&mut [hid]);

        let frame = (dev.bus().regs().read(OFF_FRMNO) & 0x07FF) as u16;
        if frame == last_frame {
            continue;
        }
        last_frame = frame;
        ms = ms.saturating_add(1);

        match phase {
            // Idle, then press A.
            0 if ms >= 500 => {
                let mut r = [0u8; 8];
                r[2] = KEY_A;
                hid.send(r);
                phase = 1;
                ms = 0;
            }
            // Hold for 50 ms, then release.
            1 if ms >= 50 => {
                hid.send([0u8; 8]);
                phase = 2;
                ms = 0;
            }
            // Rest, then start again.
            2 if ms >= 500 => {
                phase = 0;
                ms = 0;
            }
            _ => {}
        }
    }
}
