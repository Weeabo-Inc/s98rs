//! `usb-device` `UsbBus` implementation for the SN32F290 USB device controller.
//!
//! # Shape of the hardware
//!
//! The SN32F290 has seven endpoint control registers, `EP0CTL`..`EP6CTL`, and six
//! endpoint buffer-offset registers, `EP1BUFOS`..`EP6BUFOS`. Endpoint 0 is
//! bidirectional and special. **Endpoints 1..=6 are unidirectional**: `CFG.EPn_DIR`
//! selects whether endpoint number `n` answers IN tokens or OUT tokens, and there
//! is exactly one buffer and one control register for that number. There is no way
//! to have EP1-IN and EP1-OUT at the same time. [`Sn32UsbBus::alloc_ep`] enforces
//! this, and it is the main way in which this controller differs from the ones the
//! existing `usb-device` drivers were written for.
//!
//! There is also no endpoint-type field in `EPnCTL` - ChibiOS's
//! `usb_lld_init_endpoint` switches on the endpoint type and does *nothing* in
//! every arm. Control, bulk and interrupt endpoints are all handled identically by
//! this hardware. Isochronous is refused at allocation time rather than silently
//! accepted.
//!
//! # Endpoint 0 buffer sharing, and why the SETUP packet is copied out first
//!
//! Packet RAM offset 0 holds *both* the 8-byte SETUP packet and endpoint 0's
//! 64-byte data buffer. They cannot coexist at the same time, so the driver must
//! copy the SETUP packet into processor RAM before allowing any endpoint-0 data
//! traffic. [`Sn32UsbBus::interrupt_handler`] does that copy and leaves endpoint 0
//! **NAKing**; the receiver is only armed later, by
//! [`UsbBus::read`], once the SETUP bytes have been handed to `usb-device`.
//!
//! Arming any earlier - in the interrupt handler, from the direction bit in the
//! SETUP packet - would look efficient and would be wrong: the host can deliver
//! the first OUT data packet within microseconds of the SETUP, and it would land
//! on top of the SETUP packet we have not finished reading.
//!
//! # Concurrency
//!
//! The register-level interrupt work lives in [`Sn32UsbBus::interrupt_handler`],
//! which must be called from the USB IRQ (IRQ 1). Everything that touches state
//! shared with it runs inside a critical section, so the main loop and the ISR
//! never hold a `&mut State` at the same time. See `unsafe impl Sync` below.

use core::cell::UnsafeCell;
use core::cmp::min;

use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

use crate::pma::PmaAlloc;
use crate::regs::{bits, Regs, EP0_MAX_PACKET_SIZE, EP_COUNT, MAX_ENDPOINT_NUMBER};

/// The USB interrupt number on this part.
///
/// SVD: `SN_USB` declares `<interrupt><name>USB</name><value>1</value>`.
/// ChibiOS's `SN32_USB_HANDLER = Vector44` is the same thing: ChibiOS names its
/// vectors by byte offset into the vector table, and 0x44 = entry 17 = IRQ 1.
/// The PAC's `device.x` lists `USB` as the second external interrupt, after
/// `NDT`. Three independent confirmations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum UsbInterrupt {
    /// The USB device controller interrupt.
    Usb = 1,
}

#[cfg(target_arch = "arm")]
// SAFETY: a single-variant enum with a fixed discriminant, so `number()` is
// constant and unique as the trait requires.
unsafe impl cortex_m::interrupt::InterruptNumber for UsbInterrupt {
    #[inline]
    fn number(self) -> u16 {
        self as u16
    }
}

/// Runs `f` with interrupts disabled on the target.
///
/// On the host this is a plain call: the test harness is single-threaded and
/// there is no ISR to race with. Keeping the same call sites on both sides means
/// the host tests execute the same code paths the target does.
pub(crate) mod critical {
    #[cfg(target_arch = "arm")]
    #[inline(always)]
    pub fn free<R>(f: impl FnOnce() -> R) -> R {
        cortex_m::interrupt::free(|_| f())
    }

    #[cfg(not(target_arch = "arm"))]
    #[inline(always)]
    pub fn free<R>(f: impl FnOnce() -> R) -> R {
        f()
    }
}

/// Direction of a configured endpoint.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum EpDirection {
    /// Device to host.
    In,
    /// Host to device.
    Out,
}

/// Per-endpoint configuration, fixed at allocation time.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct EpConfig {
    /// Whether this endpoint number is in use.
    pub allocated: bool,
    /// Direction this endpoint answers.
    pub dir: EpDirection,
    /// Maximum packet size in bytes.
    pub max_packet_size: u16,
    /// Byte offset of this endpoint's buffer in the USB packet RAM.
    pub offset: usize,
    /// Polling interval. Recorded for completeness only: the SN32 has no
    /// per-endpoint interval register, so this is never written to hardware.
    pub interval: u8,
}

impl EpConfig {
    const FREE: EpConfig = EpConfig {
        allocated: false,
        dir: EpDirection::Out,
        max_packet_size: 0,
        offset: 0,
        interval: 0,
    };
}

/// State of the control transfer currently being serviced.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct ControlXfer {
    /// The host-to-device direction bit from the last SETUP packet's
    /// `bmRequestType` (bit 7). Zero means host-to-device.
    dir_in: bool,
    /// `wLength` from the last SETUP packet: the number of bytes the host will
    /// accept in the data stage.
    expected: u16,
    /// Bytes queued on endpoint 0 IN since that SETUP.
    sent: u16,
    /// The data stage is finished, so the next endpoint 0 IN token is the
    /// status stage.
    data_done: bool,
    /// The receiver has been armed for the status stage.
    status_armed: bool,
}

impl ControlXfer {
    const IDLE: ControlXfer = ControlXfer {
        dir_in: false,
        expected: 0,
        sent: 0,
        data_done: false,
        status_armed: false,
    };
}

/// Mutable driver state.
struct State {
    /// Packet-memory allocator.
    pma: PmaAlloc,
    /// Configuration for endpoint numbers 0..=6.
    eps: [EpConfig; EP_COUNT],

    // -- endpoint 0 (control) ------------------------------------------------
    /// Latched SETUP packet, valid while `setup_ready`.
    setup_buf: [u8; 8],
    /// A SETUP packet is waiting to be read by `read(0x00, ..)`.
    setup_ready: bool,
    /// Length of a received endpoint 0 OUT data packet.
    ep0_out_len: usize,
    /// An endpoint 0 OUT data packet is waiting to be read.
    ep0_out_ready: bool,
    /// A zero-length endpoint 0 OUT packet (a status stage) has been seen.
    ep0_status_out: bool,
    /// An endpoint 0 IN packet is queued or in flight.
    ep0_in_busy: bool,
    /// Bookkeeping for the control transfer in progress.
    ctl: ControlXfer,

    // -- endpoints 1..=6 -----------------------------------------------------
    /// Length of the received packet on endpoint `n`, valid while bit `n` of
    /// `out_ready` is set.
    out_len: [u16; EP_COUNT],
    /// Bit `n` set: an OUT packet is waiting to be read on endpoint `n`.
    out_ready: u16,
    /// Bit `n` set: an IN packet completed on endpoint `n`.
    in_complete: u16,
    /// Bit `n` set: an IN packet is queued or in flight on endpoint `n`.
    in_busy: u16,
    /// Bit `n` set: endpoint `n` is stalled.
    stalled: u16,

    // -- bus state -----------------------------------------------------------
    /// A bus reset has occurred and has not yet been reported.
    bus_reset: bool,
    /// A suspend condition has been reported and not yet consumed.
    suspended: bool,
    /// A resume or wakeup condition has been reported and not yet consumed.
    resumed: bool,
}

impl State {
    const fn new() -> Self {
        State {
            pma: PmaAlloc::new(),
            eps: [EpConfig::FREE; EP_COUNT],
            setup_buf: [0; 8],
            setup_ready: false,
            ep0_out_len: 0,
            ep0_out_ready: false,
            ep0_status_out: false,
            ep0_in_busy: false,
            ctl: ControlXfer::IDLE,
            out_len: [0; EP_COUNT],
            out_ready: 0,
            in_complete: 0,
            in_busy: 0,
            stalled: 0,
            bus_reset: false,
            suspended: false,
            resumed: false,
        }
    }

    /// Clears everything that a bus reset invalidates.
    fn clear_transient(&mut self) {
        self.setup_ready = false;
        self.ep0_out_len = 0;
        self.ep0_out_ready = false;
        self.ep0_status_out = false;
        self.ep0_in_busy = false;
        self.ctl = ControlXfer::IDLE;
        self.out_ready = 0;
        self.in_complete = 0;
        self.in_busy = 0;
        self.stalled = 0;
    }
}

/// A `usb-device` bus implementation for the Sonix SN32F290 USB device
/// controller.
///
/// # Example
///
/// ```ignore
/// let alloc = UsbBusAllocator::new(Sn32UsbBus::new());
/// let mut usb_dev = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x0c45, 0x800a))
///     .strings(&[StringDescriptors::default().manufacturer("Whale-chan")])
///     .unwrap()
///     .build();
/// ```
///
/// The bus is a singleton peripheral. [`Sn32UsbBus::new`] does not check for
/// double construction; call it once, after `Peripherals::take()`.
pub struct Sn32UsbBus {
    regs: Regs,
    state: UnsafeCell<State>,
}

// SAFETY: `Sn32UsbBus` is a handle to a single hardware peripheral plus its
// state. Every `&self` method that touches `state` does so inside
// `critical::free`, which on the target masks all configurable-priority
// interrupts - and the USB IRQ is one of those - so the ISR cannot be running
// while a `&mut State` exists. The only other mutator is `&mut self` (in
// `alloc_ep` and `enable`), which by construction cannot overlap with a `&self`
// borrow. This is the same argument `stm32-usbd` and `synopsys-usb-otg` make.
unsafe impl Sync for Sn32UsbBus {}

impl Sn32UsbBus {
    /// Creates the bus handle.
    ///
    /// Does **not** touch any register. The peripheral is brought up by
    /// [`UsbBus::enable`], which `usb-device` calls when you build the device.
    pub const fn new() -> Self {
        Sn32UsbBus {
            regs: Regs::new(),
            state: UnsafeCell::new(State::new()),
        }
    }

    /// Borrows the mutable state under a critical section.
    #[inline(always)]
    fn with_state<R>(&self, f: impl FnOnce(&mut State, &Regs) -> R) -> R {
        critical::free(|| {
            // SAFETY: see `unsafe impl Sync`. Exclusive because interrupts that
            // could re-enter are masked for the duration.
            let st = unsafe { &mut *self.state.get() };
            f(st, &self.regs)
        })
    }

    /// The register block handle, for callers that want to inspect raw
    /// registers from a debugger or a panic handler.
    #[inline(always)]
    pub fn regs(&self) -> &Regs {
        &self.regs
    }

    /// Reads up to `buf.len()` bytes from a PMA buffer, one 32-bit word at a
    /// time, as ChibiOS `sn32_usb_read_fifo` does.
    fn pma_read(regs: &Regs, offset: usize, buf: &mut [u8]) {
        let mut i = 0;
        while i < buf.len() {
            let word = regs.fifo_read_word(offset + i).to_le_bytes();
            let n = min(4, buf.len() - i);
            buf[i..i + n].copy_from_slice(&word[..n]);
            i += 4;
        }
    }

    /// Writes `data` into a PMA buffer, one 32-bit word at a time, zero-padding
    /// the final partial word.
    ///
    /// Zero-padding matters: the packet RAM is not cleared between packets, so
    /// leaving stale bytes in the tail of a short packet would put the previous
    /// transfer's contents on the wire if the endpoint were ever armed with a
    /// longer count than intended.
    fn pma_write(regs: &Regs, offset: usize, data: &[u8]) {
        let mut i = 0;
        while i < data.len() {
            let n = min(4, data.len() - i);
            let mut word = [0u8; 4];
            word[..n].copy_from_slice(&data[i..i + n]);
            regs.fifo_write_word(offset + i, u32::from_le_bytes(word));
            i += 4;
        }
    }

    /// Applies the hardware configuration for one endpoint number `n > 0`.
    fn program_endpoint(regs: &Regs, n: usize, cfg: &EpConfig) {
        debug_assert!(n > 0 && cfg.allocated);

        // CFG.EPn_DIR: 0 selects IN, 1 selects OUT.
        let dir_bit = bits::ep_dir(n as u32);
        match cfg.dir {
            EpDirection::In => regs.cfg_modify(0, dir_bit),
            EpDirection::Out => regs.cfg_modify(dir_bit, 0),
        }

        // EPnBUFOS takes the byte offset verbatim: the SVD places the OFFSET
        // field at bit 2, so the register value is the word offset shifted left
        // by two, which is the byte offset.
        regs.epbufos_write(n, cfg.offset);

        // Enable, NAKing. An OUT endpoint is armed separately, by
        // `hardware_reset`, once every endpoint has been programmed.
        regs.epctl_write(n, bits::EPCTL_NAK);
    }

    /// Full peripheral reset: clears every interrupt flag, returns the device to
    /// address 0, reprograms all endpoints and re-arms OUT endpoints.
    ///
    /// Mirrors ChibiOS `usb_lld_reset` plus `usb_lld_disable_endpoints`, and the
    /// vendor's `FUN_000012c8` - which writes `INSTSC = 0xFFFFFFFF`, `ADDR = 0`
    /// and zeroes `EPCTL[1..6]`, in that order, exactly as this does.
    ///
    /// An associated function rather than a method so that `reset` can call it
    /// with the register handle `with_state` already hands out, instead of
    /// reaching back into `self` from inside the closure.
    fn hardware_reset(regs: &Regs, st: &mut State) {
        // Clear every pending interrupt.
        regs.instsc_clear(0xFFFF_FFFF);

        // Device address returns to 0 during enumeration.
        regs.set_address(0);

        // Disable endpoints 1..=6 and clear their direction bits.
        for n in 1..EP_COUNT {
            regs.epctl_write(n, 0);
            regs.cfg_modify(bits::ep_dir(n as u32), 0);
        }

        // Endpoint 0: enabled, NAKing.
        regs.epctl_write(0, bits::EPCTL_NAK);

        st.pma.reset();
        st.clear_transient();

        // Reprogram every allocated endpoint, then arm its receiver.
        for n in 1..EP_COUNT {
            if !st.eps[n].allocated {
                continue;
            }
            Self::program_endpoint(regs, n, &st.eps[n]);
            if st.eps[n].dir == EpDirection::Out {
                // ACK with a zero byte count so the first OUT packet can
                // arrive. The SIE returns the handshake state to NAK after each
                // transaction (ChibiOS: "useless mcu resets it anyways"), so
                // this has to be re-armed on every read rather than set once.
                regs.epctl_write(n, bits::epctl_ack(0));
            }
        }
    }

    /// Decides whether the control transfer whose SETUP packet has just been
    /// handed to `usb-device` needs its receiver armed immediately, and does so.
    ///
    /// This is the one place the driver looks inside the SETUP packet. It does
    /// not parse the request - `usb-device` owns that - it reads `wLength` and
    /// the direction bit to decide which way to point endpoint 0:
    ///
    /// * **host to device, `wLength > 0`** - there is a data stage coming and
    ///   `usb-device` will not call `write` until it has arrived. Arm the
    ///   receiver now or the host NAKs forever.
    /// * **host to device, `wLength == 0`** - the status stage is an IN token,
    ///   which `usb-device` triggers through `ControlPipe::accept_out` ->
    ///   `ep_in.write(&[])`. Nothing to arm.
    /// * **device to host** - `usb-device` arms the transmitter by calling
    ///   `write`, once it has produced the response.
    fn arm_after_setup(regs: &Regs, st: &mut State) {
        let bm_request_type = st.setup_buf[0];
        let w_length = u16::from_le_bytes([st.setup_buf[6], st.setup_buf[7]]);
        let host_to_device = (bm_request_type & 0x80) == 0;

        st.ctl = ControlXfer {
            dir_in: !host_to_device,
            expected: w_length,
            sent: 0,
            data_done: false,
            status_armed: false,
        };

        if host_to_device && w_length > 0 {
            regs.epctl_write(0, bits::epctl_ack(0));
        } else {
            regs.epctl_write(0, bits::EPCTL_NAK);
        }
    }
}

impl Default for Sn32UsbBus {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbBus for Sn32UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        let dir = match ep_dir {
            UsbDirection::In => EpDirection::In,
            UsbDirection::Out => EpDirection::Out,
        };

        // The controller has no endpoint-type configuration whatsoever: EPnCTL
        // holds enable, handshake state and byte count and nothing else.
        // Isochronous transfers need periodic scheduling the SIE does not appear
        // to provide, so refuse rather than accept and misbehave.
        if matches!(ep_type, EndpointType::Isochronous { .. }) {
            return Err(UsbError::Unsupported);
        }

        let st = self.state.get_mut();

        // Endpoint 0 is bidirectional and consumes no packet-memory buffer: it
        // always uses PMA offset 0, where the SETUP packet also lands.
        // `usb-device` asks for it twice, once per direction, with explicit
        // addresses 0x00 and 0x80.
        if matches!(ep_type, EndpointType::Control) {
            let explicit_wrong = matches!(ep_addr, Some(a) if a.index() != 0);
            if explicit_wrong {
                return Err(UsbError::Unsupported);
            }
            let cfg = &mut st.eps[0];
            cfg.allocated = true;
            cfg.dir = dir;
            cfg.max_packet_size = max_packet_size;
            cfg.offset = 0;
            cfg.interval = interval;
            return Ok(EndpointAddress::from_parts(0, ep_dir));
        }

        // Pick an endpoint number 1..=6.
        let n = match ep_addr {
            Some(a) => {
                let n = a.index();
                if n == 0 || n > MAX_ENDPOINT_NUMBER {
                    return Err(UsbError::InvalidEndpoint);
                }
                if st.eps[n].allocated {
                    // Already taken. This also rejects the opposite direction,
                    // which is correct: an endpoint number on this controller is
                    // unidirectional, so EP1-IN and EP1-OUT cannot coexist.
                    return Err(UsbError::InvalidEndpoint);
                }
                n
            }
            None => match (1..EP_COUNT).find(|&i| !st.eps[i].allocated) {
                Some(i) => i,
                None => return Err(UsbError::EndpointOverflow),
            },
        };

        // Full-speed non-control endpoints are limited to 64 bytes.
        if max_packet_size == 0 || max_packet_size > 64 {
            return Err(UsbError::Unsupported);
        }

        let offset = match st.pma.alloc(max_packet_size as usize) {
            Ok(o) => o,
            Err(_) => return Err(UsbError::EndpointOverflow),
        };

        st.eps[n] = EpConfig {
            allocated: true,
            dir,
            max_packet_size,
            offset,
            interval,
        };

        Ok(EndpointAddress::from_parts(n, ep_dir))
    }

    fn enable(&mut self) {
        let regs = &self.regs;

        // 1. Clock the peripheral on: SYS1.AHBCLKEN[4] = USBCLKEN.
        //    SVD: SN_SYS1.AHBCLKEN.USBCLKEN at bitOffset 4. ChibiOS's
        //    `sys1EnableUSB()` is `sys1EnableAHB(0x1 << 4)`.
        regs.enable_usb_clock();

        // 2. Power up the transceiver while holding the bus in reset. ChibiOS
        //    writes SGCTL = BUS_DRVEN | BUS_J_STATE, then clears BUS_DRVEN to
        //    release.
        regs.sgctl_write(bits::BUS_DRVEN | bits::BUS_J_STATE);

        // 3. CFG = VREG33_EN | PHY_EN | DPPU_EN | SIE_EN | ESD_EN.
        //    The vendor's wake path (FUN_00001310) confirms PHY_EN|ESD_EN; its
        //    suspend path clears exactly those two with the literal mask
        //    0xB7FFFFFF. ChibiOS uses this set verbatim.
        regs.cfg_write(bits::VREG33_EN | bits::PHY_EN | bits::DPPU_EN | bits::SIE_EN | bits::ESD_EN);

        // 4. PHY parameters. Values copied from ChibiOS `usb_lld_start`; the SVD
        //    names the fields but documents no encoding, so these are magic
        //    numbers with a citation rather than an understanding.
        regs.phy_setup();

        // 5. Bus-event interrupts first, matching ChibiOS's ordering.
        regs.inten_write(bits::BUS_IE);

        // 6. Unmask the NVIC line. Done here rather than by the caller so that a
        //    forgotten unmask cannot present as "the device never enumerates".
        //    Priority is the caller's business: set it through
        //    `NVIC::set_priority` on the `cortex-m` peripheral handle. Note the
        //    SN32F290 has only 2 priority bits, so valid priorities are 0..=3.
        #[cfg(target_arch = "arm")]
        // SAFETY: `UsbInterrupt::Usb` is IRQ 1 on this part, a real interrupt
        // line that this driver owns.
        unsafe {
            cortex_m::peripheral::NVIC::unmask(UsbInterrupt::Usb);
        }

        // 7. Release the bus reset.
        regs.sgctl_write(bits::BUS_J_STATE);

        // 8. Post-reset initialisation, as ChibiOS's `usb_lld_start` does by
        //    calling `usb_lld_reset`.
        {
            let st = self.state.get_mut();
            Self::hardware_reset(regs, st);
        }

        // 9. Enable the SIE's event interrupt, the endpoint ACK interrupt and
        //    bus wakeup. The resulting INTEN value is
        //    BUS_IE | USB_IE | EPN_ACK_EN | BUSWK_IE = 0xB0000040 - bit for bit
        //    what the vendor's own USB enable routine (FUN_00000fe6) writes.
        //    SOF is deliberately left off; ChibiOS also only enables it when a
        //    SOF callback is registered.
        regs.inten_set(bits::USB_IE | bits::EPN_ACK_EN | bits::BUSWK_IE);
    }

    fn reset(&self) {
        self.with_state(|st, regs| Self::hardware_reset(regs, st));
    }

    fn set_device_address(&self, addr: u8) {
        self.regs.set_address(addr);
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        let n = ep_addr.index();
        if n >= EP_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }
        let is_in = ep_addr.is_in();

        self.with_state(|st, regs| {
            // ---- endpoint 0: control IN data or a status stage --------------
            if n == 0 {
                if !is_in {
                    return Err(UsbError::InvalidEndpoint);
                }
                if st.ep0_in_busy {
                    return Err(UsbError::WouldBlock);
                }
                if buf.len() > EP0_MAX_PACKET_SIZE {
                    return Err(UsbError::BufferOverflow);
                }
                Self::pma_write(regs, 0, buf);
                regs.epctl_write(0, bits::epctl_ack(buf.len() as u32));
                st.ep0_in_busy = true;

                // Track the data stage so the interrupt handler knows when the
                // next IN token is the status stage rather than more data.
                // Ending on a short packet means the stage is over; so does
                // reaching the length the host asked for.
                if st.ctl.dir_in {
                    st.ctl.sent = st.ctl.sent.saturating_add(buf.len() as u16);
                    if buf.len() < EP0_MAX_PACKET_SIZE || st.ctl.sent >= st.ctl.expected {
                        st.ctl.data_done = true;
                    }
                }
                return Ok(buf.len());
            }

            // ---- endpoints 1..=6 -------------------------------------------
            if !st.eps[n].allocated {
                return Err(UsbError::InvalidEndpoint);
            }
            if st.eps[n].dir != EpDirection::In || !is_in {
                return Err(UsbError::InvalidEndpoint);
            }
            if buf.len() > st.eps[n].max_packet_size as usize {
                return Err(UsbError::BufferOverflow);
            }
            if (st.in_busy & (1 << n)) != 0 {
                return Err(UsbError::WouldBlock);
            }

            let offset = st.eps[n].offset;
            Self::pma_write(regs, offset, buf);
            regs.epctl_write(n, bits::epctl_ack(buf.len() as u32));
            st.in_busy |= 1 << n;
            Ok(buf.len())
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        let n = ep_addr.index();
        if n >= EP_COUNT {
            return Err(UsbError::InvalidEndpoint);
        }
        let is_out = ep_addr.is_out();

        self.with_state(|st, regs| {
            // ---- endpoint 0: SETUP, OUT data, or status stage --------------
            if n == 0 {
                if !is_out {
                    return Err(UsbError::InvalidEndpoint);
                }

                if st.setup_ready {
                    let take = min(buf.len(), 8);
                    buf[..take].copy_from_slice(&st.setup_buf[..take]);
                    if take == 8 {
                        // The SETUP bytes are out of packet RAM, so offset 0 is
                        // free for the data stage. Only now is it safe to arm
                        // the receiver.
                        st.setup_ready = false;
                        Self::arm_after_setup(regs, st);
                    }
                    return Ok(take);
                }

                if st.ep0_out_ready {
                    let len = st.ep0_out_len;
                    if buf.len() < len {
                        // Leave the packet in place so the caller can retry with
                        // a bigger buffer rather than silently losing data.
                        return Err(UsbError::BufferOverflow);
                    }
                    Self::pma_read(regs, 0, &mut buf[..len]);
                    st.ep0_out_ready = false;
                    // Re-arm. This accepts the next OUT data packet if there is
                    // one, and doubles as the status stage for a control-OUT
                    // transfer: `epctl_ack(0)` also answers an IN token with a
                    // zero-length packet, which is what that status stage is.
                    regs.epctl_write(0, bits::epctl_ack(0));
                    return Ok(len);
                }

                if st.ep0_status_out {
                    // Zero-length OUT packet: the status stage of a control-IN
                    // transfer. Valid, and `usb-device` asks for it with an empty
                    // buffer via `ep_out.read(&mut [])`.
                    st.ep0_status_out = false;
                    return Ok(0);
                }

                return Err(UsbError::WouldBlock);
            }

            // ---- endpoints 1..=6 -------------------------------------------
            if !st.eps[n].allocated {
                return Err(UsbError::InvalidEndpoint);
            }
            if st.eps[n].dir != EpDirection::Out || !is_out {
                return Err(UsbError::InvalidEndpoint);
            }
            if (st.out_ready & (1 << n)) == 0 {
                return Err(UsbError::WouldBlock);
            }
            let len = st.out_len[n] as usize;
            if buf.len() < len {
                return Err(UsbError::BufferOverflow);
            }
            let offset = st.eps[n].offset;
            Self::pma_read(regs, offset, &mut buf[..len]);
            st.out_ready &= !(1 << n);
            // Re-arm for the next packet.
            regs.epctl_write(n, bits::epctl_ack(0));
            Ok(len)
        })
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let n = ep_addr.index();
        if n >= EP_COUNT {
            return;
        }
        self.with_state(|st, regs| {
            if n > 0 && !st.eps[n].allocated {
                return;
            }
            if stalled {
                st.stalled |= 1 << n;
                regs.epctl_write(n, bits::EPCTL_STALL);
                return;
            }
            st.stalled &= !(1 << n);
            // Clearing a stall leaves the endpoint NAKing, never ACKing.
            // `ControlPipe` calls `unstall` on endpoint 0 immediately after
            // reading the SETUP packet, so arming a zero-length ACK here would
            // let a stray IN token answer with a zero-length packet during the
            // window before `usb-device` calls `write`. NAK is always safe;
            // the endpoint is armed at the points where the driver knows what
            // the next token means.
            if n == 0 {
                regs.epctl_write(0, bits::EPCTL_NAK);
            } else if st.eps[n].dir == EpDirection::Out {
                // An unstalled OUT endpoint has to be ready to receive.
                regs.epctl_write(n, bits::epctl_ack(0));
            } else {
                regs.epctl_write(n, bits::EPCTL_NAK);
            }
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let n = ep_addr.index();
        if n >= EP_COUNT {
            return false;
        }
        // Read the hardware rather than a software shadow: the SIE can clear a
        // stall on its own at the end of a control transfer.
        let val = self.regs.epctl(n);
        (val & bits::ENDP_STATE_MASK) == bits::ENDP_STATE_STALL
    }

    fn suspend(&self) {
        // ChibiOS clears ESD_EN and PHY_EN; the vendor does the same with the
        // literal mask 0xB7FFFFFF in FUN_00000194 and FUN_00000754. Exact match.
        self.regs.cfg_modify(0, bits::ESD_EN | bits::PHY_EN);
    }

    fn resume(&self) {
        // ChibiOS sets ESD_EN | PHY_EN; the vendor's FUN_00001310 does
        // `CFG |= 0x48000000` - the same two bits - then clears
        // INSTS.BUS_WAKEUP.
        self.regs.cfg_modify(bits::ESD_EN | bits::PHY_EN, 0);
    }

    fn poll(&self) -> PollResult {
        self.with_state(|st, _| {
            if st.bus_reset {
                st.bus_reset = false;
                return PollResult::Reset;
            }
            if st.suspended {
                st.suspended = false;
                return PollResult::Suspend;
            }
            if st.resumed {
                st.resumed = false;
                return PollResult::Resume;
            }

            let mut ep_setup = 0u16;
            let mut ep_out = 0u16;

            // `ep_setup` and `ep_out` must keep being reported until the packet
            // is read, so these are level flags cleared by `read`.
            if st.setup_ready {
                ep_setup |= 1;
            }
            if st.ep0_out_ready || st.ep0_status_out {
                ep_out |= 1;
            }
            ep_out |= st.out_ready;

            // `ep_in_complete` is an edge notification: report each completion
            // exactly once. `usb-device` deliberately drops it when it coincides
            // with a SETUP or OUT event for the same endpoint (see the comment
            // in `UsbDevice::poll`), so clearing on report is correct.
            let ep_in_complete = st.in_complete;
            st.in_complete = 0;

            if ep_setup == 0 && ep_out == 0 && ep_in_complete == 0 {
                return PollResult::None;
            }

            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        })
    }

    /// See [`crate::SET_ADDRESS_QUIRK`] and the `quirk-set-address-before-status`
    /// cargo feature.
    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool =
        cfg!(feature = "quirk-set-address-before-status");
}

/// Register-level USB interrupt service routine.
///
/// Call this from the chip's USB IRQ (IRQ 1). It clears the interrupt status bits
/// it handles and latches events for [`UsbBus::poll`] to report.
///
/// It deliberately does **not** run `usb-device`'s state machine: `poll` is
/// expected to run in thread context. Doing the packet-RAM work here instead
/// keeps the latency between a token arriving and its data being moved to one
/// interrupt.
impl Sn32UsbBus {
    pub fn interrupt_handler(&self) {
        let regs = &self.regs;

        // Latch the status once. Reading INSTS is not destructive on this part;
        // bits are cleared by writing INSTSC.
        let status = regs.insts();

        if status == 0 {
            // ChibiOS re-arms the PHY here as EMC protection. Mirroring that.
            regs.cfg_modify(bits::ESD_EN | bits::PHY_EN, 0);
            return;
        }

        self.with_state(|st, regs| {
            // ---- bus events ------------------------------------------------
            // ChibiOS handles these with `else if` and so does the vendor's
            // dispatcher; a bus reset pre-empts everything else.
            if (status & bits::BUS_RESET) != 0 {
                // Re-arm the PHY, as ChibiOS does before handling the reset.
                regs.cfg_modify(bits::ESD_EN | bits::PHY_EN, 0);

                // Immediately stop responding: return to address 0 and disable
                // every endpoint. The full re-initialisation happens in
                // `UsbBus::reset`, which `usb-device` calls as soon as `poll`
                // reports the reset. The host waits roughly 10 ms after a bus
                // reset before enumerating, so there is ample time.
                regs.set_address(0);
                for n in 1..EP_COUNT {
                    regs.epctl_write(n, 0);
                }
                regs.epctl_write(0, bits::EPCTL_NAK);
                st.clear_transient();
                st.bus_reset = true;

                regs.instsc_clear(bits::BUS_RESET);
            } else if (status & bits::BUS_SUSPEND) != 0 {
                regs.cfg_modify(0, bits::ESD_EN | bits::PHY_EN);
                st.suspended = true;
                regs.instsc_clear(bits::BUS_SUSPEND);
            } else if (status & bits::BUS_RESUME) != 0 {
                regs.cfg_modify(bits::ESD_EN | bits::PHY_EN, 0);
                st.resumed = true;
                regs.instsc_clear(bits::BUS_RESUME);
            }

            // Wakeup is handled separately from resume because the vendor treats
            // it as its own event and it can arrive without a suspend having
            // been seen.
            if (status & bits::BUS_WAKEUP) != 0 {
                regs.cfg_modify(bits::ESD_EN | bits::PHY_EN, 0);
                st.resumed = true;
                regs.instsc_clear(bits::BUS_WAKEUP);
            }

            // ---- endpoint 0 ------------------------------------------------
            // Handled independently rather than as an `else if` chain: a single
            // status read can legitimately have more than one endpoint-0 bit set,
            // and dropping one would lose a packet. Each branch clears only the
            // bit it consumed, so anything left over re-triggers the interrupt.
            let had_setup = (status & bits::EP0_SETUP) != 0;

            if had_setup {
                // ChibiOS clears PRESETUP before handling the SETUP and clears
                // SETUP afterwards; mirroring that ordering.
                regs.instsc_clear(bits::EP0_PRESETUP);

                // The SETUP packet lands in packet RAM offset 0. ChibiOS reads it
                // through FIFO port *2* here while using port 1 for endpoint
                // data; this driver keeps that split in case the SIE holds port 1
                // during the setup phase.
                let mut setup = [0u8; 8];
                for (i, chunk) in setup.chunks_mut(4).enumerate() {
                    chunk.copy_from_slice(&regs.fifo_read_word_port2(i * 4).to_le_bytes());
                }
                st.setup_buf = setup;
                st.setup_ready = true;

                // Anything left over from the previous transfer is meaningless
                // now, and offset 0 is about to be reused.
                st.ep0_out_ready = false;
                st.ep0_status_out = false;
                st.ep0_in_busy = false;
                st.ctl = ControlXfer::IDLE;

                // Deliberately leave endpoint 0 NAKing. It is armed by `read`,
                // once the SETUP bytes are out of packet RAM - see the module
                // documentation.

                regs.instsc_clear(bits::EP0_SETUP);
            }

            if (status & bits::EP0_OUT) != 0 && !had_setup {
                // The receiver is only armed once the SETUP has been consumed, so
                // an OUT packet cannot arrive while `setup_ready` is set; the
                // guard is belt and braces.
                if !st.setup_ready && !st.ep0_out_ready {
                    let len = (regs.epctl(0) & bits::ENDP_CNT) as usize;
                    if len == 0 {
                        st.ep0_status_out = true;
                    } else {
                        st.ep0_out_len = min(len, EP0_MAX_PACKET_SIZE);
                        st.ep0_out_ready = true;
                    }
                }
                regs.instsc_clear(bits::EP0_OUT);
            }

            if (status & bits::EP0_IN) != 0 && !had_setup {
                st.ep0_in_busy = false;
                st.in_complete |= 1;

                // If the data stage of a control-IN transfer has finished, the
                // next token from the host is the status stage: an OUT token for
                // a device-to-host transfer. Arm the receiver for it, once.
                //
                // Arming `ack(0)` is right for both directions of status token -
                // it accepts a zero-length OUT, and it answers an IN token with a
                // zero-length packet, which is what a control-OUT status stage
                // is. A `usb-device`-driven zero-length write handles the
                // control-OUT case on its own; this covers control-IN.
                if st.ctl.dir_in && st.ctl.data_done && !st.ctl.status_armed {
                    regs.epctl_write(0, bits::epctl_ack(0));
                    st.ctl.status_armed = true;
                }

                regs.instsc_clear(bits::EP0_IN);
            }

            // Stall notifications: nothing to do beyond clearing them, but they
            // must be cleared or the interrupt re-fires forever.
            if (status & bits::EP0_OUT_STALL) != 0 {
                regs.instsc_clear(bits::EP0_OUT_STALL);
            }
            if (status & bits::EP0_IN_STALL) != 0 {
                regs.instsc_clear(bits::EP0_IN_STALL);
            }

            if (status & bits::ERR_SETUP) != 0 {
                // Malformed SETUP. ChibiOS stalls endpoint 0 IN and carries on.
                regs.epctl_write(0, bits::EPCTL_STALL);
                regs.instsc_clear(bits::ERR_SETUP);
            }
            if (status & bits::ERR_TIMEOUT) != 0 {
                regs.instsc_clear(bits::ERR_TIMEOUT);
            }

            // ---- endpoints 1..=6 -------------------------------------------
            let mut acks = status & bits::EP_ACK_ALL;
            while acks != 0 {
                let bit = acks.trailing_zeros();
                acks &= acks - 1;

                // EP_ACK_ALL covers bits 8..=13, i.e. endpoints 1..=6.
                let n = (bit - 8 + 1) as usize;
                if n == 0 || n >= EP_COUNT {
                    continue;
                }

                if st.eps[n].allocated {
                    match st.eps[n].dir {
                        EpDirection::In => {
                            st.in_busy &= !(1 << n);
                            st.in_complete |= 1 << n;
                        }
                        EpDirection::Out => {
                            st.out_len[n] = (regs.epctl(n) & bits::ENDP_CNT) as u16;
                            st.out_ready |= 1 << n;
                        }
                    }
                }

                regs.instsc_clear(bits::ep_ack(bit - 8 + 1));
            }

            // NAK interrupts are not enabled, so these should never be set.
            // Clearing them anyway costs one register write and removes any
            // chance of an interrupt storm if the enable mask is ever widened.
            let naks = status & bits::EP_NAK_ALL;
            if naks != 0 {
                regs.instsc_clear(naks);
            }

            if (status & bits::USB_SOF) != 0 {
                regs.instsc_clear(bits::USB_SOF);
            }
        })
    }
}

/// Convenience ISR entry point, in the shape the other `usb-device` drivers use.
///
/// ```ignore
/// static mut HANDLER: Option<InterruptHandler> = None;
///
/// #[interrupt]
/// fn USB() {
///     // SAFETY: written once during init, before the IRQ is unmasked.
///     if let Some(h) = unsafe { (*core::ptr::addr_of!(HANDLER)).as_ref() } {
///         h.handle();
///     }
/// }
/// ```
pub struct InterruptHandler {
    bus: &'static Sn32UsbBus,
}

impl InterruptHandler {
    /// Wraps a bus reference for use as an interrupt handler.
    ///
    /// # Safety
    ///
    /// `bus` must remain valid for the lifetime of the program and must not be
    /// aliased mutably elsewhere. In practice this means the
    /// [`UsbBusAllocator`] owning it must itself live in a `static`.
    pub const unsafe fn new(bus: &'static Sn32UsbBus) -> Self {
        InterruptHandler { bus }
    }

    /// The ISR body. Safe to call at any time; it does nothing when no interrupt
    /// is pending.
    #[inline]
    pub fn handle(&self) {
        self.bus.interrupt_handler();
    }

    /// The bus this handler serves.
    #[inline]
    pub fn bus(&self) -> &'static Sn32UsbBus {
        self.bus
    }
}

/// The `usb-device` allocator type for this bus.
///
/// This is the type you actually store: everything in `usb-device` - classes,
/// descriptors, the device itself - is built from a `&UsbBusAllocator`.
pub type SnUsbBus = UsbBusAllocator<Sn32UsbBus>;

/// Convenience alias matching the naming used by other `usb-device` drivers.
pub type UsbBusType = Sn32UsbBus;
