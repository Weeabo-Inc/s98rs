//! A [`usb_device::bus::UsbBus`] implementation for the Sonix SN32F290's
//! full-speed USB device controller.
//!
//! # What this is
//!
//! The SN32F290 is a Cortex-M0 keyboard/mouse controller (256 KB flash, 32 KB
//! SRAM, up to 48 MHz). This crate lets the `usb-device` stack drive its USB
//! peripheral at `0x4005C000`, IRQ 1.
//!
//! ```no_run
//! # #[cfg(target_arch = "arm")] mod example {
//! use usb_device::prelude::*;
//! use sn32f290_usb::{Sn32UsbBus, SnUsbBus};
//!
//! static mut USB_ALLOC: Option<SnUsbBus> = None;
//!
//! fn init() -> UsbDevice<'static, Sn32UsbBus> {
//!     unsafe {
//!         USB_ALLOC = Some(UsbBusAllocator::new(Sn32UsbBus::new()));
//!         let alloc = USB_ALLOC.as_ref().unwrap();
//!         UsbDeviceBuilder::new(alloc, UsbVidPid(0x0c45, 0x800a))
//!             .max_packet_size_0(64)
//!             .unwrap()
//!             .build()
//!     }
//! }
//! # }
//! ```
//!
//! # What is verified, and what is not
//!
//! Everything here compiles for `thumbv6m-none-eabi` and the logic is covered by
//! host tests that drive the real driver code against a memory-backed register
//! file. **Nothing here has run on the SN32F290.** Read
//! `research/USB-DRIVER-NOTES.md` before flashing it; in particular, the
//! `quirk-set-address-before-status` feature exists because one hardware
//! behaviour on the enumeration path could not be confirmed from any source.
//!
//! # Register documentation
//!
//! Bit positions come from the vendor SVD, cross-checked against
//! `SonixQMK/ChibiOS-Contrib`'s working C driver and against the decompiled
//! vendor stock firmware. [`regs`] documents each one and where it came from.
//!
//! **The PAC's USB field accessors are unusable and this crate does not use
//! them.** The vendor SVD's `bitWidth` elements in `SN_USB` are corrupt, which
//! svd2rust turned into nonsense masks - see [`regs`] for the details.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod bus;
pub mod pma;
pub mod regs;

#[cfg(test)]
mod tests;

pub use bus::{EpConfig, EpDirection, InterruptHandler, Sn32UsbBus, SnUsbBus, UsbBusType, UsbInterrupt};
pub use pma::{PmaAlloc, PmaOverflow};
pub use regs::{
    bits, verify_pac_layout, Regs, EP0_MAX_PACKET_SIZE, EP_COUNT, MAX_ENDPOINT_NUMBER, USB_RAM_SIZE,
};

/// The `usb-device` version this crate targets.
///
/// Pinned in `Cargo.toml` to `=0.3.2`. The `0.3.x` series is the first that uses
/// `portable-atomic` instead of `core::sync::atomic`, which is what makes it
/// buildable on Cortex-M0 at all: `thumbv6m-none-eabi` has no `LDREX`/`STREX`, so
/// `target_has_atomic = "32"` is absent. `usb-device` only ever performs
/// load/store on its single `AtomicPtr`, and ARMv6-M provides those natively, so
/// no atomic fallback is pulled in - confirmed with `cargo tree`.
pub const USB_DEVICE_VERSION: &str = "0.3.2";

/// Why the `quirk-set-address-before-status` feature exists.
///
/// `usb-device` normally follows USB 2.0 section 9.4.6 and only programs the
/// device address after the status stage of `SET_ADDRESS` has completed. On this
/// driver that means the enumeration path depends on the controller raising
/// `INSTS.EP0_IN` for a **zero-length IN packet**: `ControlPipe::accept_out`
/// writes an empty slice, the controller sends a zero-length packet, and only
/// when `handle_in_complete` reports that transfer finished does `usb-device`
/// call [`usb_device::bus::UsbBus::set_device_address`].
///
/// That the SN32F290 raises `EP0_IN` for a zero-length IN is *strongly implied*
/// but not proven - see "What I could not determine" in
/// `research/USB-DRIVER-NOTES.md`. Both the vendor firmware and ChibiOS arm
/// zero-length endpoint 0 transfers and then wait for the completion, which is
/// only sensible if the completion arrives.
///
/// If enumeration stops right after `SET_ADDRESS` - the device resets, gets a
/// descriptor, then the host gives up asking for the device descriptor again -
/// enable the `quirk-set-address-before-status` cargo feature. That sets
/// `QUIRK_SET_ADDRESS_BEFORE_STATUS = true`, making `usb-device` program the
/// address before the status stage instead of after. It is a small deviation
/// from the specification in exchange for not depending on the unverified
/// behaviour.
pub const SET_ADDRESS_QUIRK: () = ();
