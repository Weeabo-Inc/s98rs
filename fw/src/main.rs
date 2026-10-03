//! Phase 1: "Flash Key 1". The smallest firmware worth flashing.
//!
//! # WHAT THIS IS FOR
//!
//! This is not a keyboard. It is the single test that has to pass before any
//! real firmware is worth writing: **does our toolchain produce code this chip
//! will actually boot?**
//!
//! It exercises the entire chain that can silently fail:
//!
//! * `cortex-m-rt` emits a correct ARMv6-M vector table at `0x00000000`
//! * `memory.x` places `.text` and `.data` correctly for this part
//! * the initial stack pointer lands inside SRAM
//! * the reset vector is odd (Thumb) and points at real code
//! * the chip executes our instructions
//!
//! # WHY IT IS SAFE BY CONSTRUCTION
//!
//! Its normal, correct behaviour **is** entering ISP mode. A bug in it cannot
//! lock us out unless it stops the CPU reaching the jump at all - and that is
//! the one failure this exists to detect, offline, before anything more complex
//! is attempted.
//!
//! Specifically:
//!
//! * it touches **no** peripheral, so it cannot hang on a clock or a bus
//! * it configures **no** PLL, so the undetermined flash wait-state question is
//!   sidestepped entirely
//! * it does not enable interrupts
//! * it does not depend on SRAM beyond the stack
//!
//! # HOW TO TELL IT WORKED
//!
//! After flashing, wait two seconds and send the ISP check:
//!
//! ```text
//! sonix-rs --vid-pid 0C45/800A raw 01AA5500
//! ```
//!
//! If the reply echoes `01 AA 55 00` with `FA FA FA FA`, our code ran and
//! handed control to the boot ROM. If there is no reply, it did not.
//!
//! # THE RECOVERY PRIMITIVE
//!
//! Recovered instruction by instruction from the stock image at `0x014C`:
//!
//! ```asm
//! ldr r0, =0x40060024   @ SN_SYS0.IVTM
//! str r1, [r0]          @ write vector-table mapping   <- omitted, see below
//! ldr r0, =0x1FFF0301   @ boot ROM entry (Thumb)
//! bx  r0
//! ```
//!
//! **We deliberately perform only the jump.** The `IVTM` write is omitted
//! because:
//!
//! * the register's reset value is already `0` = "Map to Boot ROM", so the write
//!   is a no-op
//! * `IVTMKEY[31:16]` is write-only and key-protected, and the key is unknown
//! * the vendor's own code writes it with a **register left over from a callee**,
//!   which cannot be a deliberate value
//! * QMK's field-proven `sn32_dfu.c` jumps to `0x1FFF0301` with no IVTM write
//!   at all
//!
//! Jumping without it is therefore both simpler and better evidenced.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

/// SN32F290 boot ROM ISP entry point.
///
/// Odd address: this is a Thumb entry, which is correct and required. Confirmed
/// by four independent sources - our stock image's literal pool at `0x0158`,
/// QMK's `SN32_BOOTLOADER_ADDRESS`, QMK's `sn32_dfu.c`, and
/// `sonix-keyboard-bootloader`.
const ROM_ISP_ENTRY: u32 = 0x1FFF_0301;

/// Burn a couple of seconds so the device is visible on the bus before it
/// vanishes into ISP mode. Purely for observation; the chip does not care.
///
/// Uses a plain decrementing loop rather than SysTick, because SysTick is
/// vendor-configured on this part (`vendorSystickConfig = false` in the SVD) and
/// touching it would mean touching a peripheral. This function touches nothing.
#[inline(never)]
fn crude_delay() {
    // The chip runs on its internal RC oscillator at reset. The exact frequency
    // is not established, so this is a rough wait, not a calibrated one. Precision
    // is irrelevant here: anything from a fraction of a second to several seconds
    // serves the purpose.
    for _ in 0..2_000_000u32 {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    // No interrupts. Nothing here should ever run concurrently with anything.
    cortex_m::interrupt::disable();

    // Give the host a moment to see the device enumerate before we take it away.
    crude_delay();

    // Hand control to the masked boot ROM. This does not return.
    //
    // SAFETY: `ROM_ISP_ENTRY` is a fixed, documented entry point in mask ROM.
    // It is not a function in the C sense - it never returns - which is why the
    // signature is `fn() -> !`.
    unsafe {
        let enter_isp: extern "C" fn() -> ! = core::mem::transmute(ROM_ISP_ENTRY);
        enter_isp();
    }

    // `enter_isp` is `-> !`, so this is unreachable. Kept because `#[entry]`
    // requires a `-> !` body and an explicit loop documents the intent.
    loop {
        cortex_m::asm::wfi();
    }
}
