//! Register-level access to the SN32F290 USB device controller.
//!
//! # Why this is hand-written rather than using the PAC's field API
//!
//! The PAC in `../pac` was generated from the vendor SVD by svd2rust, and it is
//! correct about two things and wrong about one:
//!
//! * **The register block layout is correct.** `sn_usb::RegisterBlock` places
//!   INTEN/INSTS/INSTSC/ADDR/CFG/SGCTL/EP0CTL..EP6CTL/EPTOGGLE/EP1BUFOS../
//!   RWADDR../RWSTATUS2 at exactly the offsets below. Verified by hand against
//!   the generated accessors and against three other sources (see below).
//!
//! * **Every field accessor is wrong.** The vendor SVD's `bitWidth` elements in
//!   `SN_USB` are corrupt: each field was given `bitWidth == bitOffset + 1`
//!   instead of its real width. svd2rust faithfully turned that into
//!
//!   ```text
//!   pub fn out_stall_en(&self) -> OutStallEnR {
//!       OutStallEnR::new((self.bits >> 27) & 0x0fff_ffff)   // 28-bit mask!
//!   }
//!   ```
//!
//!   for a single-bit field, and `pub fn endp_cnt(&self) -> EndpCntR
//!   { ... & 0x7f }` where the hardware field is 9 bits wide.
//!
//! So this module uses the PAC only for the *base address* and for a
//! compile-time size cross-check, and does all bit manipulation with raw `u32`
//! values and the named constants in [`bits`].
//!
//! Run `tools/svd_dump.py SN_USB` to see the corruption for yourself.
//!
//! # Sources for every constant here
//!
//! Three independent sources agree on every bit position used below:
//!
//! 1. `research/SN32F290.patched.svd` - the vendor SVD's `bitOffset` values.
//!    (Reliable. Its `bitWidth` values are not.)
//! 2. `research/ChibiOS-Contrib/os/hal/ports/SN32/LLD/SN32F2xx/USB/`
//!    (`sn32_usb.h`, `usbhw.h`, `hal_usb_lld.c`) - Apache-2.0, field-proven on
//!    SN32F290 keyboards. Origin: github.com/SonixQMK/ChibiOS-Contrib.
//! 3. The vendor stock image, decompiled in Ghidra. Its USB reset handler at
//!    `0x12C8` writes `INSTSC = 0xFFFFFFFF`, `ADDR = 0` and zeroes `EPCTL[1..6]`
//!    exactly as ChibiOS does; its `INTEN` value `0xB0000040` is precisely
//!    `BUS_IE|USB_IE|BUSWK_IE|EPN_ACK_EN`; and its `CFG` suspend mask
//!    `0xB7FFFFFF` clears exactly `PHY_EN|ESD_EN`. Its endpoint-control writes
//!    are `0x00000000` (off), `0x80000000` (NAK), `0xA0000000|n` (ACK, n bytes)
//!    and `0xE0000000` (STALL).
//!
//! Where a source disagrees, the disagreement is called out in a comment.

use core::ptr::{read_volatile, write_volatile};

// ---------------------------------------------------------------------------
// Register offsets within SN_USB  (SVD: SN_USB base 0x4005C000, IRQ 1)
// ---------------------------------------------------------------------------

/// `INTEN` - USB Interrupt Enable Register.
pub const OFF_INTEN: usize = 0x00;
/// `INSTS` - USB Interrupt Event Status Register (read-only).
pub const OFF_INSTS: usize = 0x04;
/// `INSTSC` - USB Interrupt Event Status Clear Register (write-1-to-clear).
pub const OFF_INSTSC: usize = 0x08;
/// `ADDR` - USB Device Address Register.
pub const OFF_ADDR: usize = 0x0C;
/// `CFG` - USB Configuration Register.
pub const OFF_CFG: usize = 0x10;
/// `SGCTL` - USB Signal Control Register.
pub const OFF_SGCTL: usize = 0x14;
/// `EP0CTL` - USB Endpoint 0 Control Register. `EPnCTL` is at `0x18 + 4*n`.
pub const OFF_EP0CTL: usize = 0x18;
/// `EPTOGGLE` - USB Endpoint Data Toggle Register.
pub const OFF_EPTOGGLE: usize = 0x3C;
/// `EP1BUFOS` - USB Endpoint 1 Buffer Offset Register. `EPnBUFOS` is at
/// `0x48 + 4*(n-1)` for n in 1..=6.
pub const OFF_EP1BUFOS: usize = 0x48;
/// `FRMNO` - USB Frame Number Register.
pub const OFF_FRMNO: usize = 0x60;
/// `PHYPRM` - USB PHY Parameter Register.
pub const OFF_PHYPRM: usize = 0x64;
/// `PHYPRM2` - USB PHY Parameter 2 Register.
pub const OFF_PHYPRM2: usize = 0x6C;
/// `PS2CTL` - PS/2 Control Register.
pub const OFF_PS2CTL: usize = 0x70;
/// `RWADDR` - USB Read/Write Address Register (FIFO port 1).
pub const OFF_RWADDR: usize = 0x78;
/// `RWDATA` - USB Read/Write Data Register (FIFO port 1).
pub const OFF_RWDATA: usize = 0x7C;
/// `RWSTATUS` - USB Read/Write Status Register (FIFO port 1).
pub const OFF_RWSTATUS: usize = 0x80;
/// `RWADDR2` - USB Read/Write Address Register 2 (FIFO port 2).
pub const OFF_RWADDR2: usize = 0x84;
/// `RWDATA2` - USB Read/Write Data Register 2 (FIFO port 2).
pub const OFF_RWDATA2: usize = 0x88;
/// `RWSTATUS2` - USB Read/Write Status Register 2 (FIFO port 2).
pub const OFF_RWSTATUS2: usize = 0x8C;

/// Total size of the `SN_USB` register block. The SVD's `addressBlock` claims
/// 0x2000, but the last register (`RWSTATUS2`) ends at 0x90 and the USB packet
/// RAM starts at +0x100.
pub const USB_REG_BLOCK_SIZE: usize = 0x90;

/// Byte offset from the `SN_USB` base to the USB packet RAM ("PMA").
///
/// Source: ChibiOS `sn32_usb.h` - `SN32_USBRAM_BASE = (SN_USB_BASE + 0x100)`.
/// Corroborated by `SN32_USB_DIRECT_SRAM`, which treats that address as plain
/// memory.
pub const USB_RAM_OFFSET: usize = 0x100;

/// Size of the USB packet RAM in bytes.
///
/// Source: ChibiOS `sn32_usb.h` - `SN32_USB_PMA_SIZE 512` for SN32F290. The
/// allocator comment there ("The first 64 bytes are reserved for the
/// descriptors table. The effective available RAM for endpoint buffers is just
/// 192/448 bytes") corroborates 512 total with 64 reserved.
///
/// **Untested assumption**: I have not measured where the packet RAM actually
/// ends. If 512 is wrong, the failure mode is an endpoint whose buffer overlaps
/// another's, which shows up as corrupted transfers rather than a crash.
pub const USB_RAM_SIZE: usize = 512;

/// USB packet RAM reserved for endpoint 0.
///
/// EP0 uses packet RAM offset 0 for the SETUP packet *and* for its 64-byte data
/// buffer; the two never coexist. The vendor firmware writes its EP0 IN data to
/// packet RAM offset 0 (`FUN_000014a2`, which loops 16 words writing to
/// `RWADDR = i*4`), and ChibiOS's `usb_pm_reset` starts the endpoint allocator at
/// 64 for the same reason.
pub const USB_RAM_EP0_RESERVED: usize = 64;

/// Number of endpoint numbers including endpoint 0. `EP0CTL`..`EP6CTL` is seven
/// registers; `EP1BUFOS`..`EP6BUFOS` is six.
///
/// ChibiOS agrees: `USB_ENDPOINTS_NUMBER 6` and `HAL_MAX_ENDPOINTS 7` for
/// SN32F290, with the register block declaring `EPCTL[USB_ENDPOINTS_NUMBER + 1]`.
pub const EP_COUNT: usize = 7;

/// Highest endpoint *number* excluding endpoint 0.
pub const MAX_ENDPOINT_NUMBER: usize = 6;

/// Endpoint 0's maximum packet size.
///
/// Full-speed control endpoints are limited to 64 bytes, and packet RAM offset 0
/// is only 64 bytes before the allocator's first buffer. usb-device may ask for
/// 8, 16, 32 or 64; anything up to 64 fits.
pub const EP0_MAX_PACKET_SIZE: usize = 64;

// ---------------------------------------------------------------------------
// Bit definitions
// ---------------------------------------------------------------------------

/// Bit definitions, SVD-cited. Every one of these agrees with the vendor SVD's
/// `bitOffset`, with ChibiOS's `usbhw.h`/`sn32_usb.h` masks, and (where the
/// vendor firmware exercises it) with the decompiled stock image.
pub mod bits {
    /// `INSTS` / `INSTSC`: endpoint `n` NAKed a transaction, `n` in 1..=6.
    /// ChibiOS: `mskEPn_NAK(ep) = 0x1 << (ep - 1)`.
    pub const fn ep_nak(n: u32) -> u32 {
        1 << (n - 1)
    }

    /// `INSTS` / `INSTSC`: endpoint `n` ACKed a transaction, `n` in 1..=6.
    /// ChibiOS: `mskEPn_ACK(ep) = 0x1 << (8 + (ep - 1))`.
    pub const fn ep_ack(n: u32) -> u32 {
        1 << (8 + (n - 1))
    }

    /// `INSTS[17]` ERR_TIMEOUT - Timeout Status.
    pub const ERR_TIMEOUT: u32 = 1 << 17;
    /// `INSTS[18]` ERR_SETUP - Wrong Setup data received.
    pub const ERR_SETUP: u32 = 1 << 18;
    /// `INSTS[19]` EP0_OUT_STALL - EP0 OUT STALL transaction.
    pub const EP0_OUT_STALL: u32 = 1 << 19;
    /// `INSTS[20]` EP0_IN_STALL - EP0 IN STALL transaction completed.
    pub const EP0_IN_STALL: u32 = 1 << 20;
    /// `INSTS[21]` EP0_OUT - EP0 OUT ACK transaction flag.
    pub const EP0_OUT: u32 = 1 << 21;
    /// `INSTS[22]` EP0_IN - EP0 IN ACK transaction flag.
    pub const EP0_IN: u32 = 1 << 22;
    /// `INSTS[23]` EP0_SETUP - EP0 SETUP transaction flag.
    pub const EP0_SETUP: u32 = 1 << 23;
    /// `INSTS[24]` EP0_PRESETUP - EP0 SETUP token packet flag.
    ///
    /// **Guess**: the SVD and ChibiOS both name this, and ChibiOS clears it
    /// *before* handling a SETUP and clears `EP0_SETUP` *after*, which reads as
    /// "the setup token is being processed" vs "the setup packet has been
    /// consumed". I mirror ChibiOS's ordering but do not otherwise depend on the
    /// bit's meaning.
    pub const EP0_PRESETUP: u32 = 1 << 24;
    /// `INSTS[25]` BUS_WAKEUP - Bus wakeup flag.
    pub const BUS_WAKEUP: u32 = 1 << 25;
    /// `INSTS[26]` USB_SOF - USB SOF packet received flag.
    pub const USB_SOF: u32 = 1 << 26;
    /// `INSTS[29]` BUS_RESUME - USB bus resume signal flag.
    pub const BUS_RESUME: u32 = 1 << 29;
    /// `INSTS[30]` BUS_SUSPEND - USB bus suspend signal flag.
    pub const BUS_SUSPEND: u32 = 1 << 30;
    /// `INSTS[31]` BUS_RESET - USB bus reset signal flag.
    pub const BUS_RESET: u32 = 1 << 31;

    /// All endpoint NAK flags for endpoints 1..=6.
    pub const EP_NAK_ALL: u32 = 0x3F;
    /// All endpoint ACK flags for endpoints 1..=6 (bits 8..=13).
    pub const EP_ACK_ALL: u32 = 0x3F << 8;

    /// `INTEN[6]` EPN_ACK_EN - endpoint ACK interrupt enable.
    ///
    /// The SVD's own field table puts this at bit 6 with a span of 22 to the next
    /// declared field, so the SVD does not say what the field's real width is.
    /// ChibiOS writes it as a single bit (`mskEPnACK_EN = 0x1 << 6` for a part
    /// with 6 endpoints) and the vendor's `INTEN = 0xB0000040` sets exactly bit
    /// 6. Both agree: single bit.
    pub const EPN_ACK_EN: u32 = 1 << 6;
    /// `INTEN[28]` BUSWK_IE - bus wakeup interrupt enable.
    pub const BUSWK_IE: u32 = 1 << 28;
    /// `INTEN[29]` USB_IE - USB event interrupt enable.
    pub const USB_IE: u32 = 1 << 29;
    /// `INTEN[30]` USB_SOF_IE - SOF interrupt enable.
    pub const USB_SOF_IE: u32 = 1 << 30;
    /// `INTEN[31]` BUS_IE - bus event interrupt enable.
    pub const BUS_IE: u32 = 1 << 31;

    /// `ADDR[6:0]` UADDR - USB device address.
    pub const UADDR: u32 = 0x7F;

    /// `CFG[n-1]` EPn_DIR - endpoint `n` direction. 0 = IN only, 1 = OUT only.
    ///
    /// This is the single most important architectural fact about this
    /// controller: **endpoint numbers 1..=6 are unidirectional.** The SVD spells
    /// it out ("EP1 only handshakes to IN token packet" / "...to OUT token
    /// packet"), and there is one `EPnCTL` and one `EPnBUFOS` per endpoint
    /// *number*, not per direction. Unlike most USB device controllers you cannot
    /// have EP1-IN and EP1-OUT simultaneously; the driver's allocator enforces
    /// this.
    pub const fn ep_dir(n: u32) -> u32 {
        1 << (n - 1)
    }

    /// `CFG[26]` DIS_PDEN - disable the internal D+/D- 175k pull-down resistors.
    pub const DIS_PDEN: u32 = 1 << 26;
    /// `CFG[27]` ESD_EN - USB anti-ESD protection enable.
    pub const ESD_EN: u32 = 1 << 27;
    /// `CFG[28]` SIE_EN - Serial Interface Engine enable.
    pub const SIE_EN: u32 = 1 << 28;
    /// `CFG[29]` DPPU_EN - internal D+ 1.5k pull-up enable.
    pub const DPPU_EN: u32 = 1 << 29;
    /// `CFG[30]` PHY_EN - PHY transceiver enable.
    pub const PHY_EN: u32 = 1 << 30;
    /// `CFG[31]` VREG33_EN - internal 3.3 V regulator output enable.
    pub const VREG33_EN: u32 = 1 << 31;

    /// `SGCTL[1:0]` BUS_DPDN_STATE - forced bus state.
    /// ChibiOS `mskBUS_J_STATE = 0x2 << 0` (D+ = 1, D- = 0), the USB idle state.
    pub const BUS_J_STATE: u32 = 0x2;
    /// `SGCTL[1:0]` - single-ended zero (D+ = 0, D- = 0).
    pub const BUS_SE0_STATE: u32 = 0x0;
    /// `SGCTL[2]` BUS_DRVEN - enable driving the USB bus.
    pub const BUS_DRVEN: u32 = 1 << 2;

    /// `EPnCTL[8:0]` ENDP_CNT - endpoint byte count.
    ///
    /// **Source conflict, resolved deliberately.** The SVD declares this field
    /// 7 bits wide; ChibiOS uses `mskEPn_CNT = 0x1FF` (9 bits). Full-speed
    /// packets are at most 64 bytes, so both are large enough for every legal
    /// value and the difference cannot change behaviour - but masking with 9 bits
    /// is safe if the SVD is right and *necessary* if ChibiOS is right, whereas
    /// the reverse is not. Using the wider mask.
    pub const ENDP_CNT: u32 = 0x1FF;
    /// `EP0CTL[27]` OUT_STALL_EN - enable EP0 OUT STALL handshake.
    pub const EP0_OUT_STALL_EN: u32 = 1 << 27;
    /// `EP0CTL[28]` IN_STALL_EN - enable EP0 IN STALL handshake.
    pub const EP0_IN_STALL_EN: u32 = 1 << 28;
    /// `EPnCTL[30:29]` ENDP_STATE - endpoint handshake state.
    /// 0 = NAK, 1 = ACK, 3 = STALL.
    pub const ENDP_STATE_MASK: u32 = 0x3 << 29;
    /// `EPnCTL[30:29] = 1` - ACK.
    pub const ENDP_STATE_ACK: u32 = 0x1 << 29;
    /// `EPnCTL[30:29] = 0` - NAK.
    pub const ENDP_STATE_NAK: u32 = 0x0 << 29;
    /// `EPnCTL[30:29] = 3` - STALL.
    pub const ENDP_STATE_STALL: u32 = 0x3 << 29;
    /// `EPnCTL[31]` ENDP_EN - endpoint enable.
    pub const ENDP_EN: u32 = 1 << 31;

    /// Compound: an enabled endpoint in the ACK state carrying `len` bytes.
    /// Matches ChibiOS `EPCTL_SET_STAT_ACK` and the vendor's `0xA0000000 | n`.
    pub const fn epctl_ack(len: u32) -> u32 {
        ENDP_EN | ENDP_STATE_ACK | (len & ENDP_CNT)
    }
    /// Compound: an enabled endpoint in the NAK state.
    /// Matches ChibiOS `EPCTL_SET_STAT_NAK`.
    pub const EPCTL_NAK: u32 = ENDP_EN | ENDP_STATE_NAK;
    /// Compound: an enabled endpoint in the STALL state.
    /// Matches ChibiOS `EPCTL_SET_STAT_STALL` and the vendor's `0xE0000000`.
    pub const EPCTL_STALL: u32 = ENDP_EN | ENDP_STATE_STALL;

    /// `RWSTATUS[0]` W_STATUS - FIFO write still in progress.
    pub const RW_W_STATUS: u32 = 1 << 0;
    /// `RWSTATUS[1]` R_STATUS - FIFO read still in progress.
    pub const RW_R_STATUS: u32 = 1 << 1;
}

// ---------------------------------------------------------------------------
// Register block
// ---------------------------------------------------------------------------

/// Mirror of the `SN_USB` register block.
///
/// Field order and reserved gaps are taken from the PAC's generated
/// `sn_usb::RegisterBlock`, whose layout was in turn generated from the SVD. The
/// compile-time assertion below checks the two agree on size, and
/// [`verify_pac_layout`] checks the individual offsets against the PAC's own
/// accessors.
#[repr(C)]
pub struct RegBlock {
    pub inten: u32,
    pub insts: u32,
    pub instsc: u32,
    pub addr: u32,
    pub cfg: u32,
    pub sgctl: u32,
    /// `EPnCTL` for n = 0..=6, at 0x18 + 4n.
    pub epctl: [u32; 7],
    _reserved0: [u32; 2],
    pub eptoggle: u32,
    _reserved1: [u32; 2],
    /// `EPnBUFOS` for n = 1..=6, at 0x48 + 4(n-1).
    pub epbufos: [u32; 6],
    pub frmno: u32,
    pub phyprm: u32,
    _reserved2: u32,
    pub phyprm2: u32,
    pub ps2ctl: u32,
    _reserved3: u32,
    pub rwaddr: u32,
    pub rwdata: u32,
    pub rwstatus: u32,
    pub rwaddr2: u32,
    pub rwdata2: u32,
    pub rwstatus2: u32,
}

/// The register block must be exactly the size the SVD implies (last register
/// `RWSTATUS2` at 0x8C, 4 bytes wide).
const _: () = assert!(core::mem::size_of::<RegBlock>() == USB_REG_BLOCK_SIZE);

// Cross-check against the generated PAC. `RegisterBlock`'s fields are private so
// `offset_of!` cannot reach them, but size and alignment are both public and
// both must match; combined with `verify_pac_layout` (which compares the PAC's
// public per-register accessors against the offsets above) this pins the layout
// from two directions.
#[cfg(target_arch = "arm")]
const _: () = assert!(
    core::mem::size_of::<sn32f290_pac::sn_usb::RegisterBlock>()
        == core::mem::size_of::<RegBlock>()
);
#[cfg(target_arch = "arm")]
const _: () = assert!(
    core::mem::align_of::<sn32f290_pac::sn_usb::RegisterBlock>()
        == core::mem::align_of::<RegBlock>()
);

/// Compares the PAC's own register accessors against the offsets hard-coded in
/// this module.
///
/// Returns `Ok(())` if they agree, or `Err(offset_name)` naming the first
/// register whose address disagrees with the PAC. Intended to be called once at
/// startup, before any USB register is touched, so that a layout mistake is a
/// clean boot-time failure rather than an unexplained hang on a board that is
/// awkward to recover.
#[cfg(target_arch = "arm")]
pub fn verify_pac_layout() -> Result<(), &'static str> {
    use sn32f290_pac::SnUsb;

    let base = SnUsb::ptr() as usize;
    // SAFETY: `SnUsb::ptr()` is the peripheral's fixed base address; we only
    // take addresses of its fields, never read or write through them.
    let pac = unsafe { &*SnUsb::ptr() };

    let checks: [(&'static str, usize, usize); 19] = [
        ("INTEN", base + OFF_INTEN, pac.inten() as *const _ as usize),
        ("INSTS", base + OFF_INSTS, pac.insts() as *const _ as usize),
        ("INSTSC", base + OFF_INSTSC, pac.instsc() as *const _ as usize),
        ("ADDR", base + OFF_ADDR, pac.addr() as *const _ as usize),
        ("CFG", base + OFF_CFG, pac.cfg() as *const _ as usize),
        ("SGCTL", base + OFF_SGCTL, pac.sgctl() as *const _ as usize),
        ("EP0CTL", base + OFF_EP0CTL, pac.ep0ctl() as *const _ as usize),
        ("EP1CTL", base + OFF_EP0CTL + 4, pac.ep1ctl() as *const _ as usize),
        ("EP6CTL", base + OFF_EP0CTL + 24, pac.ep6ctl() as *const _ as usize),
        (
            "EPTOGGLE",
            base + OFF_EPTOGGLE,
            pac.eptoggle() as *const _ as usize,
        ),
        (
            "EP1BUFOS",
            base + OFF_EP1BUFOS,
            pac.ep1bufos() as *const _ as usize,
        ),
        (
            "EP6BUFOS",
            base + OFF_EP1BUFOS + 20,
            pac.ep6bufos() as *const _ as usize,
        ),
        ("FRMNO", base + OFF_FRMNO, pac.frmno() as *const _ as usize),
        ("PHYPRM", base + OFF_PHYPRM, pac.phyprm() as *const _ as usize),
        ("PHYPRM2", base + OFF_PHYPRM2, pac.phyprm2() as *const _ as usize),
        ("PS2CTL", base + OFF_PS2CTL, pac.ps2ctl() as *const _ as usize),
        ("RWADDR", base + OFF_RWADDR, pac.rwaddr() as *const _ as usize),
        ("RWDATA", base + OFF_RWDATA, pac.rwdata() as *const _ as usize),
        (
            "RWSTATUS",
            base + OFF_RWSTATUS,
            pac.rwstatus() as *const _ as usize,
        ),
    ];

    for (name, expected, actual) in checks {
        if expected != actual {
            return Err(name);
        }
    }

    Ok(())
}

/// Host build stand-in for [`verify_pac_layout`]. The PAC is not compiled for
/// the host, so there is nothing to compare against.
#[cfg(not(target_arch = "arm"))]
pub fn verify_pac_layout() -> Result<(), &'static str> {
    Ok(())
}

// ---------------------------------------------------------------------------
// Storage backend
// ---------------------------------------------------------------------------

/// Memory that the register block lives in.
///
/// On the target this is the real peripheral at `SN_USB_BASE`. On the host it is
/// a flat array plus a small model of the FIFO window, which lets the whole
/// driver - not a reimplementation of it - be exercised by `cargo test`.
mod backend {
    #[cfg(target_arch = "arm")]
    use super::RegBlock;

    #[cfg(target_arch = "arm")]
    pub fn usb() -> *mut RegBlock {
        // Base address straight from the PAC, so the address itself stays
        // traceable to the SVD rather than being a literal repeated here.
        // `ptr()` lives on the peripheral singleton, not on `RegisterBlock`.
        sn32f290_pac::SnUsb::ptr() as *mut RegBlock
    }

    /// Base of the USB packet RAM.
    #[cfg(target_arch = "arm")]
    pub fn usb_ram() -> *mut u8 {
        // ChibiOS: SN32_USBRAM_BASE = SN_USB_BASE + 0x100.
        unsafe { (usb() as *mut u8).add(super::USB_RAM_OFFSET) }
    }

    /// SYS1 base. `AHBCLKEN` sits at SYS1 + 0x00.
    #[cfg(target_arch = "arm")]
    pub fn sys1() -> *mut u32 {
        sn32f290_pac::SnSys1::ptr() as *mut u32
    }

    // -----------------------------------------------------------------------
    // Host model
    // -----------------------------------------------------------------------

    /// Host-side stand-in for the peripheral.
    ///
    /// This is more than a block of memory: it models the FIFO window's
    /// handshake, because `fifo_read_word` and `fifo_write_word` poll
    /// `RWSTATUS` until the hardware clears the busy bit. Without the model
    /// those loops would spin forever against a plain array, and the driver
    /// could not be tested at all.
    ///
    /// The model is deliberately small and literal:
    ///
    /// * writing `RWADDR` latches the byte offset
    /// * writing `RWDATA` latches the word
    /// * writing `RWSTATUS = W_STATUS` commits the latched word to packet RAM
    /// * writing `RWSTATUS = R_STATUS` loads the word at the latched offset
    /// * reading `RWDATA` returns the latch
    /// * reading `RWSTATUS` always returns 0, i.e. "not busy"
    ///
    /// That is exactly the sequence ChibiOS and the vendor firmware both use, so
    /// the driver's FIFO code is exercised for real. What it does *not* model is
    /// anything about the SIE: no auto-NAK, no interrupts, no bus traffic.
    #[cfg(not(target_arch = "arm"))]
    pub mod mock {
        use crate::regs::{USB_RAM_SIZE, USB_REG_BLOCK_SIZE};

        pub const REG_WORDS: usize = USB_REG_BLOCK_SIZE / 4;
        pub const PMA_WORDS: usize = USB_RAM_SIZE / 4;

        /// `RWADDR`/`RWADDR2` hold a byte offset in bits 9:2, so the word index
        /// is the stored value shifted right by two.
        const RWADDR_SHIFT: u32 = 2;

        // Offsets inside the register block, repeated here so the model does not
        // depend on the public constants staying in a particular order.
        const O_RWADDR: usize = 0x78;
        const O_RWDATA: usize = 0x7C;
        const O_RWSTATUS: usize = 0x80;
        const O_RWADDR2: usize = 0x84;
        const O_RWDATA2: usize = 0x88;
        const O_RWSTATUS2: usize = 0x8C;

        const W_STATUS: u32 = 1 << 0;
        const R_STATUS: u32 = 1 << 1;

        pub struct File {
            pub regs: [u32; REG_WORDS],
            pub pma: [u32; PMA_WORDS],
            /// Byte offset latched through FIFO port 1.
            pub addr1: u32,
            /// Byte offset latched through FIFO port 2.
            pub addr2: u32,
            /// Data latch for FIFO port 1.
            pub data1: u32,
            /// Data latch for FIFO port 2.
            pub data2: u32,
        }

        pub static mut FILE: File = File {
            regs: [0; REG_WORDS],
            pma: [0; PMA_WORDS],
            addr1: 0,
            addr2: 0,
            data1: 0,
            data2: 0,
        };

        #[inline(always)]
        fn f() -> *mut File {
            // `addr_of_mut!` on a `static mut` is safe and does not create a
            // reference, so no aliasing rule is violated by handing out the
            // pointer.
            core::ptr::addr_of_mut!(FILE)
        }

        /// Word index into packet RAM for a byte offset.
        #[inline(always)]
        fn word(byte_offset: u32) -> usize {
            ((byte_offset >> RWADDR_SHIFT) as usize) % PMA_WORDS
        }

        /// Reads a register, applying the FIFO model where it applies.
        #[inline(always)]
        pub fn read(off: usize) -> u32 {
            // SAFETY: single-threaded test use; `off` is validated by callers.
            unsafe {
                let p = f();
                match off {
                    // Not busy: the poll loops fall straight through.
                    O_RWSTATUS | O_RWSTATUS2 => 0,
                    O_RWDATA => (*p).data1,
                    O_RWDATA2 => (*p).data2,
                    _ => (*p).regs[off / 4],
                }
            }
        }

        /// Writes a register, applying the FIFO model where it applies.
        #[inline(always)]
        pub fn write(off: usize, val: u32) {
            // SAFETY: as `read`.
            unsafe {
                let p = f();
                match off {
                    O_RWADDR => (*p).addr1 = val,
                    O_RWADDR2 => (*p).addr2 = val,
                    O_RWDATA => (*p).data1 = val,
                    O_RWDATA2 => (*p).data2 = val,
                    O_RWSTATUS => {
                        if val & W_STATUS != 0 {
                            let i = word((*p).addr1);
                            (*p).pma[i] = (*p).data1;
                        }
                        if val & R_STATUS != 0 {
                            let i = word((*p).addr1);
                            (*p).data1 = (*p).pma[i];
                        }
                    }
                    O_RWSTATUS2 => {
                        if val & W_STATUS != 0 {
                            let i = word((*p).addr2);
                            (*p).pma[i] = (*p).data2;
                        }
                        if val & R_STATUS != 0 {
                            let i = word((*p).addr2);
                            (*p).data2 = (*p).pma[i];
                        }
                    }
                    _ => (*p).regs[off / 4] = val,
                }
            }
        }

        /// Reads a register without applying the FIFO model. Used by tests to
        /// inspect what the driver actually wrote.
        #[inline(always)]
        pub fn peek_raw(off: usize) -> u32 {
            // SAFETY: as `read`.
            unsafe { (*f()).regs[off / 4] }
        }

        /// Writes a register without applying the FIFO model. Used by tests to
        /// simulate the SIE raising status bits.
        #[inline(always)]
        pub fn poke_raw(off: usize, val: u32) {
            // SAFETY: as `read`.
            unsafe { (*f()).regs[off / 4] = val }
        }

        /// Clears the register file and packet RAM, and the FIFO latches.
        pub fn reset() {
            // SAFETY: as `read`.
            unsafe {
                let p = f();
                (*p).regs = [0; REG_WORDS];
                (*p).pma = [0; PMA_WORDS];
                (*p).addr1 = 0;
                (*p).addr2 = 0;
                (*p).data1 = 0;
                (*p).data2 = 0;
            }
        }

        /// Reads a word straight out of the modelled packet RAM, bypassing the
        /// FIFO window.
        pub fn pma_peek(byte_offset: usize) -> u32 {
            // SAFETY: as `read`.
            unsafe { (*f()).pma[(byte_offset / 4) % PMA_WORDS] }
        }

        /// Stand-in for the SYS1 clock gate. Kept separate from the register
        /// file because `Regs::enable_usb_clock` addresses a different
        /// peripheral.
        pub static mut SYS1: u32 = 0;
    }

    #[cfg(not(target_arch = "arm"))]
    pub fn sys1() -> *mut u32 {
        core::ptr::addr_of_mut!(mock::SYS1)
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

/// Handle to the USB register block and the SYS1 clock gate.
///
/// Zero-sized on the target; carries no state of its own so that it can be
/// freely copied.
#[derive(Clone, Copy)]
pub struct Regs;

impl Regs {
    #[inline(always)]
    pub const fn new() -> Self {
        Regs
    }

    #[cfg(target_arch = "arm")]
    #[inline(always)]
    fn p(&self) -> *mut RegBlock {
        backend::usb()
    }

    /// Volatile 32-bit read at `off` bytes into the USB register block.
    ///
    /// On the host this reads the modelled peripheral instead, including the
    /// FIFO window's handshake - see [`backend::mock`].
    #[inline(always)]
    pub fn read(&self, off: usize) -> u32 {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: `off` is always one of the `OFF_*` constants above, all
            // within the 0x90-byte block, all 4-byte aligned.
            unsafe { read_volatile((self.p() as *const u8).add(off) as *const u32) }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            debug_assert!(off + 4 <= USB_REG_BLOCK_SIZE);
            backend::mock::read(off)
        }
    }

    /// Volatile 32-bit write at `off` bytes into the USB register block.
    #[inline(always)]
    pub fn write(&self, off: usize, val: u32) {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: as `read`.
            unsafe { write_volatile((self.p() as *mut u8).add(off) as *mut u32, val) }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            debug_assert!(off + 4 <= USB_REG_BLOCK_SIZE);
            backend::mock::write(off, val)
        }
    }

    /// Read-modify-write with a mask of bits to set and a mask of bits to clear.
    #[inline(always)]
    pub fn modify(&self, off: usize, set: u32, clear: u32) {
        let v = self.read(off);
        self.write(off, (v | set) & !clear);
    }

    // -- named helpers for the registers the driver touches ------------------

    /// `INSTS` - read the interrupt status.
    #[inline(always)]
    pub fn insts(&self) -> u32 {
        self.read(OFF_INSTS)
    }

    /// `INSTSC` - write-1-to-clear the given interrupt status bits.
    #[inline(always)]
    pub fn instsc_clear(&self, mask: u32) {
        self.write(OFF_INSTSC, mask)
    }

    /// `INTEN` - overwrite the interrupt enable mask.
    #[inline(always)]
    pub fn inten_write(&self, val: u32) {
        self.write(OFF_INTEN, val)
    }

    /// `INTEN` - OR in additional enables.
    #[inline(always)]
    pub fn inten_set(&self, mask: u32) {
        self.modify(OFF_INTEN, mask, 0)
    }

    /// `ADDR` - set the device address.
    #[inline(always)]
    pub fn set_address(&self, addr: u8) {
        self.write(OFF_ADDR, (addr as u32) & bits::UADDR)
    }

    /// `CFG` - set and clear bits.
    #[inline(always)]
    pub fn cfg_modify(&self, set: u32, clear: u32) {
        self.modify(OFF_CFG, set, clear)
    }

    /// `CFG` - overwrite.
    #[inline(always)]
    pub fn cfg_write(&self, val: u32) {
        self.write(OFF_CFG, val)
    }

    /// `SGCTL` - overwrite.
    #[inline(always)]
    pub fn sgctl_write(&self, val: u32) {
        self.write(OFF_SGCTL, val)
    }

    /// `EPnCTL` for `n` in 0..=6.
    #[inline(always)]
    pub fn epctl(&self, n: usize) -> u32 {
        debug_assert!(n <= 6);
        self.read(OFF_EP0CTL + 4 * n)
    }

    /// Overwrite `EPnCTL`.
    #[inline(always)]
    pub fn epctl_write(&self, n: usize, val: u32) {
        debug_assert!(n <= 6);
        self.write(OFF_EP0CTL + 4 * n, val)
    }

    /// `EPnBUFOS` for `n` in 1..=6.
    ///
    /// The value written is a **byte offset** into the packet RAM. The SVD places
    /// the `OFFSET` field at `bitOffset 2` with width 9, so the register value is
    /// the word offset shifted left by two - i.e. the byte offset, written
    /// verbatim. ChibiOS does the same (`USB_SET_BUFFER_OFST(ep, addr)` with
    /// `addr` a byte offset from `usb_pm_alloc`).
    #[inline(always)]
    pub fn epbufos_write(&self, n: usize, byte_offset: usize) {
        debug_assert!((1..=6).contains(&n));
        self.write(OFF_EP1BUFOS + 4 * (n - 1), byte_offset as u32)
    }

    /// `EPTOGGLE` - reset the data toggle for endpoint `n` (1..=6).
    ///
    /// ChibiOS writes `EPTOGGLE = mskEPn_DATA_TOGGLE(ep)` (a plain assignment);
    /// the vendor writes `EPTOGGLE &= ~(1 << bit)`. Writing a 1 clears the
    /// toggle under ChibiOS's interpretation and writing a 0 does nothing under
    /// the vendor's, so the two are only compatible if the register is
    /// write-1-to-clear. **This driver does not touch EPTOGGLE at all** - toggles
    /// are managed by the SIE and reset automatically on bus reset - so the
    /// ambiguity does not affect it.
    pub const EPTOGGLE_UNUSED: () = ();

    /// `PHYPRM` / `PHYPRM2` - PHY parameters, values taken verbatim from
    /// ChibiOS `usb_lld_start`. The SVD names the fields (`PHY_PARAM`,
    /// `PHY_PS`) but documents no encoding, so these are magic numbers with a
    /// citation rather than an understanding.
    #[inline(always)]
    pub fn phy_setup(&self) {
        self.write(OFF_PHYPRM, 0x8000_0000);
        self.write(OFF_PHYPRM2, 0x0000_4004);
    }

    /// Enable the USB AHB clock: `SYS1.AHBCLKEN[4] = USBCLKEN`.
    ///
    /// SVD `SN_SYS1.AHBCLKEN` declares `USBCLKEN` at `bitOffset 4`. ChibiOS's
    /// `sys1EnableUSB()` is `sys1EnableAHB(0x1 << 4)`. Agreement.
    #[inline(always)]
    pub fn enable_usb_clock(&self) {
        // SAFETY: `AHBCLKEN` is at SYS1 + 0x00 per the SVD.
        unsafe {
            let p = backend::sys1();
            let v = read_volatile(p);
            write_volatile(p, v | (1 << 4));
            // Read back, as ChibiOS does, so the write has reached the peripheral
            // before the USB block is touched.
            let _ = read_volatile(p);
        }
    }

    // -- FIFO access ---------------------------------------------------------

    /// Read one 32-bit word from packet RAM through FIFO port 1.
    ///
    /// Protocol, from ChibiOS `sn32_usb_read_fifo`:
    /// `RWADDR = byte_offset; RWSTATUS = 0x02; while (RWSTATUS & 0x02); data = RWDATA`.
    /// Independently confirmed by the vendor's `FUN_0000120c`, which is
    /// byte-for-byte the same sequence.
    #[inline(always)]
    pub fn fifo_read_word(&self, byte_offset: usize) -> u32 {
        self.write(OFF_RWADDR, byte_offset as u32);
        self.write(OFF_RWSTATUS, bits::RW_R_STATUS);
        while (self.read(OFF_RWSTATUS) & bits::RW_R_STATUS) != 0 {
            core::hint::spin_loop();
        }
        self.read(OFF_RWDATA)
    }

    /// Write one 32-bit word to packet RAM through FIFO port 1.
    ///
    /// Protocol, from ChibiOS `sn32_usb_write_fifo`:
    /// `RWADDR = byte_offset; RWDATA = data; RWSTATUS = 0x01; while (RWSTATUS & 0x01)`.
    /// Confirmed by the vendor's `FUN_00001228`, which is the same sequence.
    #[inline(always)]
    pub fn fifo_write_word(&self, byte_offset: usize, data: u32) {
        self.write(OFF_RWADDR, byte_offset as u32);
        self.write(OFF_RWDATA, data);
        self.write(OFF_RWSTATUS, bits::RW_W_STATUS);
        while (self.read(OFF_RWSTATUS) & bits::RW_W_STATUS) != 0 {
            core::hint::spin_loop();
        }
    }

    /// Read one 32-bit word from packet RAM through FIFO port 2.
    ///
    /// Ports 1 and 2 are two independent windows onto the same packet RAM; both
    /// can read and write. ChibiOS uses port 2 for exactly one thing - reading
    /// the SETUP packet - and port 1 for everything else, which is why this
    /// method exists separately. The vendor firmware uses both ports for both
    /// directions (`FUN_0000120c` reads via port 1, `FUN_00001180` via port 2,
    /// `FUN_00001228` writes via port 1, `FUN_000011c2` via port 2), so the split
    /// is a matter of convention rather than capability. This driver keeps
    /// ChibiOS's split in case the SIE holds port 1 during the setup phase.
    #[inline(always)]
    pub fn fifo_read_word_port2(&self, byte_offset: usize) -> u32 {
        self.write(OFF_RWADDR2, byte_offset as u32);
        self.write(OFF_RWSTATUS2, bits::RW_R_STATUS);
        while (self.read(OFF_RWSTATUS2) & bits::RW_R_STATUS) != 0 {
            core::hint::spin_loop();
        }
        self.read(OFF_RWDATA2)
    }

    /// Reset the mock register file and packet RAM. Host builds only.
    #[cfg(not(target_arch = "arm"))]
    pub fn mock_reset(&self) {
        backend::mock::reset();
    }

    /// Write a register without going through the FIFO model, as if the SIE had
    /// produced the value. Host builds only.
    #[cfg(not(target_arch = "arm"))]
    pub fn mock_poke(&self, off: usize, val: u32) {
        backend::mock::poke_raw(off, val);
    }

    /// Read a register without going through the FIFO model. Host builds only.
    #[cfg(not(target_arch = "arm"))]
    pub fn mock_peek(&self, off: usize) -> u32 {
        backend::mock::peek_raw(off)
    }

    /// Read a word straight out of the modelled packet RAM. Host builds only.
    #[cfg(not(target_arch = "arm"))]
    pub fn mock_pma_peek(&self, byte_offset: usize) -> u32 {
        backend::mock::pma_peek(byte_offset)
    }
}

/// The address of the USB packet RAM on the target.
///
/// ChibiOS gates direct SRAM access behind `SN32_USB_DIRECT_SRAM`, which
/// **defaults to FALSE**; the indexed-port path implemented here is the one that
/// ships and the one field-proven on hardware. This function exists so the
/// address is written down somewhere, not because the driver uses it.
#[cfg(target_arch = "arm")]
pub fn usb_ram_address() -> usize {
    backend::usb_ram() as usize
}

/// Host builds have no fixed packet-RAM address.
#[cfg(not(target_arch = "arm"))]
pub fn usb_ram_address() -> usize {
    0
}
