//! Packet-memory allocation for the SN32F290 USB controller.
//!
//! The controller has a flat region of packet RAM ("PMA") that endpoint buffers
//! are carved out of. Unlike controllers with a dedicated register per endpoint
//! direction, there is exactly **one** `EPnBUFOS` per endpoint *number*, which is
//! the same fact that makes endpoints 1..=6 unidirectional: an endpoint number
//! has one buffer, and `CFG.EPn_DIR` says which direction that buffer serves.
//!
//! The allocation policy here mirrors ChibiOS's `usb_pm_reset` / `usb_pm_alloc`,
//! which is field-proven on this part:
//!
//! * start at [`USB_RAM_EP0_RESERVED`] (offset 64), because packet RAM offset 0
//!   is endpoint 0's SETUP packet and data buffer
//! * round each allocation up to a multiple of 4 bytes
//! * fail if the total would exceed [`USB_RAM_SIZE`]
//!
//! **Deviation from ChibiOS, deliberately.** `usb_pm_alloc` rounds up to an even
//! number of bytes (`(size + 1) & ~1`). This driver rounds to 4 instead, because
//! the FIFO is only reachable through 32-bit word accesses at 4-byte-aligned
//! addresses; an allocation ending on an odd 2-byte boundary would put the next
//! endpoint's buffer at an address that cannot be read correctly through
//! `RWADDR` (whose field is a *word* address, bits 9:2). Every legal full-speed
//! packet size (8, 16, 32, 64) is already a multiple of 4, so for real endpoints
//! the two policies produce identical layouts - the difference only shows up for
//! sizes this controller cannot be asked to use anyway.
//!
//! This module is pure arithmetic with no hardware access, so it is fully
//! exercised by the host test suite.

use crate::regs::{USB_RAM_EP0_RESERVED, USB_RAM_SIZE};

/// Bump allocator over the USB packet RAM.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PmaAlloc {
    next: usize,
}

/// Why an allocation failed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PmaOverflow {
    /// Bytes requested.
    pub requested: usize,
    /// Bytes still free when the request was made.
    pub free: usize,
}

impl PmaAlloc {
    /// A fresh allocator, positioned just past endpoint 0's reserved area.
    pub const fn new() -> Self {
        PmaAlloc {
            next: USB_RAM_EP0_RESERVED,
        }
    }

    /// Returns the allocator to its post-reset position.
    pub fn reset(&mut self) {
        self.next = USB_RAM_EP0_RESERVED;
    }

    /// Offset of the next free byte.
    pub const fn next(&self) -> usize {
        self.next
    }

    /// Bytes still available.
    pub const fn free(&self) -> usize {
        USB_RAM_SIZE - self.next
    }

    /// Allocates `size` bytes, returning the byte offset of the buffer.
    ///
    /// The size is rounded up to a multiple of 4 bytes. See the module
    /// documentation for why this differs from ChibiOS's round-to-even.
    pub fn alloc(&mut self, size: usize) -> Result<usize, PmaOverflow> {
        let rounded = (size + 3) & !3;
        if self.next + rounded > USB_RAM_SIZE {
            return Err(PmaOverflow {
                requested: size,
                free: self.free(),
            });
        }
        let off = self.next;
        self.next += rounded;
        Ok(off)
    }
}

impl Default for PmaAlloc {
    fn default() -> Self {
        PmaAlloc::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_after_ep0_reservation() {
        let a = PmaAlloc::new();
        assert_eq!(a.next(), 64);
        assert_eq!(a.free(), 448);
    }

    #[test]
    fn rounds_sizes_up_to_a_multiple_of_four() {
        let mut a = PmaAlloc::new();
        assert_eq!(a.alloc(7).unwrap(), 64);
        // 7 -> 8
        assert_eq!(a.next(), 72);
        assert_eq!(a.alloc(9).unwrap(), 72);
        // 9 -> 12, keeping every buffer 4-byte aligned
        assert_eq!(a.next(), 84);
        assert_eq!(a.next() % 4, 0);
    }

    #[test]
    fn every_legal_packet_size_lands_aligned() {
        // The sizes a full-speed endpoint can actually be allocated with.
        for size in [8usize, 16, 32, 64] {
            let mut a = PmaAlloc::new();
            let off = a.alloc(size).unwrap();
            assert_eq!(off % 4, 0, "offset {off} for size {size} is misaligned");
            assert_eq!(a.next() % 4, 0);
        }
    }

    #[test]
    fn allocations_are_contiguous_and_non_overlapping() {
        let mut a = PmaAlloc::new();
        let mut prev_end = 0usize;
        for size in [8usize, 16, 32, 64, 8] {
            let off = a.alloc(size).unwrap();
            assert!(
                off >= prev_end,
                "allocation at {off} overlaps the previous one ending at {prev_end}"
            );
            prev_end = off + size;
        }
    }

    #[test]
    fn exactly_fills_the_packet_ram() {
        let mut a = PmaAlloc::new();
        // 448 bytes available; seven 64-byte endpoints exactly fill it.
        for _ in 0..7 {
            a.alloc(64).unwrap();
        }
        assert_eq!(a.next(), USB_RAM_SIZE);
        assert_eq!(a.free(), 0);
        assert!(a.alloc(1).is_err());
    }

    #[test]
    fn refuses_to_overflow() {
        let mut a = PmaAlloc::new();
        let err = a.alloc(449).unwrap_err();
        assert_eq!(err.requested, 449);
        assert_eq!(err.free, 448);
        // A failed allocation must not consume space.
        assert_eq!(a.next(), 64);
    }

    #[test]
    fn reset_rewinds_to_the_start() {
        let mut a = PmaAlloc::new();
        a.alloc(64).unwrap();
        a.alloc(32).unwrap();
        a.reset();
        assert_eq!(a.next(), 64);
        assert_eq!(a.alloc(64).unwrap(), 64);
    }
}
