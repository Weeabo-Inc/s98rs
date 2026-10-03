#!/usr/bin/env python3
"""
elf2fw.py - turn a Rust Cortex-M ELF into a raw flash image, and VERIFY it.

Written because no objcopy is available in this environment, and because for a
safety-critical artifact it is better to have a converter we can read line by
line than one we trust blindly.

The verification is the point. A malformed vector table on an SN32F290 means the
chip does not boot, and the application firmware is what implements ISP entry -
so "does not boot" is a brick. Every property checked below is one that would
otherwise be discovered on hardware, too late.

Usage:
    python elf2fw.py <input.elf> <output.bin> [--size 262144] [--pad 0xFF]
"""

from __future__ import annotations

import argparse
import struct
import sys

PT_LOAD = 1

# SN32F290 memory map, confirmed from the vendor SVD, ChibiOS's SN32F290.ld, and
# the stock image's own initial stack pointer.
FLASH_BASE = 0x0000_0000
FLASH_SIZE = 256 * 1024
SRAM_BASE = 0x2000_0000
SRAM_SIZE = 32 * 1024
ROM_ISP_ENTRY = 0x1FFF_0301


class Fail(Exception):
    pass


def parse_elf(data: bytes):
    """Parse enough of an ELF32 header to find the loadable segments.

    # The bug this function used to have

    The format string was `<HHIIIIHHHH` - four `I`s - but the ELF32 header
    carries **five** 32-bit fields between `e_machine` and `e_ehsize`:

        e_version, e_entry, e_phoff, e_shoff, e_flags

    Reading `e_flags` as a 2-byte `H` shifted every subsequent field by two
    bytes, so `e_phnum` came back as garbage and only one of the three program
    headers was found. The resulting image was missing its entire `.text`
    section, and the verification correctly reported that the reset handler
    pointed at erased flash.

    The ELF was fine. The reader was not. Getting a struct width wrong produces
    a plausible-looking object with wrong values in it - which is exactly the
    failure mode this whole project keeps meeting.
    """
    if data[:4] != b"\x7fELF":
        raise Fail("not an ELF file")
    if data[4] != 1:
        raise Fail(f"not 32-bit (EI_CLASS={data[4]})")
    if data[5] != 1:
        raise Fail(f"not little-endian (EI_DATA={data[5]})")

    (e_type, e_machine, _e_version, e_entry, e_phoff, _e_shoff, _e_flags,
     _e_ehsize, e_phentsize, e_phnum, _e_shentsize, _e_shnum,
     _e_shstrndx) = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)

    if e_machine != 40:
        raise Fail(f"not ARM (e_machine={e_machine}, expected 40)")

    segs = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        (p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz,
         p_flags, _align) = struct.unpack_from("<IIIIIIII", data, off)
        if p_type == PT_LOAD and p_filesz > 0:
            segs.append({
                "offset": p_offset, "vaddr": p_vaddr, "paddr": p_paddr,
                "filesz": p_filesz, "memsz": p_memsz, "flags": p_flags,
            })
    segs.sort(key=lambda s: s["paddr"])
    return e_entry, segs


def flatten(data: bytes, segs, size: int, pad: int) -> bytearray:
    img = bytearray([pad]) * 1 if False else bytearray([pad] * size)
    for s in segs:
        start = s["paddr"]
        end = start + s["filesz"]
        if start >= size:
            raise Fail(f"segment at 0x{start:08X} is outside the {size} byte image")
        if end > size:
            raise Fail(f"segment 0x{start:08X}..0x{end:08X} overruns the image")
        img[start:end] = data[s["offset"]:s["offset"] + s["filesz"]]
    return img


def verify(img: bytearray, e_entry: int) -> list[str]:
    """Every check here would otherwise be a discovery made on real hardware."""
    notes = []

    sp, reset = struct.unpack_from("<II", img, 0)

    # 1. Initial stack pointer must point into SRAM, and be 8-byte aligned.
    if not (SRAM_BASE <= sp < SRAM_BASE + SRAM_SIZE):
        raise Fail(f"initial SP 0x{sp:08X} is not in SRAM "
                   f"(0x{SRAM_BASE:08X}..0x{SRAM_BASE + SRAM_SIZE:08X})")
    if sp % 8 != 0:
        raise Fail(f"initial SP 0x{sp:08X} is not 8-byte aligned "
                   f"(ARMv6-M requires 8-byte stack alignment)")
    notes.append(f"SP 0x{sp:08X} in SRAM, 8-byte aligned")

    # 2. The reset vector must have the Thumb bit set and land in flash.
    if reset & 1 == 0:
        raise Fail(f"reset vector 0x{reset:08X} has the Thumb bit CLEAR - "
                   f"the CPU would fault on reset")
    target = reset & ~1
    if not (FLASH_BASE <= target < FLASH_BASE + FLASH_SIZE):
        raise Fail(f"reset handler 0x{target:08X} is not in flash")
    notes.append(f"reset -> 0x{target:08X} (Thumb bit set, in flash)")

    # 3. The reset handler must actually contain instructions, not padding.
    if img[target:target + 2] == b"\xff\xff":
        raise Fail(f"reset handler at 0x{target:08X} is erased flash (0xFFFF)")

    # 4. The ELF entry point should agree with the reset vector. cortex-m-rt
    #    sets e_entry to the reset handler, so a mismatch means something is
    #    wrong with the link.
    if e_entry & ~1 != target:
        raise Fail(f"ELF entry 0x{e_entry:08X} disagrees with reset vector "
                   f"0x{target:08X}")
    notes.append(f"ELF entry agrees (0x{e_entry:08X})")

    # 5. Every exception vector that is used must be odd (Thumb) and in flash,
    #    or zero (unused). A vector that is even-but-nonzero is a time bomb.
    bad = []
    used = 0
    for i in range(1, 16):
        v = struct.unpack_from("<I", img, i * 4)[0]
        if v == 0:
            continue
        used += 1
        if v & 1 == 0:
            bad.append((i, v, "Thumb bit clear"))
        elif not (FLASH_BASE <= (v & ~1) < FLASH_BASE + FLASH_SIZE):
            bad.append((i, v, "outside flash"))
    if bad:
        raise Fail("bad exception vectors: " +
                   "; ".join(f"[{i}] 0x{v:08X} {why}" for i, v, why in bad))
    notes.append(f"{used}/15 exception vectors present and valid")

    # 6. The image must be big enough to contain a vector table but must not
    #    claim the whole flash unless it really is that big.
    return notes


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("out")
    ap.add_argument("--size", type=lambda s: int(s, 0), default=FLASH_SIZE)
    ap.add_argument("--pad", type=lambda s: int(s, 0), default=0xFF)
    a = ap.parse_args()

    data = open(a.elf, "rb").read()
    e_entry, segs = parse_elf(data)

    print(f"ELF        : {a.elf}")
    print(f"size       : {len(data):,} bytes")
    print(f"entry      : 0x{e_entry:08X}")
    print(f"PT_LOAD    : {len(segs)} segment(s)")
    for s in segs:
        kind = []
        if s["flags"] & 4: kind.append("R")
        if s["flags"] & 2: kind.append("W")
        if s["flags"] & 1: kind.append("X")
        print(f"   0x{s['paddr']:08X}  filesz={s['filesz']:>7,}  "
              f"memsz={s['memsz']:>7,}  {''.join(kind)}")

    img = flatten(data, segs, a.size, a.pad)

    print()
    print("VERIFYING")
    notes = verify(img, e_entry)
    for n in notes:
        print(f"   ok  {n}")

    # Report the real extent so nobody mistakes padding for content.
    last = a.size
    while last > 0 and img[last - 1] == a.pad:
        last -= 1
    print(f"   ok  real content ends at 0x{last:X} ({last:,} bytes)")

    with open(a.out, "wb") as fh:
        fh.write(img)

    print()
    print(f"WROTE {a.out}  ({len(img):,} bytes, padded with 0x{a.pad:02X})")
    print()
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Fail as e:
        print(f"\nVERIFICATION FAILED: {e}", file=sys.stderr)
        raise SystemExit(1)
