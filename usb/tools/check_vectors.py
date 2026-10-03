#!/usr/bin/env python3
"""Verify the SN32F290 vector table of a linked ELF, specifically that IRQ 1
(the USB interrupt) points at our handler.

WHY THIS EXISTS
---------------
`cortex-m-rt` binds every device interrupt to `DefaultHandler` through
`PROVIDE(USB = DefaultHandler)` in the PAC's `device.x`, and the vector table
itself is a `__INTERRUPTS` static emitted by the PAC. So an interrupt handler
that fails to override the default does not fail to *build*, and it does not fail
to *link* - the image is perfectly valid and the interrupt simply goes nowhere.

On a board whose only recovery route is a bootloader handshake, "the USB
interrupt silently does nothing" is worth catching on the host.

`tools/elf2fw.py` checks the vector table's shape (Thumb bits, addresses in
flash) but cannot know which symbol a vector is *supposed* to point at. This
script supplies that: it resolves a named symbol from `.symtab` and asserts the
vector matches it.

Usage:
    python check_vectors.py <elf> [--entry USB] [--irq 1]
"""

from __future__ import annotations

import argparse
import struct
import sys

# ARMv6-M: word 0 is the initial stack pointer, words 1..15 are the system
# exceptions, and word 16 is IRQ 0. The SN32F290's IRQ numbering is confirmed by
# the vendor SVD (`SN_USB` declares `<value>1</value>`), by ChibiOS's
# `SN32_USB_HANDLER = Vector44` (0x44 = word 17 = IRQ 1), and by the PAC's own
# `device.x`, which lists USB second after NDT.
VECTOR_WORDS_BEFORE_IRQ0 = 16

SHT_SYMTAB = 2
SHT_STRTAB = 3


class Fail(Exception):
    pass


def parse_symbols(data: bytes):
    """Returns {name: (value, shndx)} from .symtab, empty if there is none."""
    (e_shoff,) = struct.unpack_from("<I", data, 32)
    (e_shentsize, e_shnum, e_shstrndx) = struct.unpack_from("<HHH", data, 46)

    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        (sh_name, sh_type, _flags, _addr, sh_offset, sh_size, sh_link,
         _info, _align, sh_entsize) = struct.unpack_from("<IIIIIIIIII", data, off)
        sections.append({
            "name_off": sh_name, "type": sh_type, "offset": sh_offset,
            "size": sh_size, "link": sh_link, "entsize": sh_entsize,
        })

    if e_shstrndx >= len(sections):
        return {}
    shstr = sections[e_shstrndx]
    shstrtab = data[shstr["offset"]:shstr["offset"] + shstr["size"]]

    def shname(i):
        end = shstrtab.find(b"\0", i)
        return shstrtab[i:end].decode("ascii", "replace")

    syms = {}
    for sec in sections:
        if sec["type"] != SHT_SYMTAB:
            continue
        strtab_sec = sections[sec["link"]]
        strtab = data[strtab_sec["offset"]:strtab_sec["offset"] + strtab_sec["size"]]
        n = sec["size"] // sec["entsize"]
        for i in range(n):
            off = sec["offset"] + i * sec["entsize"]
            st_name, st_value, _st_size, _st_info, _st_other, st_shndx = \
                struct.unpack_from("<IIIBBH", data, off)
            if st_shndx == 0:  # SHN_UNDEF
                continue
            end = strtab.find(b"\0", st_name)
            name = strtab[st_name:end].decode("ascii", "replace")
            # Keep the lowest address for a duplicated name, which is what the
            # linker will have resolved a reference to.
            if name not in syms or st_value < syms[name]:
                syms[name] = st_value
    return syms


def load_vectors(data: bytes, e_phoff: int, e_phnum: int, e_phentsize: int,
                 nwords: int) -> list[int]:
    """Reads the first `nwords` words of the loadable segment at address 0."""
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        (p_type, p_offset, p_vaddr, _p_paddr, p_filesz, _p_memsz,
         _p_flags, _align) = struct.unpack_from("<IIIIIIII", data, off)
        if p_type == 1 and p_vaddr == 0 and p_filesz >= nwords * 4:
            return list(struct.unpack_from(f"<{nwords}I", data, p_offset))
    raise Fail("no loadable segment at address 0 - is this a linked image?")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--entry", default="USB",
                    help="symbol the IRQ vector must point at (default USB)")
    ap.add_argument("--irq", type=int, default=1,
                    help="IRQ number to check (default 1, the USB interrupt)")
    a = ap.parse_args()

    data = open(a.elf, "rb").read()
    if data[:4] != b"\x7fELF":
        raise Fail("not an ELF file")

    (e_phoff,) = struct.unpack_from("<I", data, 28)
    (e_phentsize, e_phnum) = struct.unpack_from("<HH", data, 42)

    syms = parse_symbols(data)
    if not syms:
        raise Fail("no .symtab - build with debug info, or this check cannot run")

    if a.entry not in syms:
        raise Fail(f"symbol {a.entry!r} not found in .symtab")

    expected = syms[a.entry]
    word_index = VECTOR_WORDS_BEFORE_IRQ0 + a.irq
    vectors = load_vectors(data, e_phoff, e_phnum, e_phentsize, word_index + 2)
    actual = vectors[word_index]

    default = syms.get("DefaultHandler")
    reset = vectors[1]

    print(f"ELF          : {a.elf}")
    print(f"symbol       : {a.entry} = 0x{expected:08X}")
    print(f"IRQ {a.irq:<3}      : vector word {word_index} (byte 0x{word_index * 4:02X}) "
          f"= 0x{actual:08X}")
    if default is not None:
        print(f"DefaultHandler = 0x{default:08X}")
    print(f"reset vector : 0x{reset:08X}")

    if actual == 0:
        raise Fail(f"IRQ {a.irq} vector is null")
    if actual & 1 == 0:
        raise Fail(f"IRQ {a.irq} vector 0x{actual:08X} has the Thumb bit clear")

    target = actual & ~1
    if target != expected & ~1:
        if default is not None and target == default & ~1:
            raise Fail(
                f"IRQ {a.irq} is bound to DefaultHandler, not to {a.entry}. "
                f"The handler did not override the PROVIDE() default - the "
                f"interrupt will be silently ignored at run time."
            )
        raise Fail(
            f"IRQ {a.irq} vector 0x{target:08X} does not match {a.entry} "
            f"(0x{expected & ~1:08X})."
        )

    print()
    print(f"OK  IRQ {a.irq} -> {a.entry} (0x{target:08X})")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Fail as e:
        print(f"\nFAILED: {e}", file=sys.stderr)
        raise SystemExit(1)
