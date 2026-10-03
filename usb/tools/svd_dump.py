#!/usr/bin/env python3
"""Dump the authoritative register/field layout for SN_USB (and friends) from the
vendor SVD.

WHY THIS EXISTS
---------------
The svd2rust-generated PAC in ../pac has a *correct register block layout* but
*completely unusable field accessors*, because the vendor SVD's `bitWidth`
elements are corrupt. Every field in SN_USB was emitted with
`bitWidth == bitOffset + 1`, so svd2rust generated e.g.

    pub fn out_stall_en(&self) -> OutStallEnR {
        OutStallEnR::new((self.bits >> 27) & 0x0fff_ffff)   // 28-bit mask!
    }

for a single-bit field. So we cannot use the PAC's field API for USB, and we
need a trustworthy source for the bit positions.

This script prints, for each register:
  * the address offset (this IS reliable - it agrees with ChibiOS's sn32_usb_t)
  * every field's bitOffset (this IS reliable)
  * every field's declared bitWidth (this is NOT reliable - printed so the
    corruption is visible rather than hidden)
  * a derived `span` = distance to the next field's offset, which is what the
    width was probably meant to be, and is 1 for every real single-bit field.

Run:
    python svd_dump.py                      # all USB-ish peripherals
    python svd_dump.py SN_SYS1              # one peripheral
"""

from __future__ import annotations

import sys
from pathlib import Path

from lxml import etree

SVD = Path(__file__).resolve().parents[3] / "research" / "SN32F290.patched.svd"

# Registers named in the SVD with these names are dumped by default: the USB
# device controller plus the system blocks that gate its clock.
DEFAULT_PERIPHERALS = ["SN_USB", "SN_SYS1", "SN_SYS0", "SN_PMU"]


def text(node, tag, default=""):
    el = node.find(tag)
    return default if el is None or el.text is None else el.text.strip()


def dump_peripheral(periph) -> None:
    name = text(periph, "name")
    base = text(periph, "baseAddress")
    desc = text(periph, "description")
    print(f"\n{'=' * 78}")
    print(f"PERIPHERAL {name}  base={base}  ({desc})")
    print("=" * 78)

    for reg in periph.findall("./registers/register"):
        rname = text(reg, "name")
        off = text(reg, "addressOffset")
        rdesc = text(reg, "description")
        print(f"\n  {rname:<12} @ {off:<8} {rdesc}")

        fields = reg.findall("./fields/field")
        if not fields:
            print("      (no fields declared)")
            continue

        entries = []
        for f in fields:
            fname = text(f, "name")
            boff = text(f, "bitOffset", text(f, "lsb"))
            bwid = text(f, "bitWidth", text(f, "width"))
            entries.append((int(boff, 0), int(bwid, 0), fname, text(f, "description")))

        entries.sort()
        for i, (boff, bwid, fname, fdesc) in enumerate(entries):
            # The width the vendor *meant*: run up to the next field, or one bit
            # for the top field. Almost always 1.
            span = (entries[i + 1][0] - boff) if i + 1 < len(entries) else 1
            flag = "" if bwid == span else f"   <-- SVD says {bwid}, span suggests {span}"
            print(f"      [{boff:>2}] {fname:<18} w={bwid:<3} span={span:<3} {fdesc}{flag}")


def main() -> int:
    if not SVD.exists():
        print(f"SVD not found: {SVD}", file=sys.stderr)
        return 1

    tree = etree.parse(str(SVD))
    root = tree.getroot()

    wanted = sys.argv[1:] or DEFAULT_PERIPHERALS
    found = set()
    for periph in root.findall("./peripherals/peripheral"):
        pname = text(periph, "name")
        if pname in wanted:
            dump_peripheral(periph)
            found.add(pname)

    missing = [w for w in wanted if w not in found]
    if missing:
        print(f"\nNOT FOUND in SVD: {missing}", file=sys.stderr)

    # Also report the derived (non-peripheral) interrupt list, since we need to
    # confirm USB is IRQ 1 and what the NVIC priority situation is.
    print(f"\n{'=' * 78}\nINTERRUPTS\n{'=' * 78}")
    for periph in root.findall("./peripherals/peripheral"):
        pname = text(periph, "name")
        for irq in periph.findall("./interrupt"):
            print(f"  {pname:<12} {text(irq, 'name'):<10} value={text(irq, 'value')}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
