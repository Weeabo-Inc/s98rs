# sn32f290-pac — READ THIS BEFORE USING THE FIELD ACCESSORS

**Status:** the register *layout* is correct. The field *accessor API* is **unreliable**.

---

## ⚠️ The vendor SVD's `bitWidth` elements are corrupt

The vendor's CMSIS-SVD for the SN32F290 has bad `bitWidth` values. `svd2rust` faithfully
generated whatever the SVD said, so **many generated field accessors read and write the wrong
bits**, writers included.

Measured against the SVD we generated from:

| | |
|---|---|
| Total fields | 2355 |
| Fields where `bitOffset + bitWidth > 32` | **437** |
| Fields matching the pattern `bitWidth == bitOffset + 1` | 1767 |
| Peripherals affected | **26 of 35** |

`bitOffset + bitWidth > 32` is not a heuristic — it is **impossible** in a 32-bit register. Every
one of those 437 is definitively wrong.

The worst offenders:

| Peripheral | Impossible fields |
|---|---|
| `SN_USB` | 52 |
| `SN_GPIO0`, `SN_GPIO1`, `SN_GPIO3` | 48 each |
| `SN_LCD` | 37 |
| `SN_SYS1` | 31 |
| `SN_CT16B0`..`SN_CT16B5` | 13–22 each |

A concrete example, from `SN_USB`:

```
PHY_EN   bitOffset = 30   bitWidth = 31     -> ends at bit 61 of a 32-bit register
EP1_DIR  bitOffset =  0   bitWidth =  1     -> this one is actually FINE
```

Note that second row. **`bitWidth == bitOffset + 1` also matches legitimate one-bit fields**, so
that pattern alone produces false positives. Use the overflow test to find the real damage.

---

## What this means in practice

### Safe

* **Register addresses and names.** Bases, offsets, register struct layout — all verified against
  the SVD, ChibiOS's `sn32_usb_t`, and the decompiled vendor firmware. These agree.
* **`read()` / `write()` on whole registers**, with your own masks and shifts.
* **`bits()` / `set_bits()`** — 32-bit whole-register access.

### Not safe

* **Any field accessor** on an affected register. `w.phy_en().set_bit()`, `r.out_stall_en().bits()`
  and friends may touch completely unrelated bits.

### The pattern to use instead

```rust
// Do NOT do this on an affected register:
//   usb.cfg.modify(|_, w| w.phy_en().set_bit());

// Do this:
const CFG_PHY_EN: u32 = 1 << 30;
usb.cfg.modify(|r, w| unsafe { w.bits(r.bits() | CFG_PHY_EN) });
```

Every constant must be traceable to a source you trust: the SVD's `bitOffset`, ChibiOS's
`SN32F290.h` / `sn32_usb_t`, or the vendor firmware. **Not** to the generated accessor.

---

## Finding the affected fields

`s98rs/usb/tools/svd_dump.py` flags every field whose declared width is impossible, and prints
the corrected `bitOffset` so you can build a mask by hand:

```console
$ python s98rs/usb/tools/svd_dump.py
```

---

## Why the layout is still trustworthy

Three independent sources agree on it:

1. The vendor SVD's register names, addresses and sizes (only the *widths* are bad)
2. ChibiOS-Contrib's `SN32F290.h`, which is what QMK actually ships
3. The decompiled vendor stock image — e.g. `FUN_00000fe6` writes `INTEN = 0xB0000040`, which
   matches `BUS_IE | USB_IE | BUSWK_IE | EPN_ACK_EN` exactly

The bit *offsets* are consistent across all three. Only the widths are junk.

---

## Fixing it properly

The right fix is to patch the SVD's `bitWidth` values using ChibiOS's headers as the source of
truth, then regenerate. That is a real piece of work and has not been done. Until then:

> **Treat every field accessor as suspect, and use explicit masks.**
