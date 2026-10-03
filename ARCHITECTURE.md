# Whale Certified Firmware — Architecture

**Target:** AULA S98Pro, Sonix SN32F290 (Cortex-M0, 256 KB flash, 32 KB SRAM)
**Language:** Rust, `thumbv6m-none-eabi`, `no_std`
**Status:** design. Phase 1 (ISP handoff) is flashed and proven.

---

## 0. The one rule

**Getting back into the boot ROM must never depend on anything that can break.**

Every design decision below follows from that. The application firmware is what implements
the only proven recovery route. If that firmware has a bug in its USB stack, the route dies
with it. So the firmware must have **more than one way out**, and at least one of them must
not use USB at all.

---

## 1. The safety net — three independent routes to ISP

Ordered by how much they depend on our own code. **Route 1 is the one that matters.**

### Route 1 — Power-on key hold (does not use USB, does not use interrupts)

```
on reset, before ANY peripheral init:
    configure ONE GPIO pair as input+output
    drive the row, read the column
    if the key is held -> enter_isp()
```

Why this is the important one:

* it runs **first**, before clocks, before USB, before anything that can hang
* it reads a single GPIO pair, so the failure surface is a handful of register writes
* it is the mechanism `sonix-keyboard-bootloader` recommends as its primary entry, precisely
  because it *"does not rely on a valid firmware being present"*
* a human can perform it with two hands and no tools

It costs a few GPIO writes and a short delay. **This is the highest-value code in the
firmware**, and it goes in before anything else.

*Blocked on:* `research/MATRIX-PINS.md` — we need the row/column pins and the GPIO register
semantics.

### Route 2 — Boot-time USB timeout (does not need matrix knowledge)

```
init USB
wait up to BOOT_ISP_TIMEOUT_MS for SET_CONFIGURATION
if not enumerated -> enter_isp()
```

If our USB stack is broken enough that the host never configures us, we fall into ISP
automatically. This catches the single most likely failure mode — a USB driver that does not
work — without needing to know anything about the matrix.

**Important scoping:** the timeout applies **only at boot**. It must not re-arm after
unplug/replug, or unplugging the keyboard for three seconds would drop it into ISP mode
during normal use.

### Route 3 — The HID magic (the one we have used all along)

```
feature report, report id 0x00, 65 bytes
  [00] AA 42 89 5A FF 71 62 CC 00 x56
then a second report
  [00] 01 AA 55 00 00 x60   -> reply must echo 01 AA 55 00
```

This is the route proven on real hardware throughout this project. It requires USB to be
working, which is exactly why it is third and not first.

### Route 4 — QMK convention (free compatibility, no extra risk)

Write `0xDEADBEEF` to the last word of RAM and `NVIC_SystemReset()`; the reset handler checks
for it and jumps to ISP. Makes our firmware compatible with `qmk`'s `bootloader_jump`, and
costs essentially nothing.

### The recovery primitive itself

```rust
const ROM_ISP_ENTRY: u32 = 0x1FFF_0301;   // odd => Thumb entry, correct

unsafe fn enter_isp() -> ! {
    cortex_m::interrupt::disable();
    // The IVTM write the vendor performs is deliberately omitted:
    //   * IVTM resets to 0 = "Map to Boot ROM", so writing 0 is a no-op
    //   * IVTMKEY[31:16] is write-only and key-protected, key unknown
    //   * the vendor writes it from a register left over from a callee,
    //     which cannot be a deliberate value
    //   * QMK's field-proven sn32_dfu.c jumps with no IVTM write at all
    let f: extern "C" fn() -> ! = core::mem::transmute(ROM_ISP_ENTRY);
    f();
}
```

**Proven on hardware.** Phase 1 firmware used exactly this and the device came up as
`0C45:7140`, a proper ISP target, with the flash checksum confirming our image.

---

## 2. Boot sequence

```
reset
 |
 +-- 1. disable interrupts
 |
 +-- 2. ROUTE 1: key-hold check          <-- first, minimal, no dependencies
 |        if held -> enter_isp()
 |
 +-- 3. check QMK RAM magic (route 4)
 |        if set  -> enter_isp()
 |
 +-- 4. clock bring-up
 |        start on the internal RC oscillator. Do NOT touch the PLL until the
 |        chip is known to run our code - the flash wait-state figure at 48 MHz
 |        is not established and a wrong value is a silent hang.
 |
 +-- 5. GPIO init (matrix, RGB, LCD)
 |
 +-- 6. USB init
 |        wait up to BOOT_ISP_TIMEOUT_MS for SET_CONFIGURATION
 |        if not enumerated -> enter_isp()      <-- ROUTE 2
 |
 +-- 7. main loop
          matrix scan -> HID reports
          magic handler live on the feature endpoint   <-- ROUTE 3
          RGB / LCD
```

**Step 2 runs before step 4 for a reason.** Clocks are the most likely place for a wrong
register value to hang the chip, and by then it is too late to check a key.

---

## 3. Module layout

```
fw/
├── src/
│   ├── main.rs        boot sequence, safety net, main loop
│   ├── recovery.rs    enter_isp(), the three routes
│   ├── clock.rs       RC bring-up; PLL later
│   ├── matrix.rs      scan, debounce, keymap
│   ├── usb/
│   │   ├── bus.rs     UsbBus implementation for SN_USB   <- the big one
│   │   ├── hid.rs     keyboard + the vendor magic handler
│   │   └── desc.rs    descriptors
│   ├── rgb.rs
│   └── lcd.rs         240x135 RGB565 over MI_02-equivalent
├── memory.x           FLASH 256K @0x00000000, RAM 32K @0x20000000
├── build.rs
└── Cargo.toml
```

`pac/` (generated from the vendor SVD, 35 peripherals, builds clean) sits alongside.

---

## 4. What is proven, and what is not

| Component | Status |
|---|---|
| Toolchain produces bootable Cortex-M0 code | ✅ **proven on hardware** |
| Vector table, linker, `memory.x` | ✅ **proven** |
| Jump to `0x1FFF0301` | ✅ **proven** |
| Recovery from ISP, full flash cycle | ✅ **proven** |
| ISP command protocol, checksum | ✅ **proven** |
| LCD is 240x135 RGB565 | ✅ measured from the vendor app's own GIF |
| LCD data channel is 4096-byte chunks over a 4097-byte output report | ✅ measured |
| **Which LCD controller the panel uses** | ❌ **unknown** |
| **Matrix pins** | ❌ being researched |
| **USB device controller driver** | ❌ being written |
| Clock tree / PLL at 48 MHz | ❌ unknown |
| BOOT pin | ❌ unknown (and we are not opening the case) |

---

## 5. Rules for anything that touches the flash

1. **Never flash without route 1 or route 2 in the image.** If neither is implemented, the
   image is not flashable, regardless of how good it is.
2. **Verify offline before flashing.** `tools/elf2fw.py` checks SP alignment, the reset
   vector's Thumb bit, that the handler lands in flash and is not erased, and that every used
   exception vector is valid. It caught a real bug on its first run.
3. **The toolchain and the flashing tool are separate risks.** The flasher is proven; a new
   firmware is not.
4. **Keep the stock image and `blob_256k.bin`.** Both verified byte-exact by checksum.

---

## 6. The known-unknown list, in priority order

1. **LCD controller.** 240x135 is consistent with an ST7789-family panel, but that is a guess
   and a guess is not good enough to write an init sequence against. Needs evidence: either
   the init sequence in the stock firmware, or a capture of the vendor app uploading a frame.
2. **Matrix pins.** Being researched.
3. **USB driver.** Being written.
4. **Flash wait states at 48 MHz.** Sidestepped by staying on the RC oscillator until the
   firmware is known to run.
5. **The encoder/knob.** The S98Pro has a rotary encoder. Unresearched.

---

## 7. Why the ordering is what it is

The instinct is to write the fun part — the LCD menu — first. That is the wrong order, and
the reason is the same one that made Phase 1 a 320-byte no-op instead of a keyboard:

> **Everything that is fun is behind a door that only opens from the inside.**

Every feature added before the safety net is a feature that can brick the device it runs on.
Route 1 costs a few GPIO writes. It goes in first, and then the fun part is genuinely free.
