MEMORY
{
  /* SN32F290: 256 KB user flash at 0x00000000, 32 KB SRAM at 0x20000000.
   * Confirmed three ways: the vendor SVD, ChibiOS-Contrib's SN32F290.ld, and
   * the stock image's own initial stack pointer of 0x20002B18 (which needs more
   * than 8 KB and fits comfortably in 32 KB).
   *
   * The ISP boot ROM lives at 0x1FFF0000 and is NOT in this map - it is masked
   * ROM and cannot be erased by anything we do here. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

/* The stock firmware's stack starts at 0x20002B18. We stay below it so that a
 * soft reset into the stock image behaves predictably. */
_stack_start = ORIGIN(RAM) + 0x2800;
