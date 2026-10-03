/* SN32F290 memory map.
 *
 * Same values as ../fw/memory.x, which is where they were established:
 * 256 KB of user flash at 0x00000000, 32 KB of SRAM at 0x20000000. Confirmed
 * from the vendor SVD, from ChibiOS-Contrib's SN32F290.ld, and from the stock
 * image's own initial stack pointer (0x20002B18).
 *
 * The ISP boot ROM lives at 0x1FFF0000 and is masked ROM, outside this map.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Kept below the stock firmware's stack top so that a soft reset back into the
 * stock image behaves predictably. */
_stack_start = ORIGIN(RAM) + 0x2800;
