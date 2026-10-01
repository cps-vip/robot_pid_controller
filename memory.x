MEMORY
{
  /* Default Zynq-7000 layout (DDR / OCM) */
  OCM : ORIGIN = 0x00000000, LENGTH = 192K
  DDR : ORIGIN = 0x00100000, LENGTH = 63M
  OCM_UPPER(rx): ORIGIN = 0xFFFF0000, LENGTH = 64K
  UNCACHED : ORIGIN = ORIGIN(DDR) + LENGTH(DDR), LENGTH = 1M
}

REGION_ALIAS("VECTORS", DDR);
REGION_ALIAS("CODE", DDR);
REGION_ALIAS("DATA", DDR);
REGION_ALIAS("STACKS", DDR);

SECTIONS
{
  /* Uncached memory */
  .uncached (NOLOAD) : ALIGN(4) {
    . = ALIGN(4);
    _sbss_uncached = .;
    *(.uncached .uncached.*);
    . = ALIGN(4);
    _ebss_uncached = .;
  } > UNCACHED
}
