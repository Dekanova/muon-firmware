/* 1K = 1 KiBi = 1024 bytes */
MEMORY {
    /* RP2040 embedded flash */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* W25Q080 external flash */
    FLASH : ORIGIN = 0x10000100, LENGTH = 512K - 0x100
    /* Half is reserved for user values */
    USER  : ORIGIN = 0x10080000, LENGTH = 512K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;