/*
 * Stage-1 trampoline at 0x00000000:
 * do nothing except jump to stage-3 bootloader.
 */

    .section .init,"ax",@progbits
    .global _start
    .align 2
_start:
    .option norvc
    li t0, CH32_STAGE3_BL_BASE
    jalr x0, 0(t0)
    .option rvc
