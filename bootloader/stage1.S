/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Stage 1: the code at address 0, never updated. It reads the boot key, checks the
 * records of the two bootloader slots and jumps to the slot common/boot_slots.hpp
 * selects (RecordValid, SelectSlot; the two must stay identical). It runs straight
 * from reset (HSI 8 MHz, interrupts off), uses no RAM, and puts the GPIO port and its
 * clock back to their reset values before the jump. With no valid slot it stops in a
 * loop; the chip's own loader (BOOT0) or a WCH-Link then has to write the flash.
 *
 * stage1：地址 0 处的代码，永不更新。读 boot 键，检查两个引导槽的记录，跳到
 * common/boot_slots.hpp 选出的槽（RecordValid、SelectSlot，两者必须一致）。复位后直接运行
 * （HSI 8 MHz，中断关闭），不用 RAM，跳转前把 GPIO 端口和它的时钟恢复为复位值。没有有效槽时
 * 停在循环里，只能用芯片自带的下载（BOOT0）或 WCH-Link 写 flash。
 *
 * Per board (cmake): XRDAP_BOOT_KEY_GPIO (port base), XRDAP_BOOT_KEY_PIN,
 * XRDAP_BOOT_KEY_RCC_BIT (the port's bit in RCC_APB2PCENR). The key pulls the pin low.
 */

#define RCC_APB2PCENR 0x40021018
#if XRDAP_BOOT_KEY_PIN < 8
#define KEY_CFGR (XRDAP_BOOT_KEY_GPIO + 0x00)
#define KEY_SHIFT (XRDAP_BOOT_KEY_PIN * 4)
#else
#define KEY_CFGR (XRDAP_BOOT_KEY_GPIO + 0x04)
#define KEY_SHIFT ((XRDAP_BOOT_KEY_PIN - 8) * 4)
#endif
#define KEY_INDR (XRDAP_BOOT_KEY_GPIO + 0x08)
#define KEY_BSHR (XRDAP_BOOT_KEY_GPIO + 0x10)
#define KEY_BCR (XRDAP_BOOT_KEY_GPIO + 0x14)
#define KEY_MASK (1 << XRDAP_BOOT_KEY_PIN)
#define CFG_INPUT_PULL 0x8     /* CNF 10, MODE 00 */
#define CFG_INPUT_FLOATING 0x4 /* reset value */

#define IMAGE_LIMIT (XRDAP_BL_SLOT_SIZE - XRDAP_BL_RECORD_SIZE)
#define SEAL_MAGIC 0x4C414553   /* "SEAL" */
#define RECORD_MAGIC 0x534C4258 /* "XBLS" */
#define ERASED_WORD 0xE339E339

    .section .text.stage1, "ax"
    .globl _start
_start:
    /* Boot key: clock on, input with pull-up, about 150 us to settle, read. */
    li   t0, RCC_APB2PCENR
    lw   t1, 0(t0)
    ori  t1, t1, XRDAP_BOOT_KEY_RCC_BIT
    sw   t1, 0(t0)
    li   t2, KEY_CFGR
    lw   t3, 0(t2)
    li   t4, ~(0xF << KEY_SHIFT)
    and  t3, t3, t4
    li   t4, (CFG_INPUT_PULL << KEY_SHIFT)
    or   t3, t3, t4
    sw   t3, 0(t2)
    li   t5, KEY_MASK
    li   t4, KEY_BSHR
    sw   t5, 0(t4)              /* output latch 1: pull-up */
    li   t6, 400
1:  addi t6, t6, -1
    bnez t6, 1b
    li   t4, KEY_INDR
    lw   a7, 0(t4)
    and  a7, a7, t5
    seqz a7, a7                 /* a7 = 1: key held */

    /* Port pin, output latch and clock back to their reset values. */
    lw   t3, 0(t2)
    li   t4, ~(0xF << KEY_SHIFT)
    and  t3, t3, t4
    li   t4, (CFG_INPUT_FLOATING << KEY_SHIFT)
    or   t3, t3, t4
    sw   t3, 0(t2)
    li   t4, KEY_BCR
    sw   t5, 0(t4)
    lw   t1, 0(t0)
    andi t1, t1, ~XRDAP_BOOT_KEY_RCC_BIT
    sw   t1, 0(t0)

    /* s0/s1: slot A valid and sequence number, s2/s3: slot B. */
    li   a0, XRDAP_BL_SLOT_A_BASE
    jal  ra, check_slot
    mv   s0, a0
    mv   s1, a1
    li   a0, XRDAP_BL_SLOT_B_BASE
    jal  ra, check_slot
    mv   s2, a0
    mv   s3, a1

    /* SelectSlot. */
    or   t0, s0, s2
    beqz t0, no_slot
    beqz s2, start_a
    beqz s0, start_b
    bltu s1, s3, newer_b        /* the larger sequence number is newer; A on a tie */
    bnez a7, start_b            /* A newer: the key turns to B */
    j    start_a
newer_b:
    bnez a7, start_a            /* B newer: the key turns to A */
start_b:
    li   t0, XRDAP_BL_SLOT_B_BASE
    jr   t0
start_a:
    li   t0, XRDAP_BL_SLOT_A_BASE
    jr   t0
no_slot:
    j    no_slot

/*
 * RecordValid. In: a0 = slot base. Out: a0 = 1 and a1 = sequence number when the slot
 * is valid, a0 = 0 otherwise. Uses a2..a4.
 */
check_slot:
    li   a2, IMAGE_LIMIT
    add  a2, a0, a2             /* a2 = the record */
    lw   a3, 0(a2)
    li   a4, SEAL_MAGIC
    bne  a3, a4, invalid
    lw   a3, 8(a2)
    lw   a4, 12(a2)
    xor  a3, a3, a4
    addi a3, a3, 1              /* crc32 ^ crc32_inv == 0xFFFFFFFF */
    bnez a3, invalid
    lw   a3, 16(a2)
    li   a4, RECORD_MAGIC
    bne  a3, a4, invalid
    lw   a1, 20(a2)
    lw   a4, 24(a2)
    xor  a3, a1, a4
    addi a3, a3, 1              /* seq ^ seq_inv == 0xFFFFFFFF */
    bnez a3, invalid
    lw   a3, 28(a2)
    bne  a3, a0, invalid        /* base */
    lw   a3, 4(a2)
    beqz a3, invalid
    li   a4, IMAGE_LIMIT
    bgtu a3, a4, invalid        /* 0 < image_size <= limit */
    lw   a3, 0(a0)
    beqz a3, invalid
    li   a4, -1
    beq  a3, a4, invalid
    li   a4, ERASED_WORD
    beq  a3, a4, invalid
    li   a0, 1
    ret
invalid:
    li   a0, 0
    li   a1, 0
    ret
