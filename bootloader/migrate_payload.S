/* SPDX-License-Identifier: Apache-2.0 */
/* boot-chain-parts.bin inside the migration image (migrate.c); MIGRATE_PAYLOAD is its
   path. */
#define STR2(x) #x
#define STR(x) STR2(x)
    .section .rodata.migrate_payload, "a"
    .balign 4
    .globl migrate_payload
    .globl migrate_payload_end
migrate_payload:
    .incbin STR(MIGRATE_PAYLOAD)
migrate_payload_end:
