// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_RESERVED_RAM_H_
#define XRDAP_RESERVED_RAM_H_

/*
 * The RAM the linker script keeps out of every image: the top RAM_TOP_RESERVED (0x100)
 * bytes of the RAM window, XRDAP_RAM_SIZE bytes from 0x20000000 (cmake/ch32v307.cmake;
 * 0x2000FF00..0x2000FFFF with the default 64 KB, platform/Link.ld). The startup
 * code does not clear it, so its contents survive a software or watchdog reset. Every
 * word used in it is listed here.
 *
 * 链接脚本留给所有镜像都不使用的 RAM：RAM 窗口（从 0x20000000 起 XRDAP_RAM_SIZE 字节，
 * cmake/ch32v307.cmake）顶部的 RAM_TOP_RESERVED（0x100）字节，默认 64 KB 时为
 * 0x2000FF00..0x2000FFFF（platform/Link.ld）。启动代码不清零，软件复位和看门狗复位
 * 后内容保留。其中用到的每个字都列在这里。
 */

#define XRDAP_RAM_TOP (0x20000000u + (XRDAP_RAM_SIZE))
/*
 * Top - 0x20 (0x2000FFE0 with 64 KB), 2 words: boot guard (app/boot_guard.cpp), a magic
 * and the count of watchdog resets in a row. 启动守护：魔数和连续看门狗复位次数。
 */
#define XRDAP_BOOT_GUARD_ADDR (XRDAP_RAM_TOP - 0x20u)

/*
 * Top - 0x10 (0x2000FFF0 with 64 KB), 2 words: boot request mailbox. The application
 * writes both magics and resets (DFU DETACH); the bootloader consumes them and stays in
 * DFU mode.
 * 启动请求信箱：应用写入两个魔数后复位（DFU DETACH），bootloader 读到后清零并停在 DFU。
 */
#define XRDAP_BOOT_MAILBOX_ADDR (XRDAP_RAM_TOP - 0x10u)
/* Per board (cmake/ch32v307.cmake): "XDAP" "BDFU" for this project's bootloader. */
#ifndef XRDAP_BOOT_REQUEST_MAGIC0
#define XRDAP_BOOT_REQUEST_MAGIC0 0x50414458u /* "XDAP" */
#define XRDAP_BOOT_REQUEST_MAGIC1 0x55464442u /* "BDFU" */
#endif

#endif /* XRDAP_RESERVED_RAM_H_ */
