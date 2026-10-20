// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_BOOT_MAILBOX_H_
#define XRDAP_BOOT_MAILBOX_H_

/*
 * Boot request mailbox shared by the application and the bootloader. It lives in
 * the RAM the linker script keeps out of every image (the top RAM_TOP_RESERVED
 * bytes of the 64 KB RAM window), so it survives a software reset: the
 * application writes both words and resets, the bootloader consumes them and
 * stays in DFU mode.
 *
 * 应用与 bootloader 共用的启动请求信箱。位于链接脚本保留、任何镜像都不使用的 RAM 顶部，
 * 软件复位后内容保留：应用写入两个字后复位，bootloader 读到后清零并停留在 DFU 模式。
 */

#define XRDAP_BOOT_MAILBOX_ADDR 0x2000FFF0u
#define XRDAP_BOOT_REQUEST_MAGIC0 0x50414458u /* "XDAP" */
#define XRDAP_BOOT_REQUEST_MAGIC1 0x55464442u /* "BDFU" */

#endif /* XRDAP_BOOT_MAILBOX_H_ */
