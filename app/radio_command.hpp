// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP vendor command 0x9F: the radio and settings of the probe that receives it; the
// link forwards it only inside 0x9F 0x30.
//   0x9F 0x00 n data[n]  SX1281 SPI transfer after BUSY -> 0x9F status n rx[n]
//   0x9F 0x01            pins -> 0x9F 0 bits (bit0 BUSY, bit1 DIO1, bit2 DIO2, bit3 DIO3)
//   0x9F 0x02 mask value outputs (bit0 NRESET, bit1 TXEN, bit2 RXEN) -> 0x9F 0
//   0x9F 0x03 ms[2]      wait up to ms for DIO1 high -> 0x9F status us[4]
//   0x9F 0x10 role       override the role until the next restart (0 local, 1 host,
//                        2 remote) -> 0x9F status
//   0x9F 0x11            statistics -> 0x9F 0 role u32[17]: radio TX, RX, CRC errors, TX
//                        timeouts; link commands, retries, failures, duplicates, last and
//                        longest round trip (us); free heap, lowest free heap, free stack;
//                        receive re-entries and radio resets after silence; pings, pongs
//   0x9F 0x12            reset the statistics -> 0x9F 0
//   0x9F 0x13            settings -> 0x9F 0 role-setting modulation paired safe-mode
//                        role pair-state link-id[4] peer-uid[12]
//   0x9F 0x14 setting    store the role setting (0 auto, 1 local, 2 host, 3 remote)
//   0x9F 0x15 modulation store the modulation (0 GFSK, 1 FLRC)
//   0x9F 0x16            forget the pairing
//   0x9F 0x17            timing -> 0x9F 0 u32[9]: SPI transfers, SPI us, BUSY wait us,
//                        TX done wait us; host send us, host wait us; remote receive us,
//                        execute us, reply us (sums since the last reset)
//   0x9F 0x20            start pairing (as holding KEY2 for 3 s) -> 0x9F 0
//   0x9F 0x30 command    send a 0x9F command to the peer -> the peer's answer, or
//                        0x9F 1 when it does not answer
//   0x9F 0x40 addr[4] n  read n bytes of memory -> 0x9F 0 data[n]
//   0x9F 0x41            hang until the watchdog resets the probe (no answer)
// Settings take effect once the answer has been sent. status: 0 OK, 1 timeout, 2 SPI,
// radio or flash error, 3 bad request.
//
// DAP 厂商命令 0x9F：收到它的探针自己的射频和设置；只有包在 0x9F 0x30 里时才经链路
// 转发。格式见上。

#include <cstdint>

namespace XrDap
{
/// DapLinkV2Class command hook: answers 0x9F and hands every other command to the link,
/// which forwards it during a session over the radio.
bool RadioCommandHook(void* ctx, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap, uint16_t& out_len);

/// Answer one 0x9F command (from USB or from the peer) / 处理一条 0x9F 命令。
uint16_t RadioVendorCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap);
}  // namespace XrDap
