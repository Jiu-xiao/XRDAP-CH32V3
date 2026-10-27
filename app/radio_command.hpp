// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP vendor command 0x9F: the radio and settings of the probe that receives it; the
// link forwards it only inside 0x9F 0x30.
//   0x9F 0x10 role       override the role until the settings are applied again (0 local,
//                        1 host, 2 remote) -> 0x9F status (4 when not paired)
//   0x9F 0x13            settings -> 0x9F 0 role-setting modulation paired safe-mode
//                        role pair-state link-id[4] peer-uid[12]
//   0x9F 0x14 setting    store the role setting (0 auto, 1 local, 2 host, 3 remote)
//   0x9F 0x15 modulation store the modulation (0 GFSK, 1 FLRC)
//   0x9F 0x16            forget the pairing
//   0x9F 0x20            start pairing (as holding KEY2 for 3 s) -> 0x9F 0
//   0x9F 0x30 command    send a 0x9F command to the peer -> the peer's answer, or
//                        0x9F 1 when it does not answer, 0x9F 4 when not paired
// Settings take effect once the answer has been sent. status: 0 OK, 1 timeout, 2 flash
// error, 3 bad request, 4 not paired. An unpaired probe does not listen to the radio, so
// commands reach it over USB only.
//
// DAP 厂商命令 0x9F：收到它的探针自己的射频和设置；只有包在 0x9F 0x30 里时才经链路
// 转发。格式见上。

#include <cstdint>

namespace XrDap
{
/// 本探针的厂商命令字节 / This probe's vendor command byte.
inline constexpr uint8_t kProbeCommand = 0x9F;

/// @brief 处理一条 0x9F 命令（来自 USB 或对端），返回应答长度 / Answer one 0x9F command
///        (from USB or from the peer) and return the answer size.
/// @note 主任务；resp_cap 至少 80 字节 / Main task; resp_cap at least 80 bytes.
uint16_t RadioVendorCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap);
}  // namespace XrDap
