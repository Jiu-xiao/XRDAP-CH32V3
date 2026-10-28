// SPDX-License-Identifier: Apache-2.0
#pragma once

// The probe's radio and settings, as control port requests (XRDAP-Core
// docs/protocol.md 3.7.1; not DAP commands). A request is `op payload`, the answer
// `op status payload`; status 0xF0 to 0xFF belong to the control port itself.
//   0x10 role       override the role until the settings are applied again (0 local,
//                   1 host, 2 remote) -> status (4 when not paired)
//   0x13            settings -> 0 role-setting modulation paired safe-mode role
//                   pair-state link-id[4] peer-uid[12]
//   0x14 setting    store the role setting (0 auto, 1 local, 2 host, 3 remote)
//   0x15 modulation store the modulation (0 GFSK, 1 FLRC)
//   0x16            forget the pairing
//   0x20            start pairing (as holding KEY2 for 3 s) -> 0
//   0x30 request    send a control request (not another 0x30) to the peer over the
//                   radio -> the peer's answer, or status 1 when it does not answer,
//                   4 when not paired
// Settings take effect once the answer has been sent. status: 0 OK, 1 timeout, 2 flash
// error, 3 bad request, 4 not paired. An unpaired probe does not listen to the radio, so
// requests reach it over USB only.
//
// 探针自己的射频和设置，作为控制口的请求（XRDAP-Core docs/protocol.md 3.7.1），不是 DAP
// 命令。请求是 `op 载荷`，应答是 `op 状态 载荷`；状态 0xF0 到 0xFF 留给控制口自己。格式见
// 上。

#include <cstdint>

namespace XrDap
{
/// @brief 处理一个控制请求（来自 USB 或对端），返回应答长度；签名与 XRDAP-Core 的
///        Control::Handler 相同 / Answer one control request (from USB or from the peer)
///        and return the answer size; the signature of XRDAP-Core's Control::Handler.
/// @note 主任务；resp_cap 至少 80 字节 / Main task; resp_cap at least 80 bytes.
uint16_t RadioControl(void* ctx, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap);
}  // namespace XrDap
