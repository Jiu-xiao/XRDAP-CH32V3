// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP over the SX1281 link between two probes. The host probe (on the PC's USB) sends
// DAP commands over the radio and returns the answers to USB; the remote probe (wired to
// the target) executes the commands it receives and sends the answers back.
//
// Roles. With the AUTO setting (the default) every probe serves the radio as a remote
// while it is not in a debug session of its own. DAP_Connect from USB starts a session:
// the probe pings its peer, and when the peer answers the session runs over the radio
// until the next DAP_Connect or DAP_Disconnect; otherwise the probe drives its own
// target. A session does not switch between radio and wire on its own, so a peer that
// drops out makes the commands fail quickly instead of reaching another target. The
// LOCAL, HOST and REMOTE settings fix the role.
//
// A message (one DAP command or answer, up to 1024 bytes) travels as fragments of up to
// 249 bytes (GFSK) or 121 bytes (FLRC), each a radio packet with a 6-byte header:
//   [0] kind  [1] sequence  [2] fragment index  [3] fragment count
//   [4..5] message size, little endian
// Kinds: 1 command, 2 answer, 3 ping, 4 pong (the pairing packets use 5, see pairing.hpp).
// The host repeats a command with the same sequence number when the answer does not
// arrive in time, waiting twice as long each time; the remote keeps the last answer and
// sends it again for a repeated sequence number instead of executing the command twice.
//
// 两块探针之间经 SX1281 传 DAP。主机端探针（接电脑 USB）把 DAP 命令经无线发出，把应答
// 交回 USB；远端探针（接目标）执行收到的命令并发回应答。
//
// 角色。设置为 AUTO（默认）时，探针在没有自己的调试会话时都作为远端为无线服务。USB 上的
// DAP_Connect 开始一次会话：探针 ping 对端，对端有回应则本次会话走无线，直到下一次
// DAP_Connect 或 DAP_Disconnect；否则驱动自己的目标。会话中不会自行在无线和有线之间
// 切换，对端掉线时命令快速失败，不会误操作另一个目标。LOCAL、HOST、REMOTE 设置固定角色。
//
// 消息（一条 DAP 命令或应答，最长 1024 字节）分成最长 249 字节（GFSK）或 121 字节（FLRC）
// 的分片，每片一个无线包，带 6 字节包头（见上）。应答超时时主机用同一序号重发命令，每次
// 等待时间加倍；远端保留上一条应答，收到重复序号时重发应答，不再执行一遍命令。

#include <cstdint>

#include "config.hpp"

namespace XrDap
{
/// What the probe does right now / 探针当前的角色。
enum class RadioRole : uint8_t
{
  LOCAL = 0,   ///< radio idle, the probe drives its own target / 不用无线，驱动本地目标
  HOST = 1,    ///< forward DAP commands over the radio / 经无线转发 DAP 命令
  REMOTE = 2,  ///< execute DAP commands from the radio / 执行无线收到的 DAP 命令
};

/// Executes one command locally: DAP commands, and 0x9F for this probe's own radio.
using DapExecutor = uint16_t (*)(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                 uint16_t resp_cap);

struct RadioLinkStats
{
  uint32_t commands = 0;     ///< host: commands sent; remote: commands executed
  uint32_t retries = 0;      ///< host: repeated sends after a timeout
  uint32_t failures = 0;     ///< host: commands given up
  uint32_t duplicates = 0;   ///< remote: repeated commands answered from the cache
  uint32_t last_rtt_us = 0;  ///< host: last command round trip
  uint32_t max_rtt_us = 0;   ///< host: longest round trip
  uint32_t pings = 0;        ///< host: pings sent
  uint32_t pongs = 0;        ///< host: pings answered
  // Time spent, summed over all commands (us) / 各阶段累计耗时（微秒）。
  uint32_t send_us = 0;      ///< host: sending commands
  uint32_t wait_us = 0;      ///< host: from the end of sending to the full answer
  uint32_t rx_us = 0;        ///< remote: Poll() calls that returned a command packet
  uint32_t exec_us = 0;      ///< remote: executing commands
  uint32_t reply_us = 0;     ///< remote: sending answers
};

void RadioLinkInit(DapExecutor executor);

/// Apply the role setting, modulation and pair identity from the settings and restart
/// the radio / 按设置应用角色、调制方式和配对身份并重启射频。
void RadioLinkApplyConfig();

RadioRole GetRadioRole();
/// Override the role until the next RadioLinkApplyConfig() (test benches) / 临时改角色。
void SetRadioRole(RadioRole role);
/// True while a debug session runs over the radio / 调试会话走无线时为 true。
bool RadioSessionWireless();

const RadioLinkStats& GetRadioLinkStats();
void ResetRadioLinkStats();

/// Called for every command from USB (except 0x9F), and for the commands from the peer
/// that the DAP class executes: true when the command was handled
/// here (forwarded over the radio), false when the DAP class executes it locally.
bool RadioLinkHandleUsbCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                               uint16_t resp_cap, uint16_t& out_len);

/// Send one command to the peer and wait for its answer; false when no answer came.
bool RadioLinkForward(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap, uint16_t& out_len);

/// Ping the peer; true when it answered within timeout_us / ping 对端。
bool RadioLinkPing(uint32_t timeout_us);

/// Serve the radio as a remote while no session of this probe uses it; returns at once
/// when idle / 本探针没有会话占用无线时作为远端服务；空闲时立即返回。
void RadioLinkService();

/// Run `action` once the answer to the command being executed has been sent (a command
/// that changes the radio must answer first) / 当前命令的应答发出后执行 action。
void RadioLinkAfterAnswer(void (*action)());
}  // namespace XrDap
