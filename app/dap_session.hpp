// SPDX-License-Identifier: Apache-2.0
#pragma once

// DAP sessions over the radio between two paired probes. The host probe (on the PC's USB)
// forwards the PC's DAP commands over the radio and returns the answers to USB; the
// remote probe (wired to the target) executes them (link_transport.hpp carries the
// messages, link_session.hpp has the session rules).
//
// Roles. With the AUTO setting (the default) a paired probe serves the radio as a remote
// while it has no debug session of its own. DAP_Connect from USB starts a session: the
// probe pings its peer with a new session id, and when the peer answers the session runs
// over the radio until the next DAP_Connect or DAP_Disconnect; otherwise the probe drives
// its own target. A session does not switch between radio and wire on its own, so a peer
// that drops out makes the commands fail quickly instead of reaching another target. The
// LOCAL, HOST and REMOTE settings fix the role. An unpaired probe does not use the radio
// at all: it is a wired probe whatever its role setting, and never executes commands from
// the radio.
//
// While a session runs over the radio the host sends a heartbeat PING after 1 s without
// a command, and ends the session (PING with id 0) on DAP_Disconnect or after 5 s
// without a command from the PC; the remote ends it on that PING, on a new id, or after
// 3 s without a packet, and then stops the sampling the session started.
//
// Context: main task only.
//
// 两块已配对探针之间经无线的 DAP 会话。主机端探针（接电脑 USB）把电脑的 DAP 命令经无线转
// 发，把应答交回 USB；远端探针（接目标）执行它们（消息由 link_transport.hpp 承载，会话规
// 则见 link_session.hpp）。
//
// 角色。设置为 AUTO（默认）时，已配对的探针在没有自己的调试会话时作为远端为无线服务。USB
// 上的 DAP_Connect 开始一次会话：探针用新的会话号 ping 对端，对端有回应则本次会话走无线，
// 直到下一次 DAP_Connect 或 DAP_Disconnect；否则驱动自己的目标。会话中不会自行在无线和有
// 线之间切换，对端掉线时命令快速失败，不会误操作另一个目标。LOCAL、HOST、REMOTE 设置固定
// 角色。未配对的探针完全不用无线：无论角色设置如何都是有线探针，也从不执行无线收到的命
// 令。
//
// 会话走无线期间，主机端在 1 s 没有命令时发心跳 PING，在 DAP_Disconnect 或 5 s 收不到电脑
// 的命令时结束会话（会话号 0 的 PING）；远端收到该 PING、新会话号或 3 s 收不到任何包时结
// 束会话，并停止该会话启动的采样。上下文：只在主任务里调用。

#include <cstdint>

#include "link_transport.hpp"

namespace XrDap
{
/// 探针当前的角色 / What the probe does right now.
enum class RadioRole : uint8_t
{
  LOCAL = 0,   ///< 不用无线，驱动本地目标 / radio idle, the probe drives its own target
  HOST = 1,    ///< 经无线转发 DAP 命令 / forward DAP commands over the radio
  REMOTE = 2,  ///< 执行无线收到的 DAP 命令 / execute DAP commands from the radio
};

/// @brief 设置远端的执行器和会话结束回调 / Set the remote's executor and session-end
///        callback.
/// @note 启动时调用一次，在 DapSessionApplyConfig() 之前 / Call once at start-up, before
///       DapSessionApplyConfig().
void DapSessionInit(LinkExecutor execute, LinkSessionEnd on_remote_session_end);

/// @brief 按设置应用角色、调制方式和配对身份；未配对时关掉射频 / Apply the role setting,
///        modulation and pair identity from the settings; without a pairing the radio is
///        off.
/// @note 阻塞约 22 ms（射频重启）；会话都结束 / Blocks about 22 ms (radio restart); every
///       session ends.
void DapSessionApplyConfig();

/// 当前角色 / The current role.
RadioRole GetRadioRole();

/// @brief 临时改角色，直到下次 DapSessionApplyConfig() / Override the role until the next
///        DapSessionApplyConfig().
/// @return 未配对且 role 不是 LOCAL 时为 false，角色不变 / false without a pairing for
///         any role but LOCAL; the role stays.
bool SetRadioRole(RadioRole role);

/// 射频在用：主循环据此不在等 USB 命令时阻塞 / The radio is in use, so the main loop does
/// not block waiting for USB commands.
bool DapSessionRadioInUse();

/// 本探针的调试会话走无线（主机端）/ This probe's debug session runs over the radio
/// (host).
bool RadioSessionWireless();
/// 远端会话打开（另一块探针经无线驱动本探针）/ A remote session is open (another probe
/// drives this one over the radio).
bool RemoteSessionActive();

/// @brief 处理一条来自 USB 的命令（0x9F 除外）/ Handle one command from USB (not 0x9F).
/// @return 已经无线转发并填好应答时为 true；false 时由 DAP 类在本地执行 / true when it
///         was forwarded over the radio and the answer is filled in; false when the DAP
///         class executes it locally.
/// @note 远端执行对端命令时也会经过 DAP 类的命令钩子，此时返回 false / Commands from the
///       peer also pass the DAP class's hook while the remote executes them; they return
///       false.
bool DapSessionHandleUsbCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len);

/// @brief 把一条命令发给对端并等应答（0x9F 0x30）/ Send one command to the peer and wait
///        for its answer (0x9F 0x30).
/// @return 对端未应答或未配对时为 false / false when the peer did not answer or the probe
///         is not paired.
bool DapSessionForward(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                       uint16_t resp_cap, uint16_t& out_len);

/// @brief 主循环每轮调用：作远端服务，或作主机端发心跳、结束空闲会话 / Call on every pass
///        of the main loop: serve as the remote, or as the host send the heartbeat and
///        end an idle session.
void DapSessionService();

/// @brief 当前命令的应答发出后执行 action / Run `action` once the answer to the command
///        being executed has gone out.
void DapSessionAfterAnswer(void (*action)());
}  // namespace XrDap
