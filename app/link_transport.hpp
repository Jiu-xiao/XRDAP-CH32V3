// SPDX-License-Identifier: Apache-2.0
#pragma once

// Message transport of the radio link over the SX1281: fragments, retries, pings and the
// remote side's session lease (packets: link_protocol.hpp, rules: link_session.hpp). It
// knows nothing about DAP; the DAP session layer (dap_session.hpp) decides what to send.
//
// Host side: LinkForward() sends a command and waits for its answer, repeating the
// command with the same sequence number when the answer does not arrive in time, each
// wait twice as long as the last; LinkPing() sends a PING with a session id.
// Remote side: LinkServeRemote() answers PINGs, keeps the session lease, reassembles
// commands, executes them through the executor and answers, or answers a repeated command
// from the cache.
//
// Context: main task only; the radio is not touched anywhere else.
//
// 无线链路在 SX1281 上的消息传输：分片、重发、PING 和远端的会话租期（包格式见
// link_protocol.hpp，规则见 link_session.hpp）。它不了解 DAP；发什么由 DAP 会话层（
// dap_session.hpp）决定。主机端：LinkForward() 发一条命令并等应答，超时后用同一序号重发，
// 每次等待加倍；LinkPing() 发带会话号的 PING。远端：LinkServeRemote() 应答 PING、维护会话
// 租期、拼回命令、经执行器执行并应答，或对重复命令从缓存应答。上下文：只在主任务里调用，
// 射频不在别处使用。

#include <cstdint>

#include "pair_identity.hpp"

namespace XrDap
{
/// @brief 执行对端发来的一条命令，返回应答长度 / Execute one command from the peer and
///        return the answer size.
using LinkExecutor = uint16_t (*)(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                  uint16_t resp_cap);
/// 远端会话结束（R4）时调用，在主任务里 / Called when a remote session ends (R4), on the
/// main task.
using LinkSessionEnd = void (*)();

/// @brief 设置执行器和会话结束回调 / Set the executor and the session-end callback.
/// @note 启动时调用一次 / Call once at start-up.
void LinkInit(LinkExecutor execute, LinkExecutor control, LinkSessionEnd on_session_end);

/// @brief 用这个调制和身份（重新）启动射频；远端会话结束 / (Re)start the radio with this
///        modulation and identity; a remote session ends.
/// @note 阻塞约 22 ms / Blocks about 22 ms.
void LinkStart(bool gfsk, const RadioIdentity& identity);
/// @brief 不用射频：之后 LinkPing/LinkForward 立即失败，LinkServeRemote 不做事 / Stop
///        using the radio: LinkPing/LinkForward then fail at once, LinkServeRemote does
///        nothing.
void LinkStop();
/// 射频在用 / The radio is in use.
bool LinkRunning();

/// @brief 主机端：发 PING(session)，timeout_us 内对端回 PONG 时为 true / Host: send
///        PING(session); true when the peer answers with a PONG within timeout_us.
/// @note 每 3 ms 重发一次，最多阻塞 timeout_us / Repeats every 3 ms; blocks up to
///       timeout_us.
bool LinkPing(uint16_t session, uint32_t timeout_us);

/// @brief 主机端：转发一条命令并等应答 / Host: forward one command and wait for its
///        answer.
/// @param execution_us 远端执行它可能用的时间，加在等待时间上 / Time the remote may take
///        to execute it, added to the waits.
/// @param control 发的是控制请求（包类型 CONTROL），远端交给控制处理函数 / It is a control
///        request (packet kind CONTROL), which the remote hands to its control handler.
/// @param more 之后还有命令：应答到达后射频提前准备发射 / More commands follow: the radio
///        prepares to transmit once the answer is in.
/// @return 收到完整应答时为 true（out_len 为其长度）；1 s 加 execution_us 内没有收到时为
///         false / true with a complete answer (out_len its size); false when none
///         arrived within 1 s plus execution_us.
bool LinkForward(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap,
                 uint16_t& out_len, uint32_t execution_us, bool more, bool control = false);

/// @brief 远端：处理一个收到的包，检查租期；没有包时立即返回 / Remote: handle one
///        received packet and check the lease; returns at once when nothing arrived.
void LinkServeRemote();

/// @brief 远端：结束当前会话（R4），例如设置改变时 / Remote: end the current session
///        (R4), e.g. when the settings change.
void LinkEndRemoteSession();

/// 远端会话打开（主机端在用本探针）/ A remote session is open (a host drives this probe).
bool LinkRemoteSessionActive();

/// 正在执行对端的命令 / A command from the peer is being executed.
bool LinkExecuting();

/// @brief 当前命令的应答发出后执行 action（改射频的命令要先应答）/ Run `action` once the
///        answer to the command being executed has gone out (a command that changes the
///        radio must answer first).
void LinkAfterAnswer(void (*action)());
}  // namespace XrDap
