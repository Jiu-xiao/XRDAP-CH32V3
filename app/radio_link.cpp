// SPDX-License-Identifier: Apache-2.0
#include "radio_link.hpp"

#include <cstring>

#include "pairing.hpp"
#include "sx1281.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
constexpr uint8_t kCommand = 1, kAnswer = 2, kPing = 3, kPong = 4;
// FLRC packets carry at least 6 bytes, so the header alone already fills a packet.
constexpr uint8_t kHeader = 6;
constexpr uint16_t kMaxMessage = 1024;
// FLRC's 127-byte packets give the most fragments.
constexpr uint8_t kMaxFragments = (kMaxMessage + (127 - kHeader) - 1) / (127 - kHeader);
static_assert(kMaxFragments <= 16, "the fragment mask has 16 bits");
// Retries: the first wait covers the command's estimated execution plus the first
// answer packet; every further wait doubles, up to kMaxWaitUs, until kBudgetUs (plus
// the command's own waits) has passed. Each answer packet that arrives extends the
// wait by kFragmentWaitUs, so a long answer is not cut off. A command sent again while
// the remote still executes it is harmless: the remote answers it from its cache.
constexpr uint32_t kFirstWaitUs = 3'000;
constexpr uint32_t kFragmentWaitUs = 3'000;
constexpr uint32_t kMaxWaitUs = 200'000;
constexpr uint32_t kBudgetUs = 1'000'000;
constexpr uint32_t kDefaultSwdHz = 500'000;  // libxr's SWD default until SWJ_Clock
// A ping is sent again every kPingWaitUs until kPingBudgetUs has passed.
constexpr uint32_t kPingWaitUs = 3'000;
constexpr uint32_t kPingBudgetUs = 30'000;
// A wired session that receives no command for this long ends, and the probe serves
// the radio again (a tool that quit without DAP_Disconnect).
constexpr uint32_t kLocalSessionIdleUs = 5'000'000;
constexpr uint32_t kQuietUs = 1'000'000;

constexpr uint8_t kDapInfo = 0x00, kDapConnect = 0x02, kDapDisconnect = 0x03;
constexpr uint8_t kDapTransfer = 0x05, kDapTransferBlock = 0x06, kDapDelay = 0x09;
constexpr uint8_t kDapResetTarget = 0x0A, kDapSwjPins = 0x10, kDapSwjClock = 0x11;
constexpr uint8_t kDapQueueCommands = 0x7E, kDapExecuteCommands = 0x7F;
constexpr uint8_t kDapTransferConfigure = 0x04, kDapSwdConfigure = 0x13;

// Settings a tool may send before DAP_Connect, while the probe still runs them locally;
// they are sent to the peer when a session starts over the radio.
struct SettingCommand
{
  uint8_t id;
  uint8_t size;
  uint8_t data[8];
};
SettingCommand settings[] = {{kDapSwjClock, 0, {}},
                             {kDapTransferConfigure, 0, {}},
                             {kDapSwdConfigure, 0, {}}};

RoleSetting setting = RoleSetting::AUTO;
RadioLinkStats stats;
DapExecutor execute = nullptr;

// Sessions in the AUTO setting.
bool session_wireless = false;
bool session_local = false;
uint32_t last_usb_us = 0;
// The last forwarded command got no answer, or the peer has been quiet for kQuietUs:
// the next command pings first and fails fast when the peer does not answer.
bool peer_lost = false;
uint32_t heard_peer_us = 0;

uint8_t host_sequence = 0;
uint32_t swd_hz = kDefaultSwdHz;  // last SWJ_Clock the host forwarded

// Remote side: the command being reassembled and the last answer sent.
uint8_t command[kMaxMessage];
uint8_t answer[kMaxMessage];
uint16_t command_size = 0, answer_size = 0;
uint16_t received_mask = 0;
uint8_t command_sequence = 0, command_fragments = 0;
uint8_t answered_sequence = 0;
bool answer_valid = false;
bool executing = false;
void (*after_answer)() = nullptr;

uint32_t NowUs() { return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds()); }

uint8_t Fragment() { return static_cast<uint8_t>(Radio().MaxPayload() - kHeader); }

void SendMessage(uint8_t kind, uint8_t sequence, const uint8_t* data, uint16_t size)
{
  struct Message
  {
    uint8_t kind, sequence, count, fragment;
    const uint8_t* data;
    uint16_t size;
  } message{kind, sequence, 1, Fragment(), data, size};
  if (size != 0U)
    message.count = static_cast<uint8_t>((size + message.fragment - 1U) / message.fragment);
  Radio().TransmitBurst(
      message.count,
      +[](void* ctx, uint8_t index, uint8_t* packet) -> uint8_t
      {
        const auto& m = *static_cast<const Message*>(ctx);
        const uint16_t offset = static_cast<uint16_t>(index * m.fragment);
        const uint8_t length = static_cast<uint8_t>(
            m.size - offset < m.fragment ? m.size - offset : m.fragment);
        packet[0] = m.kind;
        packet[1] = m.sequence;
        packet[2] = index;
        packet[3] = m.count;
        packet[4] = static_cast<uint8_t>(m.size);
        packet[5] = static_cast<uint8_t>(m.size >> 8);
        if (length != 0U) std::memcpy(packet + kHeader, m.data + offset, length);
        return static_cast<uint8_t>(kHeader + length);
      },
      &message);
}

/// Collects one fragment into a message; true once every fragment is in.
bool Collect(const uint8_t* packet, int size, uint8_t* message, uint16_t capacity,
             uint16_t& mask, uint16_t& message_size)
{
  const uint8_t index = packet[2], count = packet[3];
  const uint16_t total = static_cast<uint16_t>(packet[4] | packet[5] << 8);
  const uint16_t length = static_cast<uint16_t>(size - kHeader);
  const uint16_t offset = static_cast<uint16_t>(index * Fragment());
  if (count == 0U || count > kMaxFragments || index >= count || total > capacity ||
      offset + length > total)
    return false;
  if (length != 0U) std::memcpy(message + offset, packet + kHeader, length);
  mask |= static_cast<uint16_t>(1U << index);
  message_size = total;
  return mask == static_cast<uint16_t>((1U << count) - 1U);
}

/// Time the remote may spend on the command itself (its waits, or SWD transactions at
/// the current clock, twice over for margin).
uint32_t ExecutionUs(const uint8_t* req, uint16_t len)
{
  uint32_t transactions = 0;
  if (req[0] == kDapTransfer && len >= 3) transactions = req[2];
  if (req[0] == kDapTransferBlock && len >= 4)
    transactions = static_cast<uint32_t>(req[2] | req[3] << 8);
  uint32_t us = static_cast<uint32_t>(2ULL * 46U * transactions * 1'000'000ULL / swd_hz);
  if (req[0] == kDapDelay && len >= 3) us += static_cast<uint32_t>(req[1] | req[2] << 8);
  if (req[0] == kDapSwjPins && len >= 7)
  {
    uint32_t wait = 0;
    std::memcpy(&wait, req + 3, sizeof(wait));
    us += wait < 3'000'000U ? wait : 3'000'000U;
  }
  if (req[0] == kDapResetTarget || req[0] == kDapQueueCommands ||
      req[0] == kDapExecuteCommands)
    us += 500'000;
  return us;
}

void Fail(const uint8_t* req, uint8_t* resp, uint16_t& out_len)
{
  ++stats.failures;
  resp[0] = req[0];
  resp[1] = 0xFF;  // DAP_ERROR
  out_len = 2;
}

void ResetRemoteState()
{
  received_mask = 0;
  answer_valid = false;
}
}  // namespace

void RadioLinkInit(DapExecutor executor) { execute = executor; }

void RadioLinkApplyConfig()
{
  const Config& config = GetConfig();
  setting = config.role;
  Radio().Configure(config.modulation == Modulation::GFSK,
                    config.paired != 0U ? PairIdentity(config.link_id)
                                        : Sx1281::DefaultIdentity());
  if (setting != RoleSetting::LOCAL) Radio().Start();
  session_wireless = session_local = peer_lost = false;
  ResetRemoteState();
}

RadioRole GetRadioRole()
{
  switch (setting)
  {
    case RoleSetting::LOCAL:
      return RadioRole::LOCAL;
    case RoleSetting::HOST:
      return RadioRole::HOST;
    case RoleSetting::REMOTE:
      return RadioRole::REMOTE;
    case RoleSetting::AUTO:
    default:
      return session_wireless ? RadioRole::HOST
             : session_local  ? RadioRole::LOCAL
                              : RadioRole::REMOTE;
  }
}

void SetRadioRole(RadioRole role)
{
  setting = role == RadioRole::HOST     ? RoleSetting::HOST
            : role == RadioRole::REMOTE ? RoleSetting::REMOTE
                                        : RoleSetting::LOCAL;
  if (setting != RoleSetting::LOCAL) Radio().Start();
  session_wireless = session_local = peer_lost = false;
  ResetRemoteState();
}

bool RadioSessionWireless() { return GetRadioRole() == RadioRole::HOST; }

const RadioLinkStats& GetRadioLinkStats() { return stats; }

void ResetRadioLinkStats()
{
  stats = {};
  Sx1281& radio = Radio();
  radio.tx_packets = radio.rx_packets = radio.crc_errors = radio.tx_timeouts = 0;
  radio.rearms = radio.restarts = 0;
  radio.spi_transfers = radio.spi_us = radio.busy_us = radio.tx_wait_us = 0;
}

bool RadioLinkPing(uint32_t timeout_us)
{
  if (setting == RoleSetting::LOCAL) return false;
  ++stats.pings;
  const uint8_t sequence = ++host_sequence;
  uint8_t packet[Sx1281::kMaxPayload];
  const uint32_t begin = NowUs();
  while (NowUs() - begin < timeout_us)
  {
    SendMessage(kPing, sequence, nullptr, 0);
    const uint32_t deadline = NowUs() + kPingWaitUs;
    while (static_cast<int32_t>(deadline - NowUs()) > 0)
    {
      const int n = Radio().Poll(packet);
      if (n >= kHeader && packet[0] == kPong && packet[1] == sequence)
      {
        ++stats.pongs;
        heard_peer_us = NowUs();
        return true;
      }
    }
  }
  return false;
}

bool RadioLinkForward(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap, uint16_t& out_len)
{
  if (req_len == 0U || req_len > kMaxMessage) return false;
  if (req[0] == kDapSwjClock && req_len >= 5)
  {
    uint32_t hz = 0;
    std::memcpy(&hz, req + 1, sizeof(hz));
    if (hz != 0U) swd_hz = hz;
  }
  ++stats.commands;
  if (peer_lost || NowUs() - heard_peer_us >= kQuietUs)
  {
    if (!RadioLinkPing(kPingBudgetUs))
    {
      Fail(req, resp, out_len);
      return false;
    }
    peer_lost = false;
  }
  const uint8_t sequence = ++host_sequence;
  const uint32_t execution = ExecutionUs(req, req_len);
  const uint32_t budget = kBudgetUs + execution;

  uint8_t packet[Sx1281::kMaxPayload];
  uint16_t mask = 0, size = 0;
  uint32_t wait = kFirstWaitUs + execution;
  const uint32_t begin = NowUs();
  for (bool first = true; NowUs() - begin < budget; first = false)
  {
    if (!first) ++stats.retries;
    const uint32_t send_begin = NowUs();
    SendMessage(kCommand, sequence, req, req_len);
    const uint32_t sent = NowUs();
    stats.send_us += sent - send_begin;
    uint32_t deadline = sent + wait;
    while (static_cast<int32_t>(deadline - NowUs()) > 0)
    {
      const int n = Radio().Poll(packet);
      if (n < kHeader || packet[0] != kAnswer || packet[1] != sequence) continue;
      if (Collect(packet, n, resp, resp_cap, mask, size))
      {
        // The next command of the session is sent without the switch out of receive.
        if (req[0] != kDapDisconnect) Radio().PrepareTransmit();
        heard_peer_us = NowUs();
        stats.wait_us += heard_peer_us - sent;
        stats.last_rtt_us = NowUs() - begin;
        if (stats.last_rtt_us > stats.max_rtt_us) stats.max_rtt_us = stats.last_rtt_us;
        out_len = size;
        return true;
      }
      deadline = NowUs() + kFragmentWaitUs;
    }
    wait = wait * 2U < kMaxWaitUs ? wait * 2U : kMaxWaitUs;
  }
  peer_lost = true;
  Fail(req, resp, out_len);
  return false;
}

bool RadioLinkHandleUsbCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                               uint16_t resp_cap, uint16_t& out_len)
{
  // A command from the peer also passes the DAP class's hook: execute it here.
  if (executing) return false;
  last_usb_us = NowUs();
  switch (setting)
  {
    case RoleSetting::HOST:
      RadioLinkForward(req, req_len, resp, resp_cap, out_len);
      return true;
    case RoleSetting::LOCAL:
    case RoleSetting::REMOTE:
      return false;
    case RoleSetting::AUTO:
    default:
      break;
  }
  // DAP_Info is the same on both probes; tools ask it before they connect.
  if (req_len == 0U || req[0] == kDapInfo) return false;
  if (req[0] == kDapConnect)
  {
    peer_lost = false;
    session_wireless = RadioLinkPing(kPingBudgetUs);
    session_local = !session_wireless;
    if (session_wireless)
    {
      uint8_t ignored[16];
      uint16_t ignored_len = 0;
      for (const SettingCommand& c : settings)
        if (c.size != 0U) RadioLinkForward(c.data, c.size, ignored, sizeof(ignored), ignored_len);
    }
  }
  for (SettingCommand& c : settings)
    if (req_len != 0U && req[0] == c.id && req_len <= sizeof(c.data))
    {
      std::memcpy(c.data, req, req_len);
      c.size = static_cast<uint8_t>(req_len);
    }
  if (!session_wireless)
  {
    if (req_len != 0U && req[0] == kDapDisconnect) session_local = false;
    return false;
  }
  RadioLinkForward(req, req_len, resp, resp_cap, out_len);
  if (req[0] == kDapDisconnect) session_wireless = false;
  return true;
}

void RadioLinkService()
{
  switch (setting)
  {
    case RoleSetting::LOCAL:
    case RoleSetting::HOST:
      return;
    case RoleSetting::AUTO:
      if (session_wireless) return;
      if (session_local)
      {
        if (NowUs() - last_usb_us < kLocalSessionIdleUs) return;
        session_local = false;
      }
      break;
    case RoleSetting::REMOTE:
    default:
      break;
  }
  uint8_t packet[Sx1281::kMaxPayload];
  const uint32_t poll_begin = NowUs();
  const int n = Radio().Poll(packet);
  if (n < kHeader) return;
  stats.rx_us += NowUs() - poll_begin;
  const uint8_t sequence = packet[1];
  if (packet[0] == kPing)
  {
    SendMessage(kPong, sequence, nullptr, 0);
    return;
  }
  if (packet[0] != kCommand) return;

  if (answer_valid && sequence == answered_sequence)
  {
    // The host did not get the answer: send it again once its command is complete.
    if (packet[2] == packet[3] - 1U)
    {
      ++stats.duplicates;
      SendMessage(kAnswer, sequence, answer, answer_size);
    }
    return;
  }
  if (sequence != command_sequence || packet[3] != command_fragments)
  {
    command_sequence = sequence;
    command_fragments = packet[3];
    received_mask = 0;
  }
  if (!Collect(packet, n, command, sizeof(command), received_mask, command_size)) return;

  received_mask = 0;
  Radio().PrepareTransmit();  // the switch to FS overlaps the execution
  executing = true;
  const uint32_t exec_begin = NowUs();
  answer_size = execute(command, command_size, answer, sizeof(answer));
  const uint32_t executed = NowUs();
  executing = false;
  answered_sequence = sequence;
  answer_valid = true;
  ++stats.commands;
  SendMessage(kAnswer, sequence, answer, answer_size);
  stats.exec_us += executed - exec_begin;
  stats.reply_us += NowUs() - executed;
  if (after_answer != nullptr)
  {
    void (*action)() = after_answer;
    after_answer = nullptr;
    action();
  }
}

void RadioLinkAfterAnswer(void (*action)())
{
  if (executing)
    after_answer = action;
  else
    action();
}
}  // namespace XrDap
