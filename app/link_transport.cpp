// SPDX-License-Identifier: Apache-2.0
#include "link_transport.hpp"

#include "link_protocol.hpp"
#include "link_session.hpp"
#include "sx1281.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
using Link::kHeader;
using Link::kMaxMessage;

// Retries: the first wait covers the command's execution plus the first answer packet;
// every further wait doubles, up to kMaxWaitUs, until kBudgetUs (plus the execution) has
// passed. Each answer packet that arrives extends the wait by kFragmentWaitUs, so a long
// answer is not cut off. A command sent again while the remote still executes it is
// harmless: the remote answers it from its cache.
constexpr uint32_t kFirstWaitUs = 3'000;
constexpr uint32_t kFragmentWaitUs = 3'000;
constexpr uint32_t kMaxWaitUs = 200'000;
constexpr uint32_t kBudgetUs = 1'000'000;
constexpr uint32_t kPingWaitUs = 3'000;

LinkExecutor execute = nullptr;
LinkSessionEnd session_end = nullptr;
bool running = false;
uint8_t host_sequence = 0;

// Remote side.
uint8_t command[kMaxMessage];
uint8_t answer[kMaxMessage];
uint16_t answer_size = 0;
Link::Reassembler commands(command, sizeof(command));
Link::RemoteLease lease;
Link::AnswerCache cache;
bool executing = false;
void (*after_answer)() = nullptr;

uint32_t NowUs() { return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds()); }

uint8_t Fragment() { return static_cast<uint8_t>(Radio().MaxPayload() - kHeader); }

void SendMessage(uint8_t kind, uint8_t sequence, const uint8_t* data, uint16_t size)
{
  struct Message
  {
    uint8_t kind, sequence, fragment;
    const uint8_t* data;
    uint16_t size;
  } message{kind, sequence, Fragment(), data, size};
  Radio().TransmitBurst(
      Link::FragmentCount(size, message.fragment),
      +[](void* ctx, uint8_t index, uint8_t* packet) -> uint8_t
      {
        const auto& m = *static_cast<const Message*>(ctx);
        return Link::BuildFragment(m.kind, m.sequence, m.data, m.size, m.fragment, index,
                                   packet);
      },
      &message);
}

void SendPing(uint8_t kind, uint8_t sequence, uint16_t session)
{
  uint8_t packet[kHeader + Link::kPingPayload];
  Radio().Transmit(packet, Link::BuildPing(kind, sequence, session, packet));
}

// R4: drop what the ended session left behind and tell the session's owner.
void EndSession()
{
  commands.Reset();
  cache.Clear();
  if (session_end != nullptr) session_end();
}
}  // namespace

void LinkInit(LinkExecutor executor, LinkSessionEnd on_session_end)
{
  execute = executor;
  session_end = on_session_end;
}

void LinkStart(bool gfsk, const RadioIdentity& identity)
{
  Radio().Configure(gfsk, identity);
  (void)Radio().Start();  // a failed start recovers in Poll() (sx1281.hpp)
  running = true;
  LinkEndRemoteSession();
}

void LinkStop()
{
  running = false;
  LinkEndRemoteSession();
}

bool LinkRunning() { return running; }

bool LinkPing(uint16_t session, uint32_t timeout_us)
{
  if (!running) return false;
  const uint8_t sequence = ++host_sequence;
  uint8_t packet[Sx1281::kMaxPayload];
  const uint32_t begin = NowUs();
  do
  {
    SendPing(Link::kPing, sequence, session);
    const uint32_t deadline = NowUs() + kPingWaitUs;
    while (static_cast<int32_t>(deadline - NowUs()) > 0)
    {
      uint16_t echoed = 0;
      const int n = Radio().Poll(packet);
      if (n > 0 && packet[0] == Link::kPong && packet[1] == sequence &&
          Link::ParsePing(packet, n, echoed) && echoed == session)
        return true;
    }
  } while (NowUs() - begin < timeout_us);
  return false;
}

bool LinkForward(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap,
                 uint16_t& out_len, uint32_t execution_us, bool more)
{
  if (!running || req_len == 0U || req_len > kMaxMessage) return false;
  const uint8_t sequence = ++host_sequence;
  const uint32_t budget = kBudgetUs + execution_us;
  Link::Reassembler answers(resp, resp_cap);
  uint8_t packet[Sx1281::kMaxPayload];
  uint32_t wait = kFirstWaitUs + execution_us;
  const uint32_t begin = NowUs();
  while (NowUs() - begin < budget)
  {
    SendMessage(Link::kCommand, sequence, req, req_len);
    uint32_t deadline = NowUs() + wait;
    while (static_cast<int32_t>(deadline - NowUs()) > 0)
    {
      const int n = Radio().Poll(packet);
      if (n < kHeader || packet[0] != Link::kAnswer || packet[1] != sequence) continue;
      if (answers.Add(packet, n, Fragment()) == Link::Reassembler::Result::COMPLETE)
      {
        // The next command of the session goes out without the switch out of receive.
        if (more) Radio().PrepareTransmit();
        out_len = answers.size();
        return true;
      }
      deadline = NowUs() + kFragmentWaitUs;
    }
    wait = wait * 2U < kMaxWaitUs ? wait * 2U : kMaxWaitUs;
  }
  return false;
}

void LinkServeRemote()
{
  if (!running) return;
  uint8_t packet[Sx1281::kMaxPayload];
  const int n = Radio().Poll(packet);
  const uint32_t now = NowUs();
  if (n < kHeader)
  {
    if (lease.Tick(now) == Link::RemoteLease::Event::ENDED) EndSession();
    return;
  }
  const uint8_t sequence = packet[1];
  if (packet[0] == Link::kPing)
  {
    uint16_t session = 0;
    if (!Link::ParsePing(packet, n, session)) return;
    if (lease.OnPing(session, now) != Link::RemoteLease::Event::NONE) EndSession();
    SendPing(Link::kPong, sequence, lease.session());
    return;
  }
  if (packet[0] != Link::kCommand || !lease.OnCommand(now)) return;  // R5
  if (commands.Add(packet, n, Fragment()) != Link::Reassembler::Result::COMPLETE) return;

  if (cache.Has(commands.sequence()))
  {
    // The host did not get the answer: send it again instead of executing twice.
    SendMessage(Link::kAnswer, commands.sequence(), answer, answer_size);
    return;
  }
  Radio().PrepareTransmit();  // the switch to FS overlaps the execution
  executing = true;
  answer_size = execute(command, commands.size(), answer, sizeof(answer));
  executing = false;
  cache.Store(commands.sequence());
  SendMessage(Link::kAnswer, commands.sequence(), answer, answer_size);
  if (after_answer != nullptr)
  {
    void (*action)() = after_answer;
    after_answer = nullptr;
    action();
  }
}

void LinkEndRemoteSession()
{
  if (lease.OnPing(0, NowUs()) != Link::RemoteLease::Event::NONE) EndSession();
  commands.Reset();
  cache.Clear();
}

bool LinkRemoteSessionActive() { return running && lease.Active(); }

bool LinkExecuting() { return executing; }

void LinkAfterAnswer(void (*action)())
{
  if (executing)
    after_answer = action;
  else
    action();
}
}  // namespace XrDap
