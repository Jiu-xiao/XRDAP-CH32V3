// SPDX-License-Identifier: Apache-2.0
// Tests of the pure logic on the build machine. The expected values come from the
// specifications in the headers (link_protocol.hpp, link_session.hpp, pair_identity.hpp,
// dap_forwarding.hpp, settings_log.hpp, flash_layout.hpp), which follow the review
// decisions B3, B6 and C4 (F3, F15, F16).
//
// 在编译机上测试纯逻辑。判据来自各头文件里的规范（link_protocol.hpp、link_session.hpp、
// pair_identity.hpp、dap_forwarding.hpp、settings_log.hpp、flash_layout.hpp），它们依据审
// 查决定 B3、B6 和 C4（F3、F15、F16）。

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "dap_forwarding.hpp"
#include "flash_layout.hpp"
#include "link_protocol.hpp"
#include "link_session.hpp"
#include "pair_identity.hpp"
#include "settings_log.hpp"

namespace
{
int failures = 0;
int checks = 0;

#define CHECK(cond)                                               \
  do                                                              \
  {                                                               \
    ++checks;                                                     \
    if (!(cond))                                                  \
    {                                                             \
      ++failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

using namespace XrDap;
using Link::Reassembler;
using Result = Reassembler::Result;

constexpr uint8_t kGfsk = 255 - Link::kHeader;  // fragment sizes
constexpr uint8_t kFlrc = 127 - Link::kHeader;

std::vector<uint8_t> Message(uint16_t size, uint8_t salt)
{
  std::vector<uint8_t> m(size);
  for (uint16_t i = 0; i < size; ++i) m[i] = static_cast<uint8_t>(i * 7U + salt);
  return m;
}

std::vector<std::vector<uint8_t>> Fragments(const std::vector<uint8_t>& m,
                                            uint8_t sequence, uint8_t fragment)
{
  std::vector<std::vector<uint8_t>> out;
  const uint16_t total = static_cast<uint16_t>(m.size());
  for (uint8_t i = 0; i < Link::FragmentCount(total, fragment); ++i)
  {
    std::vector<uint8_t> p(Link::kHeader + fragment);
    p.resize(Link::BuildFragment(Link::kCommand, sequence, m.data(), total, fragment, i,
                                 p.data()));
    out.push_back(p);
  }
  return out;
}

// --- link_protocol.hpp: fragment counts and lengths ---------------------------------
void TestFragmentCount()
{
  CHECK(Link::FragmentCount(0, kGfsk) == 1);
  CHECK(Link::FragmentCount(1, kGfsk) == 1);
  CHECK(Link::FragmentCount(kGfsk, kGfsk) == 1);
  CHECK(Link::FragmentCount(kGfsk + 1, kGfsk) == 2);
  CHECK(Link::FragmentCount(Link::kMaxMessage, kGfsk) == 5);
  CHECK(Link::FragmentCount(Link::kMaxMessage, kFlrc) == 9);
  CHECK(Link::kMaxFragments == 9);
}

// A message comes back byte for byte, in order or not.
void TestRoundTrip()
{
  for (uint8_t fragment : {kGfsk, kFlrc})
  {
    for (uint16_t size : {0, 1, 120, 121, 122, 248, 249, 250, 1023, 1024})
    {
      const auto m = Message(size, 3);
      auto frags = Fragments(m, 9, fragment);
      for (bool reversed : {false, true})
      {
        uint8_t buffer[Link::kMaxMessage];
        Reassembler r(buffer, sizeof(buffer));
        Result last = Result::INCOMPLETE;
        for (size_t k = 0; k < frags.size(); ++k)
        {
          const auto& p = frags[reversed ? frags.size() - 1 - k : k];
          last = r.Add(p.data(), static_cast<int>(p.size()), fragment);
          if (k + 1 < frags.size()) CHECK(last == Result::INCOMPLETE);
        }
        CHECK(last == Result::COMPLETE);
        CHECK(r.size() == size);
        CHECK(r.sequence() == 9);
        CHECK(size == 0 || std::memcmp(buffer, m.data(), size) == 0);
      }
    }
  }
}

// F16: fragments that do not cover `total` exactly are dropped, never completed.
void TestReassemblyRejects()
{
  uint8_t buffer[Link::kMaxMessage];
  std::memset(buffer, 0xAA, sizeof(buffer));
  Reassembler r(buffer, sizeof(buffer));
  uint8_t p[255] = {Link::kCommand, 1, 0, 1, 0, 0};

  // One fragment claiming 500 bytes but carrying 10 (old code completed it with stale
  // bytes behind).
  p[4] = 500 & 0xFF;
  p[5] = 500 >> 8;
  CHECK(r.Add(p, Link::kHeader + 10, kGfsk) == Result::REJECTED);
  // Count 2 for a total that fits one fragment.
  p[3] = 2;
  p[4] = 100;
  p[5] = 0;
  CHECK(r.Add(p, Link::kHeader + 100, kGfsk) == Result::REJECTED);
  // A short non-last fragment: total 300 in two, first carries 100 instead of 249.
  p[3] = 2;
  p[4] = 300 & 0xFF;
  p[5] = 300 >> 8;
  CHECK(r.Add(p, Link::kHeader + 100, kGfsk) == Result::REJECTED);
  // A last fragment longer than the rest of the message.
  p[2] = 1;
  CHECK(r.Add(p, Link::kHeader + 100, kGfsk) == Result::REJECTED);
  // Index outside the count, too many fragments, a total above the buffer.
  p[2] = 2;
  CHECK(r.Add(p, Link::kHeader + 51, kGfsk) == Result::REJECTED);
  p[2] = 0;
  p[3] = Link::kMaxFragments + 1;
  CHECK(r.Add(p, Link::kHeader, kGfsk) == Result::REJECTED);
  Reassembler small(buffer, 100);
  p[3] = 1;
  p[4] = 101;
  p[5] = 0;
  CHECK(small.Add(p, Link::kHeader + 101, kGfsk) == Result::REJECTED);
  // A packet shorter than the header.
  CHECK(r.Add(p, 3, kGfsk) == Result::REJECTED);
}

// A fragment of another message drops the half-assembled one; repeats do not complete it.
void TestReassemblyRestart()
{
  uint8_t buffer[Link::kMaxMessage];
  Reassembler r(buffer, sizeof(buffer));
  const auto a = Fragments(Message(500, 1), 1, kGfsk);  // 3 fragments
  const auto b = Fragments(Message(500, 2), 2, kGfsk);
  CHECK(r.Add(a[0].data(), static_cast<int>(a[0].size()), kGfsk) == Result::INCOMPLETE);
  CHECK(r.Add(a[0].data(), static_cast<int>(a[0].size()), kGfsk) == Result::INCOMPLETE);
  CHECK(r.Add(b[1].data(), static_cast<int>(b[1].size()), kGfsk) == Result::INCOMPLETE);
  CHECK(r.Add(a[1].data(), static_cast<int>(a[1].size()), kGfsk) == Result::INCOMPLETE);
  CHECK(r.Add(a[2].data(), static_cast<int>(a[2].size()), kGfsk) == Result::INCOMPLETE);
  CHECK(r.Add(a[0].data(), static_cast<int>(a[0].size()), kGfsk) == Result::COMPLETE);
  CHECK(r.sequence() == 1);
}

void TestPing()
{
  uint8_t p[16];
  const uint8_t n = Link::BuildPing(Link::kPing, 4, 0xBEEF, p);
  CHECK(n == Link::kHeader + Link::kPingPayload);
  CHECK(p[0] == Link::kPing && p[1] == 4 && p[3] == 1);
  uint16_t session = 0;
  CHECK(Link::ParsePing(p, n, session) && session == 0xBEEF);
  CHECK(!Link::ParsePing(p, n - 1, session));
  CHECK(!Link::ParsePing(p, Link::kHeader, session));
}

// --- link_session.hpp: remote lease (R1..R5) ----------------------------------------
void TestRemoteLease()
{
  using Event = Link::RemoteLease::Event;
  Link::RemoteLease lease;
  CHECK(!lease.Active());
  CHECK(!lease.OnCommand(0));                // R5
  CHECK(lease.OnPing(0, 0) == Event::NONE);  // no session, nothing ends
  CHECK(lease.OnPing(7, 10) == Event::REPLACED);
  CHECK(lease.Active() && lease.session() == 7);
  CHECK(lease.OnPing(7, 20) == Event::NONE);  // the heartbeat
  CHECK(lease.OnCommand(30));
  CHECK(lease.OnPing(8, 40) == Event::REPLACED);  // R2
  CHECK(lease.session() == 8);
  CHECK(lease.OnPing(0, 50) == Event::ENDED);  // R1
  CHECK(!lease.Active());
  CHECK(!lease.OnCommand(60));  // R5

  // R3: every host packet renews the lease; kLeaseUs of silence ends it.
  CHECK(lease.OnPing(9, 1'000) == Event::REPLACED);
  CHECK(lease.Tick(1'000 + Link::kLeaseUs - 1) == Event::NONE);
  CHECK(lease.OnCommand(2'000'000));
  CHECK(lease.Tick(2'000'000 + Link::kLeaseUs - 1) == Event::NONE);
  CHECK(lease.Tick(2'000'000 + Link::kLeaseUs) == Event::ENDED);
  CHECK(!lease.Active());
  CHECK(lease.Tick(9'000'000) == Event::NONE);
  // Time wraps around 32 bits.
  CHECK(lease.OnPing(10, 0xFFFFFF00U) == Event::REPLACED);
  CHECK(lease.Tick(0x100U) == Event::NONE);
}

// F3: the answer cache is per session; a new session cannot get an old answer.
void TestAnswerCacheAcrossSessions()
{
  using Event = Link::RemoteLease::Event;
  Link::RemoteLease lease;
  Link::AnswerCache cache;
  auto ping = [&](uint16_t id, uint32_t now)
  {
    if (lease.OnPing(id, now) != Event::NONE) cache.Clear();  // R4, as link_transport
  };
  ping(5, 0);
  cache.Store(1);
  CHECK(cache.Has(1));  // a repeated command of the same session
  CHECK(!cache.Has(2));
  ping(5, 10);  // the heartbeat keeps the answer
  CHECK(cache.Has(1));
  ping(6, 20);  // the host restarted: new id, its sequence starts again at 1
  CHECK(!cache.Has(1));
  cache.Store(1);
  if (lease.Tick(20 + Link::kLeaseUs) != Event::NONE) cache.Clear();
  CHECK(!cache.Has(1));
}

// --- link_session.hpp: host session (H1..H3) ----------------------------------------
void TestHostSession()
{
  using Action = Link::HostSession::Action;
  Link::HostSession host(0xFFFE);
  CHECK(!host.Active());
  CHECK(host.Tick(0) == Action::NONE);
  CHECK(host.OnCommand(0));  // H1
  const uint16_t first = host.session();
  CHECK(first != 0);
  CHECK(!host.OnCommand(10));
  CHECK(host.session() == first);
  host.OnSent(10);
  CHECK(host.Tick(10 + Link::kHeartbeatUs - 1) == Action::NONE);
  CHECK(host.Tick(10 + Link::kHeartbeatUs) == Action::HEARTBEAT);  // H2
  CHECK(host.Tick(10 + Link::kHeartbeatUs + 1) == Action::NONE);
  CHECK(host.Tick(10 + Link::kHostIdleUs) == Action::END);  // H3, idle
  CHECK(!host.Active());
  CHECK(host.OnCommand(20'000'000));
  const uint16_t second = host.session();
  CHECK(second != 0 && second != first);
  CHECK(host.OnDisconnect());  // H3, DAP_Disconnect
  CHECK(!host.OnDisconnect());
  // Ids stay non-zero across the 16-bit wrap.
  bool nonzero = true;
  for (int i = 0; i < 70'000 && nonzero; ++i)
  {
    nonzero = host.OnCommand(30'000'000) && host.session() != 0;
    (void)host.OnDisconnect();
  }
  CHECK(nonzero);
}

// --- pair_identity.hpp -----------------------------------------------------------------
void TestPairIdentity()
{
  const uint8_t a[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  const uint8_t b[12] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
  CHECK(LinkId(a, b, 12) == LinkId(b, a, 12));
  CHECK(LinkId(a, b, 12) != 0);
  CHECK(LinkId(a, b, 12) != LinkId(a, a, 12));
  for (uint32_t id = 1; id < 5000; id = id * 3 + 1)
  {
    const RadioIdentity x = PairIdentity(id);
    CHECK(x.frequency_hz >= 2'403'000'000U && x.frequency_hz <= 2'479'000'000U);
    const uint32_t word = static_cast<uint32_t>(x.sync_word[0]) << 24 |
                          static_cast<uint32_t>(x.sync_word[1]) << 16 |
                          static_cast<uint32_t>(x.sync_word[2]) << 8 | x.sync_word[3];
    const int transitions =
        __builtin_popcount(word ^ (word >> 1)) - static_cast<int>(word >> 31);
    CHECK(transitions >= 10 && transitions <= 22);
    const RadioIdentity y = PairIdentity(id);
    CHECK(std::memcmp(&x, &y, sizeof(x)) == 0);
  }
  const RadioIdentity d = DefaultIdentity();
  CHECK(d.frequency_hz == 2'440'000'000U && d.crc_seed == 0xFFFF);
  CHECK(std::memcmp(d.sync_word, "XRDP", 4) == 0);
}

// --- dap_forwarding.hpp ----------------------------------------------------------------
void TestExecutionUs()
{
  const uint8_t transfer[] = {0x05, 0, 10};
  CHECK(ExecutionUs(transfer, sizeof(transfer), 1'000'000) == 2U * 46U * 10U);
  const uint8_t block[] = {0x06, 0, 0x00, 0x01, 0x0F};  // 256 transactions
  CHECK(ExecutionUs(block, sizeof(block), 4'000'000) == 2U * 46U * 256U / 4U);
  const uint8_t delay[] = {0x09, 0xE8, 0x03};  // 1000 us
  CHECK(ExecutionUs(delay, sizeof(delay), 1'000'000) == 1000U);
  const uint8_t pins[] = {0x10, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF};  // wait capped at 3 s
  CHECK(ExecutionUs(pins, sizeof(pins), 1'000'000) == 3'000'000U);
  const uint8_t reset[] = {0x0A};
  CHECK(ExecutionUs(reset, sizeof(reset), 1'000'000) == 500'000U);
  const uint8_t info[] = {0x00, 0x01};
  CHECK(ExecutionUs(info, sizeof(info), 1'000'000) == 0U);
  const uint8_t clock[] = {0x11, 0x40, 0x42, 0x0F, 0x00};  // 1 MHz
  CHECK(SwjClockHz(clock, sizeof(clock)) == 1'000'000U);
  CHECK(SwjClockHz(info, sizeof(info)) == 0U);
}

void TestSettingsReplay()
{
  SettingsReplay replay;
  for (uint8_t i = 0; i < SettingsReplay::kSettings; ++i) CHECK(replay.Size(i) == 0);
  const uint8_t clock1[] = {0x11, 1, 0, 0, 0};
  const uint8_t clock2[] = {0x11, 2, 0, 0, 0};
  const uint8_t config[] = {0x04, 0, 0x40, 0, 0, 0};
  const uint8_t swd[] = {0x13, 0};
  const uint8_t transfer[] = {0x05, 0, 1, 0x02};
  replay.Note(clock1, sizeof(clock1));
  replay.Note(transfer, sizeof(transfer));
  replay.Note(clock2, sizeof(clock2));
  replay.Note(config, sizeof(config));
  replay.Note(swd, sizeof(swd));
  CHECK(replay.Size(0) == sizeof(clock2) && replay.Data(0)[1] == 2);  // the last one kept
  CHECK(replay.Size(1) == sizeof(config));
  CHECK(replay.Size(2) == sizeof(swd));
  const uint8_t too_long[9] = {0x11};
  replay.Note(too_long, sizeof(too_long));
  CHECK(replay.Size(0) == sizeof(clock2));
}

// --- settings_log.hpp ------------------------------------------------------------------
void Erase(SettingsLog::Record* slots, uint32_t from, uint32_t count)
{
  for (uint32_t i = from; i < from + count; ++i)
  {
    uint32_t* words = reinterpret_cast<uint32_t*>(&slots[i]);
    for (size_t w = 0; w < sizeof(SettingsLog::Record) / 4; ++w)
      words[w] = SettingsLog::kErasedWord;
  }
}

void TestSettingsLog()
{
  constexpr uint32_t kSlots = 32,
                     kPerPage = 4;  // 2 KB of 64-byte records, 256-byte pages
  SettingsLog::Record slots[kSlots];
  Erase(slots, 0, kSlots);
  SettingsLog::Head head = SettingsLog::Find(slots, kSlots);
  CHECK(!head.found);
  CHECK(SettingsLog::NextSlot(head, slots, kSlots, kPerPage) == 0);

  // A save writes the next slot; a page start is erased first, as SaveConfig() does.
  auto save = [&](uint8_t role)
  {
    const uint32_t slot = SettingsLog::NextSlot(head, slots, kSlots, kPerPage);
    if (slot % kPerPage == 0) Erase(slots, slot, kPerPage);
    Config c;
    c.role = static_cast<RoleSetting>(role);
    slots[slot] = SettingsLog::Make(c, head.sequence + 1);
    head = SettingsLog::Find(slots, kSlots);
    return slot;
  };
  for (uint32_t i = 0; i < kSlots + 5; ++i)
  {
    const uint32_t slot = save(static_cast<uint8_t>(i % 4));
    CHECK(slot == i % kSlots);  // wraps around at the end
    CHECK(head.found && head.slot == slot && head.sequence == i + 1);
    CHECK(slots[head.slot].config.role == static_cast<RoleSetting>(i % 4));
  }

  // A torn write (power loss) leaves the previous record current.
  const SettingsLog::Head before = head;
  const uint32_t next = SettingsLog::NextSlot(head, slots, kSlots, kPerPage);
  slots[next] = SettingsLog::Make(Config{}, head.sequence + 1);
  slots[next].crc ^= 1U;
  head = SettingsLog::Find(slots, kSlots);
  CHECK(head.found && head.slot == before.slot && head.sequence == before.sequence);

  // A slot in the middle of a page that is not erased moves the write to the next page.
  Erase(slots, 0, kSlots);
  head = {};
  CHECK(save(0) == 0);
  slots[1].magic = 0;  // something else wrote here
  CHECK(SettingsLog::NextSlot(head, slots, kSlots, kPerPage) == kPerPage);
}

// --- flash_layout.hpp: F15 -------------------------------------------------------------
void TestFlashLayout()
{
  using namespace Layout;
  CHECK(kConfigOffset == 0x2F700U && kSealOffset == 0x2FF00U);
  CHECK(!OverlapsConfig(0, kConfigOffset));     // the largest image
  CHECK(OverlapsConfig(0, kConfigOffset + 1));  // one byte more
  CHECK(OverlapsConfig(kConfigOffset - 1, 2));
  CHECK(OverlapsConfig(kSealOffset - 1, 1));
  CHECK(!OverlapsConfig(kSealOffset, 256));  // the seal itself
  CHECK(!OverlapsConfig(kConfigOffset, 0));
  CHECK(OverlapsConfig(0xFFFFFF00U, 0x200U) == false);  // no wrap-around
}
}  // namespace

int main()
{
  TestFragmentCount();
  TestRoundTrip();
  TestReassemblyRejects();
  TestReassemblyRestart();
  TestPing();
  TestRemoteLease();
  TestAnswerCacheAcrossSessions();
  TestHostSession();
  TestPairIdentity();
  TestExecutionUs();
  TestSettingsReplay();
  TestSettingsLog();
  TestFlashLayout();
  std::printf("%d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
