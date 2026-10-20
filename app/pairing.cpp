// SPDX-License-Identifier: Apache-2.0
#include "pairing.hpp"

#include <cstring>

#include XRDAP_BOARD_HEADER
#include "boot_guard.hpp"
#include "ch32_gpio.hpp"
#include "config.hpp"
#include "probe_info.hpp"
#include "radio_link.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
constexpr uint8_t kHello = 5;
constexpr uint8_t kHelloSize = 1 + 2 * kChipUidSize;
constexpr uint32_t kHoldUs = 3'000'000;
constexpr uint32_t kPairUs = 10'000'000;
constexpr uint32_t kLingerUs = 1'000'000;
constexpr uint32_t kHelloEveryUs = 20'000;

PairState state = PairState::IDLE;
bool requested = false;
uint32_t pressed_since = 0;
bool pressed = false, fired = false;

uint32_t NowUs() { return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds()); }

uint32_t Fnv1a(const uint8_t* data, size_t size, uint32_t hash = 2166136261U)
{
  for (size_t i = 0; i < size; ++i) hash = (hash ^ data[i]) * 16777619U;
  return hash;
}

/// Bit changes along a 32-bit word: a sync word needs enough of them to stand out from
/// the preamble and from an idle channel.
int Transitions(uint32_t word)
{
  return __builtin_popcount(word ^ (word >> 1)) - static_cast<int>((word >> 31) & 1U);
}

uint32_t LinkId(const uint8_t* a, const uint8_t* b)
{
  // Smaller UID first, so that both probes compute the same id.
  const bool a_first = std::memcmp(a, b, kChipUidSize) < 0;
  uint32_t hash = Fnv1a(a_first ? a : b, kChipUidSize);
  hash = Fnv1a(a_first ? b : a, kChipUidSize, hash);
  return hash != 0U ? hash : 1U;
}

/// A HELLO packet: own UID, then the peer's (zeros while unknown).
uint8_t Hello(uint8_t* packet, const uint8_t* peer)
{
  packet[0] = kHello;
  std::memcpy(packet + 1, ChipUid(), kChipUidSize);
  if (peer != nullptr)
    std::memcpy(packet + 1 + kChipUidSize, peer, kChipUidSize);
  else
    std::memset(packet + 1 + kChipUidSize, 0, kChipUidSize);
  return kHelloSize;
}

bool KeyDown()
{
  static LibXR::CH32GPIO key(Board::kBootKey.Port(), Board::kBootKey.Mask(),
                             LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP);
  return !key.Read();
}

void RunPairing()
{
  state = PairState::RUNNING;
  Sx1281& radio = Radio();
  radio.Configure(true, Sx1281::DefaultIdentity());
  radio.Start();

  uint8_t peer[kChipUidSize] = {};
  bool peer_known = false, done = false;
  uint8_t packet[Sx1281::kMaxPayload];
  const uint32_t begin = NowUs();
  uint32_t done_at = 0, next_hello = begin;
  // Jitter from the UID keeps two probes from sending in lockstep.
  const uint32_t jitter = Fnv1a(ChipUid(), kChipUidSize) % 8'000U;
  while (NowUs() - begin < kPairUs + kLingerUs)
  {
    BootGuardFeed();
    const uint32_t now = NowUs();
    if (done && now - done_at >= kLingerUs) break;
    if (!done && now - begin >= kPairUs) break;
    if (static_cast<int32_t>(now - next_hello) >= 0)
    {
      radio.Transmit(packet, Hello(packet, peer_known ? peer : nullptr));
      next_hello = NowUs() + kHelloEveryUs + jitter;
    }
    const int n = radio.Poll(packet);
    if (n != kHelloSize || packet[0] != kHello) continue;
    const uint8_t* from = packet + 1;
    const uint8_t* names = packet + 1 + kChipUidSize;
    if (std::memcmp(from, ChipUid(), kChipUidSize) == 0) continue;
    if (!peer_known)
    {
      std::memcpy(peer, from, kChipUidSize);
      peer_known = true;
      next_hello = NowUs();  // tell the peer about it at once
    }
    if (std::memcmp(from, peer, kChipUidSize) != 0) continue;  // a third probe
    if (!done && std::memcmp(names, ChipUid(), kChipUidSize) == 0)
    {
      done = true;
      done_at = NowUs();
    }
  }

  if (done)
  {
    Config config = GetConfig();
    config.paired = 1;
    config.link_id = LinkId(ChipUid(), peer);
    config.modulation = Modulation::GFSK;
    std::memcpy(config.peer_uid, peer, kChipUidSize);
    state = SaveConfig(config) ? PairState::DONE : PairState::FAILED;
  }
  else
  {
    state = PairState::FAILED;
  }
  RadioLinkApplyConfig();
}
}  // namespace

Sx1281::Identity PairIdentity(uint32_t link_id)
{
  Sx1281::Identity identity{};
  // 2403..2479 MHz keeps the 2.4 MHz GFSK bandwidth inside 2400..2483.5 MHz.
  identity.frequency_hz = (2403U + link_id % 77U) * 1'000'000U;
  identity.crc_seed = static_cast<uint16_t>(link_id >> 16 ^ link_id);
  uint32_t word = Fnv1a(reinterpret_cast<const uint8_t*>(&link_id), sizeof(link_id));
  while (Transitions(word) < 10 || Transitions(word) > 22)
    word = Fnv1a(reinterpret_cast<const uint8_t*>(&word), sizeof(word));
  for (int i = 0; i < 4; ++i) identity.sync_word[i] = static_cast<uint8_t>(word >> (24 - 8 * i));
  return identity;
}

void PairingRequest() { requested = true; }

void PairingPoll()
{
  const bool down = KeyDown();
  if (down && !pressed)
  {
    pressed_since = NowUs();
    fired = false;
  }
  if (down && !fired && NowUs() - pressed_since >= kHoldUs)
  {
    requested = true;  // once per press
    fired = true;
  }
  pressed = down;
  if (!requested) return;
  requested = false;
  RunPairing();
}

PairState GetPairState() { return state; }
}  // namespace XrDap
