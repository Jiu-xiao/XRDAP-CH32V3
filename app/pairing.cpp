// SPDX-License-Identifier: Apache-2.0
#include "pairing.hpp"

#include <cstring>

#include XRDAP_BOARD_HEADER
#include "boot_guard.hpp"
#include "ch32_gpio.hpp"
#include "config.hpp"
#include "link_protocol.hpp"
#include "pair_identity.hpp"
#include "probe_info.hpp"
#include "sx1281.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
constexpr uint8_t kHello = Link::kHello;
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

/// Returns true when the settings changed.
bool RunPairing()
{
  state = PairState::RUNNING;
  Sx1281& radio = Radio();
  radio.Configure(true, DefaultIdentity());
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
    config.link_id = LinkId(ChipUid(), peer, kChipUidSize);
    config.modulation = Modulation::GFSK;
    std::memcpy(config.peer_uid, peer, kChipUidSize);
    state = SaveConfig(config) ? PairState::DONE : PairState::FAILED;
  }
  else
  {
    state = PairState::FAILED;
  }
  return true;  // the radio was taken over; the caller applies the settings again
}
}  // namespace

void PairingRequest() { requested = true; }

bool PairingPoll()
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
  if (!requested) return false;
  requested = false;
  return RunPairing();
}

PairState GetPairState() { return state; }
}  // namespace XrDap
