// SPDX-License-Identifier: Apache-2.0
#pragma once

// What every board with a radio has on top of its transceiver: XRDAP-Core's RadioLink,
// which runs DAP sessions with the paired probe (xrdap/radio/radio_link.hpp), the
// settings in flash (settings_store.hpp), and the boot key: holding it for 3 s starts
// pairing.
//
// 有无线的板子在收发器之上都有的东西：XRDAP-Core 的 RadioLink，它与配对的探针运行
// DAP 会话（xrdap/radio/radio_link.hpp），Flash 里的设置（settings_store.hpp），
// 和 boot 键：按住 3 s 开始配对。

#include <cstdint>

#include "ch32_gpio.hpp"
#include "chip_uid.hpp"
#include "part.hpp"
#include "probe.hpp"
#include "settings_store.hpp"
#include "timebase.hpp"
#include "xrdap/radio/radio_link.hpp"

namespace XrDap
{

template <typename RadioT>
class RadioPart final : public Part
{
 public:
  using Link = Radio::RadioLink<RadioT, Probe>;

  /// @param radio 收发器（xrdap/radio/transceiver.hpp）/ The transceiver
  ///        (xrdap/radio/transceiver.hpp).
  RadioPart(RadioT& radio, Probe& probe)
      : store_(probe.SafeMode()),
        link_(radio, probe, store_, ChipUid()),
        key_(Board::kBootKey.Port(), Board::kBootKey.Mask(),
             LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP)
  {
    link_.Begin(store_.Current(), probe.SafeMode());
  }

  bool Poll() override
  {
    PollKey();
    return link_.Service();
  }

  bool Control(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
               uint16_t& out) override
  {
    return link_.HandleControl(req, len, resp, cap, out);
  }

  /// 一个调试会话正在经无线运行（本机转发，或对方在用本机）/ A debug session runs over
  /// the radio (this probe forwards, or the peer drives it).
  bool Wireless() const
  {
    return link_.CurrentRole() == Radio::Role::HOST || link_.RemoteActive();
  }
  /// 正在配对 / Pairing is running.
  bool Pairing() const { return link_.Pairing() == Radio::PairState::RUNNING; }

 private:
  static constexpr uint32_t kPairHoldMs = 3'000;

  void PollKey()
  {
    // 按键低电平有效；每按一次只触发一次。 / The key is active low; one request per
    // press.
    const bool down = !key_.Read();
    const uint32_t now = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (down && !key_down_)
    {
      key_since_ms_ = now;
      key_fired_ = false;
    }
    if (down && !key_fired_ && now - key_since_ms_ >= kPairHoldMs)
    {
      link_.RequestPairing();
      key_fired_ = true;
    }
    key_down_ = down;
  }

  SettingsStore store_;
  Link link_;
  LibXR::CH32GPIO key_;
  bool key_down_ = false;
  bool key_fired_ = false;
  uint32_t key_since_ms_ = 0;
};

}  // namespace XrDap
