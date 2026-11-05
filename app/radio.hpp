// SPDX-License-Identifier: Apache-2.0
#pragma once

// The radio of the wdap board: the SX1281 module (xrobot-org/SX1281) on SPI1 and
// XRDAP-Core's RadioLink, which runs DAP sessions with the paired probe
// (xrdap/radio/radio_link.hpp), with the settings in flash (settings_store.hpp). Holding
// the boot key for 3 s starts pairing. Every SPI transfer to the SX1281 is short and
// polled: the SPI's DMA threshold lies above the largest transfer.
//
// wdap 板的无线：SPI1 上的 SX1281 模块（xrobot-org/SX1281）和 XRDAP-Core 的
// RadioLink，它与配对的探针运行 DAP 会话（xrdap/radio/radio_link.hpp），
// 设置在 Flash 里（settings_store.hpp）。按住 boot 键 3 s 开始配对。
// 到 SX1281 的每次 SPI 传输都很短，用轮询：SPI 的 DMA 门限高于最长的传输。

#include <cstdint>

#include "SX1281.hpp"
#include "ch32_gpio.hpp"
#include "ch32_spi.hpp"
#include "part.hpp"
#include "probe.hpp"
#include "settings_store.hpp"
#include "xrdap/radio/radio_link.hpp"

namespace XrDap
{

class RadioPart final : public Part
{
 public:
  using Link = Radio::RadioLink<SX1281, Probe>;

  explicit RadioPart(Probe& probe);

  bool Poll() override;
  bool Control(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
               uint16_t& out) override;

  /// 一个调试会话正在经无线运行（本机转发，或对方在用本机）/ A debug session runs over
  /// the radio (this probe forwards, or the peer drives it).
  bool Wireless() const;
  /// 正在配对 / Pairing is running.
  bool Pairing() const { return link_.Pairing() == Radio::PairState::RUNNING; }

 private:
  // 最长的 SPI 传输：命令 + 地址 + 空字节 + 255 字节载荷。 / The longest SPI transfer:
  // command + address + dummy + 255-byte payload.
  static constexpr uint16_t kSpiBuffer = 4 + SX1281::MAX_PAYLOAD_SIZE;

  void PollKey();

  uint8_t spi_rx_[kSpiBuffer] = {};
  uint8_t spi_tx_[kSpiBuffer] = {};
  LibXR::CH32SPI spi_;
  LibXR::CH32GPIO nss_;
  LibXR::CH32GPIO busy_;
  LibXR::CH32GPIO nreset_;
  LibXR::CH32GPIO dio1_;
  LibXR::CH32GPIO txen_;
  LibXR::CH32GPIO rxen_;
  SX1281 radio_;
  SettingsStore store_;
  Link link_;

  LibXR::CH32GPIO key_;
  bool key_down_ = false;
  bool key_fired_ = false;
  uint32_t key_since_ms_ = 0;
};

}  // namespace XrDap
