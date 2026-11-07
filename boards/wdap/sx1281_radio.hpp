// SPDX-License-Identifier: Apache-2.0
#pragma once

// The transceiver of the wdap board: the SX1281 module (xrobot-org/SX1281) on SPI1. Every
// SPI transfer to it is short and polled: the SPI's DMA threshold lies above the largest
// transfer.
//
// wdap 板的收发器：SPI1 上的 SX1281 模块（xrobot-org/SX1281）。到 SX1281 的每次 SPI
// 传输都很短，用轮询：SPI 的 DMA 门限高于最长的传输。

#include <cstdint>
#include <cstring>

#include XRDAP_BOARD_HEADER
#include "SX1281.hpp"
#include "ch32_gpio.hpp"
#include "ch32_spi.hpp"
#include "xrdap/radio/transceiver.hpp"

namespace XrDap
{

/// SX1281 模块作为 RadioLink 要的收发器（xrdap/radio/transceiver.hpp）/ The SX1281 module
/// as the transceiver RadioLink asks for (xrdap/radio/transceiver.hpp).
class RadioDriver : public SX1281
{
 public:
  using SX1281::SX1281;

  LibXR::ErrorCode Start(const Radio::LinkParams& params)
  {
    // FLRC 的同步字固定：勘误 16.4 对编码率 1 没有公布要避开的值，不能用算出来的地址。
    // The FLRC sync word is fixed: errata 16.4 publishes no values to avoid at coding
    // rate 1, so a computed address cannot be used.
    static constexpr uint8_t kFlrcSyncWord[4] = {'X', 'R', 'D', 'P'};
    const bool high = params.rate == Radio::Rate::kHigh;
    Config config;
    config.modulation = high ? Modulation::GFSK : Modulation::FLRC;
    config.frequency_hz = params.frequency_hz;
    // 模块的 PA 跟在芯片后面，芯片输出 0 dBm。 / The module's PA follows the chip, which
    // sends at 0 dBm.
    config.tx_power_dbm = 0;
    config.crc_seed = params.crc_seed;
    std::memcpy(config.sync_word, high ? params.address : kFlrcSyncWord,
                sizeof(config.sync_word));
    return SX1281::Start(config);
  }
};

/// 收发器和它在 SPI1 上的接线 / The transceiver with its wiring on SPI1.
class Sx1281Radio
{
 public:
  Sx1281Radio();

  RadioDriver& Driver() { return radio_; }

 private:
  // 最长的 SPI 传输：命令 + 地址 + 空字节 + 255 字节载荷。 / The longest SPI transfer:
  // command + address + dummy + 255-byte payload.
  static constexpr uint16_t kSpiBuffer = 4 + SX1281::MAX_PAYLOAD_SIZE;

  uint8_t spi_rx_[kSpiBuffer] = {};
  uint8_t spi_tx_[kSpiBuffer] = {};
  LibXR::CH32SPI spi_;
  LibXR::CH32GPIO nss_;
  LibXR::CH32GPIO busy_;
  LibXR::CH32GPIO nreset_;
  LibXR::CH32GPIO dio1_;
  LibXR::CH32GPIO txen_;
  LibXR::CH32GPIO rxen_;
  RadioDriver radio_;
};

}  // namespace XrDap
