// SPDX-License-Identifier: Apache-2.0
#include "radio.hpp"

#include "chip_uid.hpp"
#include "timebase.hpp"

namespace XrDap
{

namespace
{

constexpr uint32_t kPairHoldMs = 3'000;

// 板级射频配置：模块的 PA 跟在芯片后面，芯片输出 0 dBm。 / Board configuration of the
// radio: the module's PA follows the chip, which sends at 0 dBm.
SX1281::Config BaseConfig()
{
  SX1281::Config config;
  config.tx_power_dbm = 0;
  return config;
}

}  // namespace

RadioPart::RadioPart(Probe& probe)
    : spi_(CH32_SPI1, {spi_rx_, sizeof(spi_rx_)}, {spi_tx_, sizeof(spi_tx_)},
           Board::kRadioSck.Port(), Board::kRadioSck.Mask(), Board::kRadioMiso.Port(),
           Board::kRadioMiso.Mask(), Board::kRadioMosi.Port(), Board::kRadioMosi.Mask(),
           0, true, true, SPI_BaudRatePrescaler_16, kSpiBuffer),
      nss_(Board::kRadioNss.Port(), Board::kRadioNss.Mask()),
      busy_(Board::kRadioBusy.Port(), Board::kRadioBusy.Mask(),
            LibXR::GPIO::Direction::INPUT),
      nreset_(Board::kRadioNreset.Port(), Board::kRadioNreset.Mask()),
      dio1_(Board::kRadioDio1.Port(), Board::kRadioDio1.Mask(),
            LibXR::GPIO::Direction::INPUT),
      txen_(Board::kRadioTxEn.Port(), Board::kRadioTxEn.Mask()),
      rxen_(Board::kRadioRxEn.Port(), Board::kRadioRxEn.Mask()),
      radio_(spi_, nss_, busy_, nreset_, dio1_, txen_, rxen_),
      store_(probe.SafeMode()),
      link_(radio_, probe, store_, ChipUid(), BaseConfig()),
      key_(Board::kBootKey.Port(), Board::kBootKey.Mask(), LibXR::GPIO::Direction::INPUT,
           LibXR::GPIO::Pull::UP)
{
  link_.Begin(store_.Current(), probe.SafeMode());
}

bool RadioPart::Poll()
{
  PollKey();
  return link_.Service();
}

bool RadioPart::Control(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                        uint16_t& out)
{
  return link_.HandleControl(req, len, resp, cap, out);
}

bool RadioPart::Wireless() const
{
  return link_.CurrentRole() == Radio::Role::HOST || link_.RemoteActive();
}

void RadioPart::PollKey()
{
  // 按键低电平有效；每按一次只触发一次。 / The key is active low; one request per press.
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

}  // namespace XrDap
