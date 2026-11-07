// SPDX-License-Identifier: Apache-2.0
#include "sx1281_radio.hpp"

namespace XrDap
{

Sx1281Radio::Sx1281Radio()
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
      radio_(spi_, nss_, busy_, nreset_, dio1_, txen_, rxen_)
{
}

}  // namespace XrDap
