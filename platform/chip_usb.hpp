// SPDX-License-Identifier: Apache-2.0
#pragma once

// USB of the CH32V30x boards: the high-speed controller with its own PHY. The DAP
// interface uses both directions of EP1, each with slot memory for two whole DAP packets,
// as LibXR's USBHS endpoints have separate DMA registers for the two directions.
//
// CH32V30x 板子的 USB：高速控制器和它自带的 PHY。DAP 接口用 EP1 的两个方向，每个方向的
// 槽内存放得下两个完整的 DAP 包，因为 LibXR 的 USBHS 端点两个方向的 DMA 寄存器是分开的。

#include <cstdint>
#include <initializer_list>

#include "ch32_usb_dev.hpp"
#include "xrdap_chip.h"

namespace XrDap
{

inline constexpr auto kDapInEndpoint = LibXR::USB::Endpoint::EPNumber::EP1;
inline constexpr auto kDapOutEndpoint = LibXR::USB::Endpoint::EPNumber::EP1;

using UsbStrings =
    std::initializer_list<const LibXR::USB::DescriptorStrings::LanguagePack*>;
using UsbConfigs =
    std::initializer_list<const std::initializer_list<LibXR::USB::ConfigDescriptorItem*>>;

namespace ChipUsbDetail
{
template <uint16_t kDapPacketSize>
struct ProbeMemory
{
  alignas(64) uint8_t ep0[64];
  alignas(64) uint8_t ep1_in[2 * kDapPacketSize];
  alignas(64) uint8_t ep1_out[2 * kDapPacketSize];
};

struct BootMemory
{
  alignas(64) uint8_t ep0[64];
};
}  // namespace ChipUsbDetail

/// USB 设备：应用 / The USB device of the application.
template <uint16_t kDapPacketSize>
class ProbeUsb : private ChipUsbDetail::ProbeMemory<kDapPacketSize>,
                 public LibXR::CH32USBOtgHS
{
 public:
  ProbeUsb(uint16_t vid, uint16_t pid, uint16_t bcd, UsbStrings strings,
           UsbConfigs configs, LibXR::ConstRawData uid)
      : LibXR::CH32USBOtgHS({{this->ep0}, {this->ep1_out}}, {{this->ep0}, {this->ep1_in}},
                            vid, pid, bcd, strings, configs, uid)
  {
  }
};

/// USB 设备：引导程序，只有 EP0 / The USB device of the bootloader, EP0 only.
class BootUsb : private ChipUsbDetail::BootMemory, public LibXR::CH32USBOtgHS
{
 public:
  BootUsb(uint16_t vid, uint16_t pid, uint16_t bcd, UsbStrings strings,
          UsbConfigs configs, LibXR::ConstRawData uid)
      : LibXR::CH32USBOtgHS({{ep0}}, {{ep0}}, vid, pid, bcd, strings, configs, uid)
  {
  }
};

/// @brief 断开上拉，主机看到设备离开 / Drop the pull-up, so that the host sees the device
///        leave.
inline void UsbDropPullUp() { USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE; }

/// @brief 跳到应用之前把控制器停下 / Stop the controller before the jump to the
///        application.
inline void UsbStop()
{
  NVIC_DisableIRQ(USBHS_IRQn);
  NVIC_DisableIRQ(USBHSWakeup_IRQn);
  USBHSD->INT_EN = 0U;
  UsbDropPullUp();
  USBHSD->CONTROL = 0U;
}

}  // namespace XrDap
