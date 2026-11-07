// SPDX-License-Identifier: Apache-2.0
#pragma once

// USB of the CH32V208 boards: the full-speed controller on PB6/PB7. The DAP interface
// uses two single-direction endpoints, EP1 OUT and EP2 IN: only a single-direction
// endpoint can hold a DAP packet longer than one USB packet in its slots. The other mode
// of the controller, both directions on one endpoint number, has one 64-byte slot per
// direction, and the DAP packet would shrink to that.
//
// CH32V208 板子的 USB：PB6/PB7 上的全速控制器。DAP 接口用两个单向端点，EP1 OUT 和
// EP2 IN：只有单向端点的槽放得下比一个 USB 包长的 DAP 包。控制器的另一种模式是两个方向
// 用同一个端点号，每个方向只有一个 64 字节的槽，DAP 包也只能缩到这么长。

#include <cstdint>
#include <initializer_list>

#include "ch32_usb_dev.hpp"
#include "xrdap_chip.h"

namespace XrDap
{

inline constexpr auto kDapInEndpoint = LibXR::USB::Endpoint::EPNumber::EP2;
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
  alignas(64) uint8_t ep1_out[2 * kDapPacketSize];
  alignas(64) uint8_t ep2_in[2 * kDapPacketSize];
};

struct BootMemory
{
  alignas(64) uint8_t ep0[64];
};
}  // namespace ChipUsbDetail

/// USB 设备：应用 / The USB device of the application.
template <uint16_t kDapPacketSize>
class ProbeUsb : private ChipUsbDetail::ProbeMemory<kDapPacketSize>,
                 public LibXR::CH32USBOtgFS
{
 public:
  ProbeUsb(uint16_t vid, uint16_t pid, uint16_t bcd, UsbStrings strings,
           UsbConfigs configs, LibXR::ConstRawData uid)
      : LibXR::CH32USBOtgFS({{this->ep0}, {this->ep1_out}, {nullptr, 0}},
                            {{this->ep0}, {nullptr, 0}, {this->ep2_in}},
                            LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64, vid, pid,
                            bcd, strings, configs, uid)
  {
  }
};

/// USB 设备：引导程序，只有 EP0 / The USB device of the bootloader, EP0 only.
class BootUsb : private ChipUsbDetail::BootMemory, public LibXR::CH32USBOtgFS
{
 public:
  BootUsb(uint16_t vid, uint16_t pid, uint16_t bcd, UsbStrings strings,
          UsbConfigs configs, LibXR::ConstRawData uid)
      : LibXR::CH32USBOtgFS({{ep0}}, {{ep0}},
                            LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64, vid, pid,
                            bcd, strings, configs, uid)
  {
  }
};

/// @brief 断开上拉，主机看到设备离开 / Drop the pull-up, so that the host sees the device
///        leave.
inline void UsbDropPullUp() { USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL; }

/// @brief 跳到应用之前把控制器停下 / Stop the controller before the jump to the
///        application.
inline void UsbStop()
{
  NVIC_DisableIRQ(USBFS_IRQn);
  NVIC_DisableIRQ(USBFSWakeUp_IRQn);
  USBFSD->INT_EN = 0U;
  UsbDropPullUp();
  USBFSD->BASE_CTRL = 0U;
  // 应用的系统时钟和引导程序不同，要重设 USB 分频，而 USBPRE 只能在 USB 时钟打开之前设，
  // 时钟开着时写入无效。 / The application's system clock differs from the bootloader's,
  // so it sets the USB divider again; USBPRE can only be set before the USB clock is
  // enabled, and a write with the clock on has no effect.
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, DISABLE);
}

}  // namespace XrDap
