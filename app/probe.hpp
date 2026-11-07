// SPDX-License-Identifier: Apache-2.0
#pragma once

// What every CH32V3 probe has: the watchdog, the SWD pins, the CMSIS-DAP v2 and DFU
// runtime interfaces on USBHS, XRDAP-Core's vendor extension and executor, the sampling
// tick and the control port. A board file (boards/<name>.cpp) constructs the Probe, the
// parts the board has on top (part.hpp), and runs the main loop with them.
//
// The main loop is the execution context: every round it feeds the watchdog, polls the
// parts, runs the executor (USB commands, control requests, sampling) and carries out a
// DFU DETACH. It waits up to kStepWaitMs for a USB command or the tick interrupt only
// when no part has work; DFU DETACH therefore restarts into the bootloader soon after
// its advertised timeout.
//
// 每块 CH32V3 探针都有的东西：看门狗、SWD 引脚、USBHS 上的 CMSIS-DAP v2 和
// DFU runtime 接口、XRDAP-Core 的厂商扩展和执行器、采样节拍和控制口。
// 板子文件（boards/<name>.cpp）构造 Probe 和这块板另有的部件（part.hpp），
// 用它们运行主循环。
//
// 主循环就是执行上下文：每轮喂狗、轮询部件、运行执行器（USB 命令、控制请求、采样），
// 并执行 DFU DETACH。只有没有部件有事时，它才最多等 kStepWaitMs 等 USB 命令或节拍中断；
// 所以 DFU DETACH 在它声明的超时之后不久就重启进入引导程序。

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include XRDAP_BOARD_HEADER
#include "boot_guard.hpp"
#include "ch32_gpio.hpp"
#include "ch32_timebase.hpp"
#include "ch32_usb_dev.hpp"
#include "daplink_v2_profile_swd.hpp"
#include "debug/swd_general_gpio.hpp"
#include "dfu/dfu_runtime.hpp"
#include "fast_gpio.hpp"
#include "part.hpp"
#include "sampler_tick.hpp"
#include "xrdap/activity/dap_activity.hpp"
#include "xrdap/control/usb_control.hpp"
#include "xrdap/session/executor.hpp"
#include "xrdap/vendor/dap_vendor.hpp"
#include "xrdap/vendor/reset_counting_usb.hpp"

namespace XrDap
{

class Probe
{
 public:
  static constexpr uint16_t kDapPacketSize = 1024;
  static constexpr uint8_t kDapPacketCount = XRDAP_DAP_PACKET_COUNT;  // 每板 / per board

  using Swclk = FastGpioFor<Board::kSwclk>;
  using Swdio = FastGpioFor<Board::kSwdio>;
  using Swd = LibXR::Debug::SwdGeneralGPIO<Swclk, Swdio>;
  using Dap =
      LibXR::USB::DapLinkV2Class<Swd, kDapPacketSize, kDapPacketCount, kDapPacketSize>;
  using Exec = Executor<Dap>;

  /// @brief 启动看门狗、板子、时间基和 USB / Start the watchdog, the board, the timebase
  ///        and USB.
  Probe();

  Probe(const Probe&) = delete;
  Probe& operator=(const Probe&) = delete;

  /// @brief 运行主循环，不返回 / Run the main loop; does not return.
  /// @param parts 这块板另有的部件 / The board's other parts.
  [[noreturn]] void Run(std::initializer_list<Part*> parts);

  // 执行器的这几个调用给无线链路用（Radio::RadioLink 的 ExecT）。定义在
  // probe.cpp：DAP命令的处理只编译一份。
  // These executor calls serve the radio link (ExecT of Radio::RadioLink). They are
  // defined in probe.cpp, so the DAP command processing is compiled once.
  void SetForwarder(Exec::Forwarder forwarder, void* ctx);
  void ExecuteHere(Transport t, const uint8_t* req, uint16_t len, uint8_t* resp,
                   uint16_t cap, uint16_t& out);
  void ControlHere(const uint8_t* req, uint16_t len, uint8_t* resp, uint16_t cap,
                   uint16_t& out);
  void LinkLost(Transport t);
  const DapActivity& Activity() const { return activity_; }
  /// USB 已配置（电脑在用）/ USB is configured (the PC uses the probe).
  bool UsbConfigured() const { return dap_.IsInited(); }
  bool SafeMode() const { return guard_.SafeMode(); }

 private:
  // DFU runtime 接口，同时数 USB 总线复位（libxr 把它报给每个类）：从 USB 开始的采样在总
  // 线复位时停止。
  // The DFU runtime interface, also counting USB bus resets (libxr reports them to every
  // class): sampling started from USB stops when the bus resets.
  class DfuRuntime : public ResetCountingUsb<LibXR::USB::DfuRuntimeClass>
  {
   public:
    DfuRuntime();
  };

  // 构造顺序里的两步：板子初始化（引脚、电源开关），时间基之后的 LibXR 平台初始化。
  // Two steps in the construction order: the board's own set-up (pins, power switches),
  // and LibXR's platform set-up after the timebase.
  struct BoardInit
  {
    BoardInit();
  };
  struct PlatformInit
  {
    PlatformInit();
  };

  static uint16_t HandleControl(void* ctx, const uint8_t* req, uint16_t len,
                                uint8_t* resp, uint16_t cap);

  BootGuard guard_;
  BoardInit board_;
  LibXR::CH32Timebase timebase_;
  PlatformInit platform_;
  Swclk swclk_;
  Swdio swdio_;
  Swd swd_;
  LibXR::CH32GPIO nreset_;
  Dap dap_;
  DfuRuntime dfu_;
  DapVendor vendor_;
  DapActivity activity_;
  Exec exec_;
  SamplerTick tick_;
  Control::UsbControl<Dap> usb_control_;
  LibXR::CH32USBOtgHS usb_;
  Part* const* parts_ = nullptr;
  size_t part_count_ = 0;
};

}  // namespace XrDap
