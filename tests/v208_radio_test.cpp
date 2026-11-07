// SPDX-License-Identifier: Apache-2.0
// V208Radio against the transceiver contract, on the build machine. The registers of the
// radio are plain memory mapped at their real addresses; a second thread lets the
// calibration's countdown timer run out. Everything that needs the air (timing, the
// calibration results, reception) is left to the boards.
//
// 在编译机上按收发器契约测试 V208Radio。射频的寄存器是映射在真实地址上的普通内存，另一个
// 线程让校准用的倒计时结束。要空中才能验证的部分（时序、校准结果、接收）留给板子。

#include <sys/mman.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>

// clang-format off
#include "v208_chip.h"
#include "v208_radio.hpp"
// clang-format on

namespace
{

using XrDap::V208Radio;
using XrDap::Radio::LinkParams;
using XrDap::Radio::Rate;

int failures = 0;
int checks = 0;

#define CHECK(cond)                                               \
  do                                                              \
  {                                                               \
    ++checks;                                                     \
    if (!(cond))                                                  \
    {                                                             \
      ++failures;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (false)

constexpr uintptr_t kRegisterPage = 0x40024000U;
constexpr size_t kRegisterBytes = 0x2000U;
// DMA 地址只有 32 位，驱动对象也放在低地址。 / DMA addresses have 32 bits, so the driver
// object lives at a low address as well.
constexpr uintptr_t kObjectPage = 0x50000000U;

volatile uint32_t& Bb(size_t offset)
{
  return *reinterpret_cast<volatile uint32_t*>(kRegisterPage + 0x100U + offset);
}
volatile uint32_t& Ll(size_t offset)
{
  return *reinterpret_cast<volatile uint32_t*>(kRegisterPage + 0x200U + offset);
}
volatile uint32_t& Rf(size_t offset)
{
  return *reinterpret_cast<volatile uint32_t*>(kRegisterPage + 0x1000U + offset);
}

constexpr size_t kBbCtrlCfg = 0x00;
constexpr size_t kBbCrcInit = 0x04;
constexpr size_t kBbAccessAddress = 0x08;
constexpr size_t kBbCtrlTx = 0x2C;
constexpr size_t kBbStatus = 0x38;
constexpr size_t kLlStatus = 0x08;
constexpr size_t kLlTimer = 0x64;
constexpr size_t kLlRxBuffer = 0x74;
constexpr uint32_t kPhyMask = 0xF000U;
constexpr uint32_t kPhy2M = 0x0000U;
constexpr uint32_t kPhy1M = 0x1000U;
constexpr uint32_t kRxDone = 0x01U;
constexpr uint32_t kCrcError = 0x20U;

void* Map(uintptr_t address, size_t bytes)
{
  void* page = mmap(reinterpret_cast<void*>(address), bytes, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (page != reinterpret_cast<void*>(address))
  {
    std::printf("no memory at 0x%lx\n", static_cast<unsigned long>(address));
    std::exit(2);
  }
  return page;
}

LinkParams Params(Rate rate)
{
  LinkParams params;
  params.rate = rate;
  params.frequency_hz = 2448500000U;
  params.crc_seed = 0x1234U;
  params.address[0] = 0x11U;
  params.address[1] = 0x22U;
  params.address[2] = 0x33U;
  params.address[3] = 0x44U;
  return params;
}

// 一个写 1 清除的状态寄存器在普通内存上读回写入的值，这里清掉。 / A write-1-to-clear
// status register on plain memory reads back what was written; clear it here.
void ClearStatus()
{
  Ll(kLlStatus) = 0U;
  Bb(kBbStatus) = 0U;
}

// 让驱动看到一个收到的包。 / Make the driver see a received packet.
int Deliver(V208Radio& radio, uint8_t size, bool crc_ok, uint8_t* out)
{
  auto* rx = reinterpret_cast<uint8_t*>(static_cast<uintptr_t>(Ll(kLlRxBuffer)));
  rx[1] = size;
  for (unsigned i = 0; i < size; ++i)
  {
    rx[2U + i] = static_cast<uint8_t>(i);
  }
  Bb(kBbStatus) = crc_ok ? 0U : kCrcError;
  Ll(kLlStatus) = kRxDone;
  const int result = radio.Receive(out);
  ClearStatus();
  return result;
}

void TestRates(V208Radio& radio)
{
  CHECK(radio.Start(Params(Rate::kHigh)) == LibXR::ErrorCode::OK);
  ClearStatus();
  CHECK((Bb(kBbCtrlCfg) & kPhyMask) == kPhy2M);
  CHECK(Bb(kBbAccessAddress) == 0x11223344U);
  CHECK(Bb(kBbCrcInit) == 0x123434U);
  CHECK(radio.MaxPayload() == 251U);

  CHECK(radio.Start(Params(Rate::kLow)) == LibXR::ErrorCode::OK);
  ClearStatus();
  CHECK((Bb(kBbCtrlCfg) & kPhyMask) == kPhy1M);
  CHECK(Bb(kBbAccessAddress) == 0x11223344U);
  CHECK(Bb(kBbCrcInit) == 0x123434U);
  CHECK(radio.MaxPayload() == 127U);
  // 第二次起算重启。 / The second start onwards counts as a restart.
  CHECK(radio.Restarts() == 1U);
}

void TestStartWrites(V208Radio& radio)
{
  radio.Start(Params(Rate::kHigh));
  ClearStatus();
  // 0 dBm。 / 0 dBm.
  CHECK(((Bb(kBbCtrlTx) >> 25) & 0x3FU) == 0x09U);
  // 不写这个字段时，2432 MHz 上发出的包收不到。 / Without this field, packets sent on
  // 2432 MHz are not received.
  CHECK((Rf(0x3C) >> 29) == 2U);
  CHECK((Fake::ahb_clocks & RCC_AHBPeriph_BLE_CRC) != 0U);
}

void TestReceiveLimit(V208Radio& radio)
{
  static uint8_t out[300];
  radio.Start(Params(Rate::kHigh));
  ClearStatus();
  CHECK(Deliver(radio, 251U, true, out) == 251);
  CHECK(out[250] == 250U);
  CHECK(Deliver(radio, 252U, true, out) < 0);

  radio.Start(Params(Rate::kLow));
  ClearStatus();
  std::memset(out, 0xA5, sizeof(out));
  CHECK(Deliver(radio, 127U, true, out) == 127);
  CHECK(out[126] == 126U && out[127] == 0xA5U);
  // out 只保证 MaxPayload() 字节，更长的是坏包。 / out holds MaxPayload() bytes only, so
  // a longer one is a bad packet.
  const uint32_t errors = radio.RxErrors();
  CHECK(Deliver(radio, 128U, true, out) < 0);
  CHECK(out[127] == 0xA5U);
  CHECK(radio.RxErrors() == errors + 1U);
  CHECK(Deliver(radio, 10U, false, out) < 0);
  CHECK(Deliver(radio, 10U, true, out) == 10);
}

void TestSend(V208Radio& radio)
{
  static const uint8_t data[251] = {};
  radio.Start(Params(Rate::kLow));
  ClearStatus();
  CHECK(radio.Send(data, 128U) == LibXR::ErrorCode::OUT_OF_RANGE);
  CHECK(!radio.Sending());
  CHECK(radio.Send(data, 127U) == LibXR::ErrorCode::OK);
  CHECK(radio.Sending());
  ClearStatus();
  CHECK(radio.Send(data, 1U) == LibXR::ErrorCode::BUSY);

  radio.Start(Params(Rate::kHigh));
  ClearStatus();
  CHECK(radio.Send(data, 251U) == LibXR::ErrorCode::OK);
}

}  // namespace

int main()
{
  Map(kRegisterPage, kRegisterBytes);
  void* object = Map(kObjectPage, 0x10000U);

  std::atomic<bool> done{false};
  std::thread timer(
      [&done]
      {
        while (!done.load(std::memory_order_relaxed))
        {
          Ll(kLlTimer) = 0U;
        }
      });

  auto* radio = new (object) V208Radio();
  TestRates(*radio);
  TestStartWrites(*radio);
  TestReceiveLimit(*radio);
  TestSend(*radio);

  done = true;
  timer.join();
  std::printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
