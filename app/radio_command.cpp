// SPDX-License-Identifier: Apache-2.0
#include "radio_command.hpp"

#include <cstring>
#include <initializer_list>

#include "FreeRTOS.h"
#include "boot_guard.hpp"
#include "config.hpp"
#include "pairing.hpp"
#include "radio_link.hpp"
#include "sx1281.hpp"
#include "task.h"

namespace XrDap
{
namespace
{
constexpr uint8_t kCommand = 0x9F;
constexpr uint8_t kOpSpi = 0x00, kOpPins = 0x01, kOpOutputs = 0x02, kOpWaitDio1 = 0x03;
constexpr uint8_t kOpRole = 0x10, kOpStats = 0x11, kOpResetStats = 0x12;
constexpr uint8_t kOpSettings = 0x13, kOpSetRole = 0x14, kOpSetModulation = 0x15;
constexpr uint8_t kOpUnpair = 0x16, kOpTiming = 0x17, kOpPair = 0x20, kOpForward = 0x30;
constexpr uint8_t kOpPeek = 0x40, kOpHang = 0x41;
constexpr uint8_t kOk = 0, kTimeout = 1, kError = 2, kBadRequest = 3;

uint16_t Reply(uint8_t* resp, uint8_t status)
{
  resp[0] = kCommand;
  resp[1] = status;
  return 2;
}

uint16_t PutWords(uint8_t* resp, uint16_t at, std::initializer_list<uint32_t> words)
{
  for (uint32_t word : words)
  {
    std::memcpy(resp + at, &word, sizeof(word));
    at = static_cast<uint16_t>(at + sizeof(word));
  }
  return at;
}

uint16_t Store(uint8_t* resp, const Config& config)
{
  if (!SaveConfig(config)) return Reply(resp, kError);
  RadioLinkAfterAnswer(RadioLinkApplyConfig);
  return Reply(resp, kOk);
}

uint16_t Handle(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap)
{
  Sx1281& radio = Radio();
  switch (req[1])
  {
    case kOpSpi:
    {
      const uint16_t n = req_len >= 3U ? req[2] : 0U;
      if (n == 0U || req_len < 3U + n || resp_cap < 3U + n) return Reply(resp, kBadRequest);
      if (!radio.Transfer(req + 3, resp + 3, n)) return Reply(resp, kError);
      Reply(resp, kOk);
      resp[2] = static_cast<uint8_t>(n);
      return static_cast<uint16_t>(3U + n);
    }
    case kOpPins:
      resp[2] = radio.Pins();
      return static_cast<uint16_t>(Reply(resp, kOk) + 1U);
    case kOpOutputs:
      if (req_len < 4U) return Reply(resp, kBadRequest);
      radio.SetOutputs(req[2], req[3]);
      return Reply(resp, kOk);
    case kOpWaitDio1:
    {
      if (req_len < 4U) return Reply(resp, kBadRequest);
      uint32_t waited = 0;
      const bool ok = radio.WaitDio1((req[2] | req[3] << 8) * 1000U, waited);
      Reply(resp, ok ? kOk : kTimeout);
      return PutWords(resp, 2, {waited});
    }
    case kOpRole:
      if (req_len < 3U || req[2] > static_cast<uint8_t>(RadioRole::REMOTE))
        return Reply(resp, kBadRequest);
      SetRadioRole(static_cast<RadioRole>(req[2]));
      return Reply(resp, kOk);
    case kOpStats:
    {
      const RadioLinkStats& link = GetRadioLinkStats();
      Reply(resp, kOk);
      resp[2] = static_cast<uint8_t>(GetRadioRole());
      return PutWords(
          resp, 3,
          {radio.tx_packets, radio.rx_packets, radio.crc_errors, radio.tx_timeouts,
           link.commands, link.retries, link.failures, link.duplicates, link.last_rtt_us,
           link.max_rtt_us, static_cast<uint32_t>(xPortGetFreeHeapSize()),
           static_cast<uint32_t>(xPortGetMinimumEverFreeHeapSize()),
           static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)),
           radio.rearms, radio.restarts, link.pings, link.pongs});
    }
    case kOpResetStats:
      ResetRadioLinkStats();
      return Reply(resp, kOk);
    case kOpTiming:
    {
      const RadioLinkStats& link = GetRadioLinkStats();
      Reply(resp, kOk);
      return PutWords(resp, 2,
                      {radio.spi_transfers, radio.spi_us, radio.busy_us, radio.tx_wait_us,
                       link.send_us, link.wait_us, link.rx_us, link.exec_us, link.reply_us});
    }
    case kOpSettings:
    {
      const Config& config = GetConfig();
      Reply(resp, kOk);
      resp[2] = static_cast<uint8_t>(config.role);
      resp[3] = static_cast<uint8_t>(config.modulation);
      resp[4] = config.paired;
      resp[5] = SafeMode() ? 1U : 0U;
      resp[6] = static_cast<uint8_t>(GetRadioRole());
      resp[7] = static_cast<uint8_t>(GetPairState());
      std::memcpy(resp + 8, &config.link_id, sizeof(config.link_id));
      std::memcpy(resp + 12, config.peer_uid, sizeof(config.peer_uid));
      return 24;
    }
    case kOpSetRole:
    {
      if (req_len < 3U || req[2] > static_cast<uint8_t>(RoleSetting::REMOTE))
        return Reply(resp, kBadRequest);
      Config config = GetConfig();
      config.role = static_cast<RoleSetting>(req[2]);
      return Store(resp, config);
    }
    case kOpSetModulation:
    {
      if (req_len < 3U || req[2] > static_cast<uint8_t>(Modulation::FLRC))
        return Reply(resp, kBadRequest);
      Config config = GetConfig();
      config.modulation = static_cast<Modulation>(req[2]);
      return Store(resp, config);
    }
    case kOpUnpair:
    {
      Config config = GetConfig();
      config.paired = 0;
      config.link_id = 0;
      std::memset(config.peer_uid, 0, sizeof(config.peer_uid));
      return Store(resp, config);
    }
    case kOpPair:
      PairingRequest();
      return Reply(resp, kOk);
    case kOpPeek:
    {
      uint32_t address = 0;
      if (req_len < 7U || req[6] == 0U || resp_cap < 2U + req[6]) return Reply(resp, kBadRequest);
      std::memcpy(&address, req + 2, sizeof(address));
      std::memcpy(resp + 2, reinterpret_cast<const void*>(address), req[6]);
      Reply(resp, kOk);
      return static_cast<uint16_t>(2U + req[6]);
    }
    case kOpHang:
      // Test of the watchdog and safe mode (boot_guard.hpp): stop feeding the watchdog.
      for (;;)
      {
      }
    case kOpForward:
    {
      if (req_len < 4U || req[2] != kCommand) return Reply(resp, kBadRequest);
      uint16_t size = 0;
      if (!RadioLinkForward(req + 2, static_cast<uint16_t>(req_len - 2U), resp, resp_cap, size))
        return Reply(resp, kTimeout);
      return size;
    }
    default:
      return Reply(resp, kBadRequest);
  }
}
}  // namespace

uint16_t RadioVendorCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                            uint16_t resp_cap)
{
  return req_len >= 2U && resp_cap >= 80U ? Handle(req, req_len, resp, resp_cap)
                                          : Reply(resp, kBadRequest);
}

bool RadioCommandHook(void*, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap, uint16_t& out_len)
{
  if (req_len >= 2U && req[0] == kCommand)
  {
    out_len = RadioVendorCommand(req, req_len, resp, resp_cap);
    return true;
  }
  return RadioLinkHandleUsbCommand(req, req_len, resp, resp_cap, out_len);
}
}  // namespace XrDap
