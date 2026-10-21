// SPDX-License-Identifier: Apache-2.0
#include "radio_command.hpp"

#include <cstring>

#include "boot_guard.hpp"
#include "config.hpp"
#include "dap_session.hpp"
#include "pairing.hpp"

namespace XrDap
{
namespace
{
constexpr uint8_t kCommand = kProbeCommand;
constexpr uint8_t kOpRole = 0x10, kOpSettings = 0x13, kOpSetRole = 0x14;
constexpr uint8_t kOpSetModulation = 0x15, kOpUnpair = 0x16, kOpPair = 0x20,
                  kOpForward = 0x30;
constexpr uint8_t kOk = 0, kTimeout = 1, kError = 2, kBadRequest = 3, kNotPaired = 4;

uint16_t Reply(uint8_t* resp, uint8_t status)
{
  resp[0] = kCommand;
  resp[1] = status;
  return 2;
}

uint16_t Store(uint8_t* resp, const Config& config)
{
  if (!SaveConfig(config)) return Reply(resp, kError);
  DapSessionAfterAnswer(DapSessionApplyConfig);
  return Reply(resp, kOk);
}

uint16_t Handle(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap)
{
  switch (req[1])
  {
    case kOpRole:
      if (req_len < 3U || req[2] > static_cast<uint8_t>(RadioRole::REMOTE))
        return Reply(resp, kBadRequest);
      return Reply(resp, SetRadioRole(static_cast<RadioRole>(req[2])) ? kOk : kNotPaired);
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
    case kOpForward:
    {
      if (req_len < 4U || req[2] != kCommand) return Reply(resp, kBadRequest);
      if (!DapSessionRadioInUse()) return Reply(resp, kNotPaired);
      uint16_t size = 0;
      if (!DapSessionForward(req + 2, static_cast<uint16_t>(req_len - 2U), resp, resp_cap,
                             size))
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

bool RadioCommandHook(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap, uint16_t& out_len)
{
  if (req_len >= 2U && req[0] == kCommand)
  {
    out_len = RadioVendorCommand(req, req_len, resp, resp_cap);
    return true;
  }
  return DapSessionHandleUsbCommand(req, req_len, resp, resp_cap, out_len);
}
}  // namespace XrDap
