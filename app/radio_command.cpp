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
constexpr uint8_t kOpRole = 0x10, kOpSettings = 0x13, kOpSetRole = 0x14;
constexpr uint8_t kOpSetModulation = 0x15, kOpUnpair = 0x16, kOpPair = 0x20,
                  kOpForward = 0x30;
constexpr uint8_t kOk = 0, kTimeout = 1, kError = 2, kBadRequest = 3, kNotPaired = 4;

// The answer starts with the op of the request (`op status payload`).
uint16_t Reply(uint8_t* resp, uint8_t op, uint8_t status)
{
  resp[0] = op;
  resp[1] = status;
  return 2;
}

uint16_t Store(uint8_t* resp, uint8_t op, const Config& config)
{
  if (!SaveConfig(config)) return Reply(resp, op, kError);
  DapSessionAfterAnswer(DapSessionApplyConfig);
  return Reply(resp, op, kOk);
}

uint16_t Handle(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap)
{
  const uint8_t op = req[0];
  switch (op)
  {
    case kOpRole:
      if (req_len < 2U || req[1] > static_cast<uint8_t>(RadioRole::REMOTE))
        return Reply(resp, op, kBadRequest);
      return Reply(resp, op, SetRadioRole(static_cast<RadioRole>(req[1])) ? kOk : kNotPaired);
    case kOpSettings:
    {
      const Config& config = GetConfig();
      Reply(resp, op, kOk);
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
      if (req_len < 2U || req[1] > static_cast<uint8_t>(RoleSetting::REMOTE))
        return Reply(resp, op, kBadRequest);
      Config config = GetConfig();
      config.role = static_cast<RoleSetting>(req[1]);
      return Store(resp, op, config);
    }
    case kOpSetModulation:
    {
      if (req_len < 2U || req[1] > static_cast<uint8_t>(Modulation::FLRC))
        return Reply(resp, op, kBadRequest);
      Config config = GetConfig();
      config.modulation = static_cast<Modulation>(req[1]);
      return Store(resp, op, config);
    }
    case kOpUnpair:
    {
      Config config = GetConfig();
      config.paired = 0;
      config.link_id = 0;
      std::memset(config.peer_uid, 0, sizeof(config.peer_uid));
      return Store(resp, op, config);
    }
    case kOpPair:
      PairingRequest();
      return Reply(resp, op, kOk);
    case kOpForward:
    {
      // The request for the peer is the rest; forwarding a forward is refused.
      if (req_len < 2U || req[1] == kOpForward) return Reply(resp, op, kBadRequest);
      if (!DapSessionRadioInUse()) return Reply(resp, op, kNotPaired);
      uint16_t size = 0;
      if (!DapSessionForward(req + 1, static_cast<uint16_t>(req_len - 1U), resp, resp_cap,
                             size))
        return Reply(resp, op, kTimeout);
      return size;
    }
    default:
      return Reply(resp, op, kBadRequest);
  }
}
}  // namespace

uint16_t RadioControl(void*, const uint8_t* req, uint16_t req_len, uint8_t* resp,
                      uint16_t resp_cap)
{
  if (req == nullptr || req_len < 1U || resp_cap < 80U)
    return resp_cap >= 2U ? Reply(resp, req_len >= 1U ? req[0] : 0U, kBadRequest) : 0U;
  return Handle(req, req_len, resp, resp_cap);
}

}  // namespace XrDap
