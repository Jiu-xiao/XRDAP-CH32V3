// SPDX-License-Identifier: Apache-2.0
#include "dap_forwarding.hpp"

#include <cstring>

namespace XrDap
{
namespace
{
// One SWD transaction: 8 request + 1 turnaround + 3 ACK + 1 turnaround + 33 data bits.
constexpr uint32_t kBitsPerTransaction = 46;
// A batch or a target reset may hold the probe for this long beyond its transactions.
constexpr uint32_t kLongCommandUs = 500'000;
// The longest SWJ_Pins wait counted.
constexpr uint32_t kMaxPinWaitUs = 3'000'000;
}  // namespace

uint32_t ExecutionUs(const uint8_t* req, uint16_t len, uint32_t swd_hz)
{
  if (len == 0U || swd_hz == 0U) return 0U;
  uint32_t transactions = 0;
  if (IsCommand(req, len, DapCommand::TRANSFER) && len >= 3U) transactions = req[2];
  if (IsCommand(req, len, DapCommand::TRANSFER_BLOCK) && len >= 4U)
    transactions = static_cast<uint32_t>(req[2] | req[3] << 8);
  uint32_t us = static_cast<uint32_t>(2ULL * kBitsPerTransaction * transactions *
                                      1'000'000ULL / swd_hz);
  if (IsCommand(req, len, DapCommand::DELAY) && len >= 3U)
    us += static_cast<uint32_t>(req[1] | req[2] << 8);
  if (IsCommand(req, len, DapCommand::SWJ_PINS) && len >= 7U)
  {
    uint32_t wait = 0;
    std::memcpy(&wait, req + 3, sizeof(wait));
    us += wait < kMaxPinWaitUs ? wait : kMaxPinWaitUs;
  }
  if (IsCommand(req, len, DapCommand::RESET_TARGET) ||
      IsCommand(req, len, DapCommand::QUEUE_COMMANDS) ||
      IsCommand(req, len, DapCommand::EXECUTE_COMMANDS))
    us += kLongCommandUs;
  return us;
}

uint32_t SwjClockHz(const uint8_t* req, uint16_t len)
{
  if (!IsCommand(req, len, DapCommand::SWJ_CLOCK) || len < 5U) return 0U;
  uint32_t hz = 0;
  std::memcpy(&hz, req + 1, sizeof(hz));
  return hz;
}

void SettingsReplay::Note(const uint8_t* req, uint16_t len)
{
  if (len == 0U) return;
  for (Item& item : items_)
  {
    if (req[0] == item.id && len <= sizeof(item.data))
    {
      std::memcpy(item.data, req, len);
      item.size = static_cast<uint8_t>(len);
    }
  }
}
}  // namespace XrDap
