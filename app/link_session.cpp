// SPDX-License-Identifier: Apache-2.0
#include "link_session.hpp"

namespace XrDap::Link
{
RemoteLease::Event RemoteLease::OnPing(uint16_t session, uint32_t now_us)
{
  heard_us_ = now_us;
  if (session == session_) return Event::NONE;
  const bool had = session_ != 0U;
  session_ = session;
  if (session == 0U) return had ? Event::ENDED : Event::NONE;  // R1
  return Event::REPLACED;                                      // R2
}

bool RemoteLease::OnCommand(uint32_t now_us)
{
  if (session_ == 0U) return false;  // R5
  heard_us_ = now_us;                // R3
  return true;
}

RemoteLease::Event RemoteLease::Tick(uint32_t now_us)
{
  if (session_ == 0U || now_us - heard_us_ < kLeaseUs) return Event::NONE;
  session_ = 0;  // R3
  return Event::ENDED;
}

bool HostSession::OnCommand(uint32_t now_us)
{
  command_us_ = now_us;
  if (session_ != 0U) return false;
  // H1: a new non-zero id, different from the previous one. The time of the first
  // session varies from start to start, so a probe that restarts does not repeat the
  // ids of its previous run.
  if (!seeded_)
  {
    next_ = static_cast<uint16_t>(next_ ^ now_us ^ (now_us >> 16));
    seeded_ = true;
  }
  do
  {
    ++next_;
  } while (next_ == 0U);
  session_ = next_;
  return true;
}

bool HostSession::OnDisconnect()
{
  if (session_ == 0U) return false;
  session_ = 0;  // H3
  return true;
}

HostSession::Action HostSession::Tick(uint32_t now_us)
{
  if (session_ == 0U) return Action::NONE;
  if (now_us - command_us_ >= kHostIdleUs)
  {
    session_ = 0;  // H3
    return Action::END;
  }
  if (now_us - sent_us_ >= kHeartbeatUs)
  {
    sent_us_ = now_us;  // H2
    return Action::HEARTBEAT;
  }
  return Action::NONE;
}
}  // namespace XrDap::Link
