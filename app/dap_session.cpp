// SPDX-License-Identifier: Apache-2.0
#include "dap_session.hpp"

#include "config.hpp"
#include "dap_forwarding.hpp"
#include "link_session.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
// A ping is sent again every 3 ms (link_transport) until this budget has passed.
constexpr uint32_t kPingBudgetUs = 30'000;
// A single try, for the heartbeat and the session end.
constexpr uint32_t kPingOnceUs = 0;
// A wired session that receives no command for this long ends, and the probe serves
// the radio again (a tool that quit without DAP_Disconnect).
constexpr uint32_t kLocalSessionIdleUs = 5'000'000;
// After this long without hearing the peer, the next command pings first.
constexpr uint32_t kQuietUs = 1'000'000;

RoleSetting setting = RoleSetting::AUTO;
bool paired = false;

// Sessions in the AUTO setting.
bool session_wireless = false;
bool session_local = false;
uint32_t last_usb_us = 0;

// Host side of the radio session.
constinit Link::HostSession host;  // constant initialised: no startup constructors
// The last forwarded command got no answer, or the peer has been quiet for kQuietUs:
// the next command pings first and fails fast when the peer does not answer.
bool peer_lost = false;
uint32_t heard_peer_us = 0;
uint32_t swd_hz = kDefaultSwdHz;  // last SWJ_Clock forwarded
constinit SettingsReplay replay;

uint32_t NowUs() { return static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds()); }

void Fail(const uint8_t* req, uint8_t* resp, uint16_t& out_len)
{
  resp[0] = req[0];
  resp[1] = Byte(DapStatus::ERROR);
  out_len = 2;
}

void ResetSessions()
{
  session_wireless = session_local = peer_lost = false;
  if (host.OnDisconnect()) (void)LinkPing(0, kPingOnceUs);
}

/// Open the host session if needed (H1) and make sure the peer is there.
bool ReachPeer()
{
  const uint32_t now = NowUs();
  const bool fresh = host.OnCommand(now);
  if (fresh || peer_lost || now - heard_peer_us >= kQuietUs)
  {
    if (!LinkPing(host.session(), kPingBudgetUs)) return false;
    peer_lost = false;
    heard_peer_us = NowUs();
    host.OnSent(heard_peer_us);
  }
  return true;
}

bool Forward(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t resp_cap,
             uint16_t& out_len)
{
  const uint32_t hz = SwjClockHz(req, req_len);
  if (hz != 0U) swd_hz = hz;
  if (!ReachPeer())
  {
    peer_lost = true;
    Fail(req, resp, out_len);
    return false;
  }
  const bool disconnect = IsCommand(req, req_len, Byte(DapCmd::DISCONNECT));
  if (!LinkForward(req, req_len, resp, resp_cap, out_len,
                   ExecutionUs(req, req_len, swd_hz), !disconnect))
  {
    peer_lost = true;
    Fail(req, resp, out_len);
    return false;
  }
  heard_peer_us = NowUs();
  host.OnSent(heard_peer_us);
  if (disconnect && host.OnDisconnect()) (void)LinkPing(0, kPingOnceUs);  // H3
  return true;
}
}  // namespace

void DapSessionInit(LinkExecutor execute, LinkSessionEnd on_remote_session_end)
{
  LinkInit(execute, on_remote_session_end);
}

void DapSessionApplyConfig()
{
  const Config& config = GetConfig();
  ResetSessions();
  setting = config.role;
  paired = config.paired != 0U;
  // B3: an unpaired probe is a wired probe and does not listen to the radio.
  if (paired && setting != RoleSetting::LOCAL)
    LinkStart(config.modulation == Modulation::GFSK, PairIdentity(config.link_id));
  else
    LinkStop();
}

RadioRole GetRadioRole()
{
  if (!LinkRunning()) return RadioRole::LOCAL;
  switch (setting)
  {
    case RoleSetting::LOCAL:
      return RadioRole::LOCAL;
    case RoleSetting::HOST:
      return RadioRole::HOST;
    case RoleSetting::REMOTE:
      return RadioRole::REMOTE;
    case RoleSetting::AUTO:
    default:
      return session_wireless ? RadioRole::HOST
             : session_local  ? RadioRole::LOCAL
                              : RadioRole::REMOTE;
  }
}

bool SetRadioRole(RadioRole role)
{
  if (!paired && role != RadioRole::LOCAL) return false;
  const Config& config = GetConfig();
  ResetSessions();
  setting = role == RadioRole::HOST     ? RoleSetting::HOST
            : role == RadioRole::REMOTE ? RoleSetting::REMOTE
                                        : RoleSetting::LOCAL;
  if (setting != RoleSetting::LOCAL)
    LinkStart(config.modulation == Modulation::GFSK, PairIdentity(config.link_id));
  else
    LinkStop();
  return true;
}

bool DapSessionRadioInUse() { return LinkRunning(); }

bool RadioSessionWireless() { return GetRadioRole() == RadioRole::HOST; }

bool RemoteSessionActive() { return LinkRemoteSessionActive(); }

bool DapSessionHandleUsbCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                                uint16_t resp_cap, uint16_t& out_len)
{
  // A command from the peer also passes the DAP class's hook: execute it here.
  if (LinkExecuting() || !LinkRunning() || req_len == 0U) return false;
  last_usb_us = NowUs();
  switch (setting)
  {
    case RoleSetting::HOST:
      (void)Forward(req, req_len, resp, resp_cap, out_len);
      return true;
    case RoleSetting::LOCAL:
    case RoleSetting::REMOTE:
      return false;
    case RoleSetting::AUTO:
    default:
      break;
  }
  // DAP_Info is the same on both probes; tools ask it before they connect.
  if (IsCommand(req, req_len, Byte(DapCmd::INFO))) return false;
  if (IsCommand(req, req_len, Byte(DapCmd::CONNECT)))
  {
    // A new session: the previous one (if any) is over for the remote too.
    ResetSessions();
    session_wireless = ReachPeer();
    session_local = !session_wireless;
    if (session_wireless)
    {
      for (uint8_t i = 0; i < SettingsReplay::kSettings; ++i)
      {
        if (replay.Size(i) == 0U) continue;
        uint8_t ignored[16];
        uint16_t ignored_len = 0;
        (void)Forward(replay.Data(i), replay.Size(i), ignored, sizeof(ignored),
                      ignored_len);
      }
    }
    else
    {
      (void)host.OnDisconnect();  // nobody answered: no session over the radio
    }
  }
  replay.Note(req, req_len);
  if (!session_wireless)
  {
    if (IsCommand(req, req_len, Byte(DapCmd::DISCONNECT))) session_local = false;
    return false;
  }
  (void)Forward(req, req_len, resp, resp_cap, out_len);
  if (IsCommand(req, req_len, Byte(DapCmd::DISCONNECT))) session_wireless = false;
  return true;
}

bool DapSessionForward(const uint8_t* req, uint16_t req_len, uint8_t* resp,
                       uint16_t resp_cap, uint16_t& out_len)
{
  if (!LinkRunning()) return false;
  return Forward(req, req_len, resp, resp_cap, out_len);
}

void DapSessionService()
{
  if (!LinkRunning()) return;
  const uint32_t now = NowUs();
  if (setting == RoleSetting::AUTO && session_local &&
      now - last_usb_us >= kLocalSessionIdleUs)
    session_local = false;
  const RadioRole role = GetRadioRole();
  if (role == RadioRole::HOST)
  {
    switch (host.Tick(now))
    {
      case Link::HostSession::Action::HEARTBEAT:  // H2
        if (LinkPing(host.session(), kPingOnceUs))
          heard_peer_us = NowUs();
        else
          peer_lost = true;
        break;
      case Link::HostSession::Action::END:  // H3
        (void)LinkPing(0, kPingOnceUs);
        break;
      case Link::HostSession::Action::NONE:
      default:
        break;
    }
    return;
  }
  if (role == RadioRole::REMOTE) LinkServeRemote();
}

void DapSessionAfterAnswer(void (*action)()) { LinkAfterAnswer(action); }
}  // namespace XrDap
