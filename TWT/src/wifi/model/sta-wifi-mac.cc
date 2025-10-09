/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006, 2009 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 */

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "sta-wifi-mac.h"
#include "wifi-phy.h"
#include "mgt-headers.h"
#include "snr-tag.h"
#include "wifi-net-device.h"
#include "ns3/ht-configuration.h"
#include "ns3/he-configuration.h"
#include "wifi-mac-queue.h"
#include "ns3/wifi-psdu.h"
#include "ns3/frame-exchange-manager.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("StaWifiMac");

NS_OBJECT_ENSURE_REGISTERED (StaWifiMac);

TypeId
StaWifiMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::StaWifiMac")
    .SetParent<RegularWifiMac> ()
    .SetGroupName ("Wifi")
    .AddConstructor<StaWifiMac> ()
    .AddAttribute ("ProbeRequestTimeout", "The duration to actively probe the channel.",
                   TimeValue (Seconds (0.05)),
                   MakeTimeAccessor (&StaWifiMac::m_probeRequestTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("WaitBeaconTimeout", "The duration to dwell on a channel while passively scanning for beacon",
                   TimeValue (MilliSeconds (120)),
                   MakeTimeAccessor (&StaWifiMac::m_waitBeaconTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("AssocRequestTimeout", "The interval between two consecutive association request attempts.",
                   TimeValue (Seconds (0.5)),
                   MakeTimeAccessor (&StaWifiMac::m_assocRequestTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("AdvanceWakeupPS", "The duration for which STA wakes up in advance before expected beacon arrival while in PS. Account for the beacon length here. ",
                   TimeValue (MicroSeconds (100)),
                   MakeTimeAccessor (&StaWifiMac::m_advanceWakeupPS),
                   MakeTimeChecker ())                                              
    .AddAttribute ("MaxMissedBeacons",
                   "Number of beacons which much be consecutively missed before "
                   "we attempt to restart association.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&StaWifiMac::m_maxMissedBeacons),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ActiveProbing",
                   "If true, we send probe requests. If false, we don't."
                   "NOTE: if more than one STA in your simulation is using active probing, "
                   "you should enable it at a different simulation time for each STA, "
                   "otherwise all the STAs will start sending probes at the same time resulting in collisions. "
                   "See bug 1060 for more info.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&StaWifiMac::SetActiveProbing, &StaWifiMac::GetActiveProbing),
                   MakeBooleanChecker ())
    .AddAttribute ("WakeForBeaconsInPowerSave",
                   "If true, STA wakes up for all beacons while in power save mode",
                   BooleanValue (true), 
                   MakeUintegerAccessor (&StaWifiMac::m_twtWakeForBeacons),
                   MakeBooleanChecker ())
    .AddTraceSource ("Assoc", "Associated with an access point.",
                     MakeTraceSourceAccessor (&StaWifiMac::m_assocLogger),
                     "ns3::Mac48Address::TracedCallback")
    .AddTraceSource ("DeAssoc", "Association with an access point lost.",
                     MakeTraceSourceAccessor (&StaWifiMac::m_deAssocLogger),
                     "ns3::Mac48Address::TracedCallback")
    .AddTraceSource ("BeaconArrival",
                     "Time of beacons arrival from associated AP",
                     MakeTraceSourceAccessor (&StaWifiMac::m_beaconArrival),
                     "ns3::Time::TracedCallback")
    .AddAttribute ("EnableUApsd",
                   "If true, U-APSD is assumed for the STA if using PSM. Else basic PSM is used",
                   BooleanValue (false),
                   MakeBooleanAccessor (&StaWifiMac::m_uApsd),
                   MakeBooleanChecker ())                     
  ;
  return tid;
}

StaWifiMac::StaWifiMac ()
  : m_state (UNASSOCIATED),
    m_aid (0),
    m_waitBeaconEvent (),
    m_probeRequestEvent (),
    m_assocRequestEvent (),
    m_nextExpectedBeaconGenerationEvent (),
    m_beaconWakeUpPsEvent (),
    m_apsdEospRxTimeoutEvent (),  // TODO remove apsd
    m_beaconWatchdogEnd (Seconds (0)),
    m_PSM (false),
    m_waitingToSwitchToPSM (false),
    m_waitingToSwitchOutOfPSM (false),
    m_pendingPsPoll (false),
    m_PsNeedToBeAwake (false),
    m_PsWaitingForPolledFrame (false),
    m_PsBeaconTimeOut (MilliSeconds (5)),   // shyam - hardcoded value
    m_uApsd (false),
    m_apsdEospTimeOut (MilliSeconds (20)), // #shyam #hardcoded value
    m_uApsdWaitingToSleepAfterAck (false),
    m_uApsdWaitingForEospFrame (false),
    m_twtAgreement (false),
    m_twtActive (false),
    m_awakeForTwtSp (false),
    m_twtWakeForBeacons (true),
    m_sleepAfterAck (false)
{
  NS_LOG_FUNCTION (this);

  //Let the lower layers know that we are acting as a non-AP STA in
  //an infrastructure BSS.
  SetTypeOfStation (STA);
  // Creating empty Unicast uplink TWT for PS stations
  // shyam - create only if TWT is supported
  m_twtPsBuffer = CreateObject<WifiMacQueue> (AC_BE);
  m_twtPsBuffer->SetMaxSize (QueueSize ("1000p"));    // shyam - make this an attribute - current attribute not working
  m_twtPsBuffer->SetMaxDelay (Time (MilliSeconds(20000)));   // shyam - make this an attribute - current attribute not working
  m_twtPsBuffer->SetAttribute ("DropPolicy", EnumValue (WifiMacQueue::DROP_OLDEST));
  NS_LOG_DEBUG ("Empty uplink TWT buffer created upon STA initialization. Max delay of Buffer is "<< m_twtPsBuffer->GetMaxDelay() );
  NS_LOG_DEBUG ("Number of packets in twt STA buffer now is "<< m_twtPsBuffer->GetNPackets() );
  NS_LOG_DEBUG ("Max Size of twt STA buffer is "<< m_twtPsBuffer->GetMaxSize ());

  // m_uniformRandomVariable initialization
  // m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

void
StaWifiMac::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  StartScanning ();
}

StaWifiMac::~StaWifiMac ()
{
  NS_LOG_FUNCTION (this);
}

uint16_t
StaWifiMac::GetAssociationId (void) const
{
  NS_ASSERT_MSG (IsAssociated (), "This station is not associated to any AP");
  return m_aid;
}

bool
StaWifiMac::IsAidInTim (Tim tim) const
{
  NS_LOG_FUNCTION (this << tim);
  uint32_t N1 = (tim.GetBitmapControl()>>1) << 1;   // Getting rid of the last bit. Making 7 bits to 8 bits by left shift is same as multiplying by 2
  uint32_t N2 = tim.GetInformationFieldSize() - 4 + N1;

  // NS_LOG_DEBUG ("N1 ="<<unsigned (N1)<<"; N2 ="<<unsigned (N2));
  uint8_t* pvb = (uint8_t*)malloc (tim.GetInformationFieldSize() - 3);
  pvb = tim.GetPVB();

  // NS_LOG_DEBUG ("PVB[0] as int: "<< unsigned(pvb[0]));

  // Create fullAidBitmap
  uint8_t fullAidBitmap [251] = {0};    // All entries set to zero

  for (uint32_t i = N1 ; i <= N2 ; i++)
  {
    fullAidBitmap[i] = pvb[i-N1];
  }

  // NS_LOG_DEBUG ("fullAidBitmap[0]: "<< unsigned(fullAidBitmap[0]));
  // NS_LOG_DEBUG("My GetAssociationId()::"<<GetAssociationId());

  uint8_t myOctet = 0;
  
  myOctet = fullAidBitmap[GetAssociationId()/8];
  // NS_LOG_DEBUG("myOctet::"<<unsigned(myOctet));
  myOctet = myOctet >> GetAssociationId()%8;
  // NS_LOG_DEBUG("myOctet::"<<unsigned(myOctet));
  if ((myOctet & 0x01) == 1)
  {
    NS_LOG_DEBUG ("Found my AID in TIM");
    return true;
  }
  NS_LOG_DEBUG ("My AID not found in TIM");
  return false;

}

void
StaWifiMac::SetActiveProbing (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_activeProbing = enable;
  if (m_state == WAIT_PROBE_RESP || m_state == WAIT_BEACON)
    {
      NS_LOG_DEBUG ("STA is still scanning, reset scanning process");
      StartScanning ();
    }
}

bool
StaWifiMac::GetActiveProbing (void) const
{
  return m_activeProbing;
}

void
StaWifiMac::SetWifiPhy (const Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  RegularWifiMac::SetWifiPhy (phy);
  m_phy->SetCapabilitiesChangedCallback (MakeCallback (&StaWifiMac::PhyCapabilitiesChanged, this));
}

void
StaWifiMac::SendProbeRequest (void)
{
  NS_LOG_FUNCTION (this);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_PROBE_REQUEST);
  hdr.SetAddr1 (Mac48Address::GetBroadcast ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (Mac48Address::GetBroadcast ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtProbeRequestHeader probe;
  probe.SetSsid (GetSsid ());
  probe.SetSupportedRates (GetSupportedRates ());
  if (GetHtSupported ())
    {
      probe.SetExtendedCapabilities (GetExtendedCapabilities ());
      probe.SetHtCapabilities (GetHtCapabilities ());
    }
  if (GetVhtSupported ())
    {
      probe.SetVhtCapabilities (GetVhtCapabilities ());
    }
  if (GetHeSupported ())
    {
      probe.SetHeCapabilities (GetHeCapabilities ());
    }
  packet->AddHeader (probe);

  if (!GetQosSupported ())
    {
      m_txop->Queue (packet, hdr);
    }
  // "A QoS STA that transmits a Management frame determines access category used
  // for medium access in transmission of the Management frame as follows
  // (If dot11QMFActivated is false or not present)
  // — If the Management frame is individually addressed to a non-QoS STA, category
  //   AC_BE should be selected.
  // — If category AC_BE was not selected by the previous step, category AC_VO
  //   shall be selected." (Sec. 10.2.3.2 of 802.11-2020)
  else
    {
      GetVOQueue ()->Queue (packet, hdr);
    }
}

void
StaWifiMac::SendAssociationRequest (bool isReassoc)
{
  NS_LOG_FUNCTION (this << GetBssid () << isReassoc);
  WifiMacHeader hdr;
  hdr.SetType (isReassoc ? WIFI_MAC_MGT_REASSOCIATION_REQUEST : WIFI_MAC_MGT_ASSOCIATION_REQUEST);
  hdr.SetAddr1 (GetBssid ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetBssid ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  if (!isReassoc)
    {
      MgtAssocRequestHeader assoc;
      assoc.SetSsid (GetSsid ());
      assoc.SetSupportedRates (GetSupportedRates ());
      assoc.SetCapabilities (GetCapabilities ());
      assoc.SetListenInterval (0);
      if (GetHtSupported ())
        {
          assoc.SetExtendedCapabilities (GetExtendedCapabilities ());
          assoc.SetHtCapabilities (GetHtCapabilities ());
        }
      if (GetVhtSupported ())
        {
          assoc.SetVhtCapabilities (GetVhtCapabilities ());
        }
      if (GetHeSupported ())
        {
          assoc.SetHeCapabilities (GetHeCapabilities ());
        }
      packet->AddHeader (assoc);
    }
  else
    {
      MgtReassocRequestHeader reassoc;
      reassoc.SetCurrentApAddress (GetBssid ());
      reassoc.SetSsid (GetSsid ());
      reassoc.SetSupportedRates (GetSupportedRates ());
      reassoc.SetCapabilities (GetCapabilities ());
      reassoc.SetListenInterval (0);
      if (GetHtSupported ())
        {
          reassoc.SetExtendedCapabilities (GetExtendedCapabilities ());
          reassoc.SetHtCapabilities (GetHtCapabilities ());
        }
      if (GetVhtSupported ())
        {
          reassoc.SetVhtCapabilities (GetVhtCapabilities ());
        }
      if (GetHeSupported ())
        {
          reassoc.SetHeCapabilities (GetHeCapabilities ());
        }
      packet->AddHeader (reassoc);
    }

  if (!GetQosSupported ())
    {
      m_txop->Queue (packet, hdr);
    }
  // "A QoS STA that transmits a Management frame determines access category used
  // for medium access in transmission of the Management frame as follows
  // (If dot11QMFActivated is false or not present)
  // — If the Management frame is individually addressed to a non-QoS STA, category
  //   AC_BE should be selected.
  // — If category AC_BE was not selected by the previous step, category AC_VO
  //   shall be selected." (Sec. 10.2.3.2 of 802.11-2020)
  else if (!m_stationManager->GetQosSupported (GetBssid ()))
    {
      GetBEQueue ()->Queue (packet, hdr);
    }
  else
    {
      GetVOQueue ()->Queue (packet, hdr);
    }

  if (m_assocRequestEvent.IsRunning ())
    {
      m_assocRequestEvent.Cancel ();
    }
  m_assocRequestEvent = Simulator::Schedule (m_assocRequestTimeout,
                                             &StaWifiMac::AssocRequestTimeout, this);
}

void
StaWifiMac::SendNullFramePS (bool enablePS)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("SendNullFramePS called with enablePS = "<<enablePS);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_DATA_NULL);
  hdr.SetAddr1 (GetBssid ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetBssid ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  if (enablePS)
  {
    hdr.SetPowerMgt ();
  }
  Ptr<Packet> packet = Create<Packet> ();

  if (!GetQosSupported ())
  {
    m_txop->Queue (packet, hdr);
  }

  else
  {

    GetBEQueue ()->QueueFront (packet, hdr); 

  }  
}

void
StaWifiMac::CheckPsPolledFrameReceived (void)
{
  NS_LOG_FUNCTION (this);
  if (m_PSM && m_PsWaitingForPolledFrame)
  {
    if (!GetQosSupported ())
    {
      m_txop->UpdateFailedCw();
    }

    else
    {
      GetBEQueue ()->UpdateFailedCw ();
    }
    SendPsPoll();
  }
}

 bool 
 StaWifiMac::isStaPsWaitingForPolledFrame ()
 {
   NS_LOG_FUNCTION (this<<m_PsWaitingForPolledFrame);
   return m_PsWaitingForPolledFrame;
 }

void
StaWifiMac::SendPsPoll (void)
{
  
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("SendPsPoll called");
  NS_LOG_DEBUG ("Waking up STA if it is in SLEEP");
  m_phy->ResumeFromSleep();
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_PSPOLL);
  hdr.SetAddr1 (GetBssid ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetBssid ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();

  Ptr<Packet> packet = Create<Packet> ();
  
  if (!GetQosSupported ())
  {
    m_txop->Queue (packet, hdr);
  }

  else
  {
    GetBEQueue ()->Queue (packet, hdr);
//    GetVOQueue ()->Queue (packet, hdr);
  }
  m_PsNeedToBeAwake = true;
  m_PsWaitingForPolledFrame = true;
  // shyam - hardcoded number here
  // Simulator::Schedule (MilliSeconds (1), &StaWifiMac::CheckPsPolledFrameReceived, this); // #fixAsap
}

void
StaWifiMac::TryToEnsureAssociated (void)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case ASSOCIATED:
      return;
      break;
    case WAIT_PROBE_RESP:
      /* we have sent a probe request earlier so we
         do not need to re-send a probe request immediately.
         We just need to wait until probe-request-timeout
         or until we get a probe response
       */
      break;
    case WAIT_BEACON:
      /* we have initiated passive scanning, continue to wait
         and gather beacons
       */
      break;
    case UNASSOCIATED:
      /* we were associated but we missed a bunch of beacons
       * so we should assume we are not associated anymore.
       * We try to initiate a scan now.
       */
      m_linkDown ();
      StartScanning ();
      break;
    case WAIT_ASSOC_RESP:
      /* we have sent an association request so we do not need to
         re-send an association request right now. We just need to
         wait until either assoc-request-timeout or until
         we get an association response.
       */
      break;
    case REFUSED:
      /* we have sent an association request and received a negative
         association response. We wait until someone restarts an
         association with a given SSID.
       */
      break;
    }
}
void 
StaWifiMac::NextExpectedBeaconGeneration ()
{
  NS_LOG_DEBUG ("NextExpectedBeaconGeneration called at STA MAC "<<this);
  return;
}

Time 
StaWifiMac::GetExpectedRemainingTimeTillNextBeacon(void) const
{
  NS_LOG_FUNCTION (this);
  // check if m_nextExpectedBeaconGenerationEvent is running. If not, throw error
  if (!m_nextExpectedBeaconGenerationEvent.IsRunning ())
  {
    NS_ABORT_MSG ("m_nextExpectedBeaconGenerationEvent is not running. Cannot get remaining time till next beacon. Check if last beacon was received at STA");
  }
  return Simulator::GetDelayLeft (m_nextExpectedBeaconGenerationEvent);
}

void
StaWifiMac::StartScanning (void)
{
  NS_LOG_FUNCTION (this);
  m_candidateAps.clear ();
  if (m_probeRequestEvent.IsRunning ())
    {
      m_probeRequestEvent.Cancel ();
    }
  if (m_waitBeaconEvent.IsRunning ())
    {
      m_waitBeaconEvent.Cancel ();
    }
  if (GetActiveProbing ())
    {
      SetState (WAIT_PROBE_RESP);
      SendProbeRequest ();
      m_probeRequestEvent = Simulator::Schedule (m_probeRequestTimeout,
                                                 &StaWifiMac::ScanningTimeout,
                                                 this);
    }
  else
    {
      SetState (WAIT_BEACON);
      m_waitBeaconEvent = Simulator::Schedule (m_waitBeaconTimeout,
                                               &StaWifiMac::ScanningTimeout,
                                               this);
    }
}

void
StaWifiMac::ScanningTimeout (void)
{
  NS_LOG_FUNCTION (this);
  if (!m_candidateAps.empty ())
    {
      ApInfo bestAp = m_candidateAps.front();
      m_candidateAps.erase(m_candidateAps.begin ());
      NS_LOG_DEBUG ("Attempting to associate with BSSID " << bestAp.m_bssid);
      Time beaconInterval;
      if (bestAp.m_activeProbing)
        {
          UpdateApInfoFromProbeResp (bestAp.m_probeResp, bestAp.m_apAddr, bestAp.m_bssid);
          beaconInterval = MicroSeconds (bestAp.m_probeResp.GetBeaconIntervalUs ());
        }
      else
        {
          UpdateApInfoFromBeacon (bestAp.m_beacon, bestAp.m_apAddr, bestAp.m_bssid);
          beaconInterval = MicroSeconds (bestAp.m_beacon.GetBeaconIntervalUs ());
        }

      Time delay = beaconInterval * m_maxMissedBeacons;
      RestartBeaconWatchdog (delay);
      SetState (WAIT_ASSOC_RESP);
      SendAssociationRequest (false);
    }
  else
    {
      NS_LOG_DEBUG ("Exhausted list of candidate AP; restart scanning");
      StartScanning ();
    }
}

void
StaWifiMac::AssocRequestTimeout (void)
{
  NS_LOG_FUNCTION (this);
  SetState (WAIT_ASSOC_RESP);
  SendAssociationRequest (false);
}

void
StaWifiMac::MissedBeacons (void)
{
  NS_LOG_FUNCTION (this);
  if (m_beaconWatchdogEnd > Simulator::Now ())
    {
      if (m_beaconWatchdog.IsRunning ())
        {
          m_beaconWatchdog.Cancel ();
        }
      m_beaconWatchdog = Simulator::Schedule (m_beaconWatchdogEnd - Simulator::Now (),
                                              &StaWifiMac::MissedBeacons, this);
      return;
    }
  NS_LOG_DEBUG ("beacon missed");
  // We need to switch to the UNASSOCIATED state. However, if we are receiving
  // a frame, wait until the RX is completed (otherwise, crashes may occur if
  // we are receiving a MU frame because its reception requires the STA-ID)
  Time delay = Seconds (0);
  if (m_phy->IsStateRx ())
    {
      delay = m_phy->GetDelayUntilIdle ();
    }
  Simulator::Schedule (delay, &StaWifiMac::SetState, this, UNASSOCIATED);
  Simulator::Schedule (delay, &StaWifiMac::TryToEnsureAssociated, this);
  // If STA is in PSM, wake it up
  if (m_PSM)
  {
    NS_LOG_DEBUG ("STA MAC "<<this<<" missed >= "<<m_maxMissedBeacons<<" beacons while in SLEEP (PSM). Waking the STA up now.");
    Simulator::Schedule (delay, &StaWifiMac::SetPowerSaveMode, this, false);
  }
}

void
StaWifiMac::RestartBeaconWatchdog (Time delay)
{
  NS_LOG_FUNCTION (this << delay);
  m_beaconWatchdogEnd = std::max (Simulator::Now () + delay, m_beaconWatchdogEnd);
  if (Simulator::GetDelayLeft (m_beaconWatchdog) < delay
      && m_beaconWatchdog.IsExpired ())
    {
      NS_LOG_DEBUG ("really restart watchdog.");
      m_beaconWatchdog = Simulator::Schedule (delay, &StaWifiMac::MissedBeacons, this);
    }
}

void
StaWifiMac::WakeForNextBeaconPS (void)
{
  NS_LOG_FUNCTION (this<<";m_PSM="<<m_PSM<<";m_twtAgreement="<<m_twtAgreement<<";m_twtWakeForBeacons="<<m_twtWakeForBeacons);
  // shyam - hardcoded - setting to not wake for beacons with PSM  ****************************************************************
  bool m_psmWakeForBeacons = true;
  if ( (m_PSM && m_psmWakeForBeacons) || (m_twtAgreement && m_twtWakeForBeacons))
  {
    // If PS = 1, wake up PHY. STA is still in PS = 1 state. PHY is woken up only to receive beacons. Upon successful beacom Rx while m_PSM = 1, STA is automatically put back to SLEEP. TIM and DTIM not yet implemented.
    NS_LOG_DEBUG ("STA is in PS = 1 or has twt agreement. Waking up PHY while in PS for beacon Rx. STA is woken up "<< m_advanceWakeupPS.GetNanoSeconds() << " nanoseconds in advance.");
    NS_LOG_DEBUG ("STA is in PS = 1. Waking up PHY while in PS for beacon Rx.");
    m_phy->ResumeFromSleep();
    m_PsNeedToBeAwake = true;

  }
  
}

void
StaWifiMac::CheckBeaconTimeOutPS (Time beaconInterval, Time beaconTimeOutPs)
{
  NS_LOG_FUNCTION (this<<beaconInterval<<beaconTimeOutPs);
  NS_LOG_DEBUG ("CheckBeaconTimeOutPS called");
  if (! (m_PSM || m_twtAgreement))
  {
    NS_LOG_DEBUG ("Warning; CheckBeaconTimeOutPS called without PSM or TWT");
    return;
  }
  if (m_beaconWakeUpPsEvent.IsRunning())
  {
    SleepPsIfQueuesEmpty();
    return;
  }
  else
  {
    // m_beaconWakeUpPsEvent not running - rescheduling
    NS_LOG_DEBUG ("beacon event not running. Scheduling after "<<(beaconInterval - beaconTimeOutPs).GetMicroSeconds()<<" us");
    m_beaconWakeUpPsEvent = Simulator::Schedule ((beaconInterval - beaconTimeOutPs), &StaWifiMac::WakeForNextBeaconPS, this);
    m_PsNeedToBeAwake = false;    // shyam - check
    SleepPsIfQueuesEmpty();
    return;
  }
}

void
StaWifiMac::SetPowerSaveMode (bool enable)
{
  NS_LOG_FUNCTION (this << enable);

  if (enable == m_PSM)
  {
    // STA is already in desired power mode (PSM or no PSM)
    NS_LOG_FUNCTION ("Station already is in PSM = "<<enable<<". SetPowerSaveMode() function returning without any change.");
    return;
  }

  if (enable && (m_state != ASSOCIATED) )
  {
    // If STA is not associated to an AP, it is not allowed to go to SLEEP state
    NS_LOG_FUNCTION ("Station is not ASSOCIATED to an AP. Station can enter SLEEP only after association. SetPowerSaveMode() function returning without any change.");
    return;
  }

  NS_LOG_FUNCTION ("PSM to be changed for MAC: "<<this<<" with PHY: "<<m_phy<<" from "<< m_PSM <<" to "<<enable);
  SendNullFramePS (enable);
  if (enable)
  {
    
    NS_LOG_FUNCTION ("PSM to be enabled. STA will be put to sleep by FeMan after receiving ACK for NDP.");
    m_waitingToSwitchToPSM = true;
    // NS_LOG_FUNCTION ("After switching: PHY IsStateSleep "<<m_phy->IsStateSleep());
  }
  else
  {
    NS_LOG_FUNCTION ("Before switching: PHY IsStateSleep "<<m_phy->IsStateSleep());
    m_phy->ResumeFromSleep();
    
    NS_LOG_FUNCTION ("After switching: PHY IsStateSleep "<<m_phy->IsStateSleep());
    m_waitingToSwitchOutOfPSM = true;
    // m_PSM = enable;
  }
  
  NS_LOG_DEBUG("After switching: m_PSM = "<<m_PSM<<"; m_phy->IsStateSleep()="<<m_phy->IsStateSleep());
  
  return;
}

bool
StaWifiMac::GetPowerSaveMode (void) const
{
  return m_PSM;
}

bool
StaWifiMac::GetTwtTriggerBased (void) const
{
  return m_twtTriggerBased;
}

bool
StaWifiMac::GetTwtSpNow (void) const
{
  return m_twtSpNow;
}

Time
StaWifiMac::GetRemainingTimeInTwtSp (void) const
{
  if (m_twtAgreement && m_twtSpNow && m_twtSpEndRoutineEvent.IsRunning())
  {
    // m_twtSpEndRoutineEvent.GetTs() - Simulator::Now(); 
    return Simulator::GetDelayLeft (m_twtSpEndRoutineEvent);
    
  }
  else
  {
    return MilliSeconds(0);
  }
}

bool
StaWifiMac::GetTwtAgreement (void) const
{
  return m_twtAgreement;
}

bool
StaWifiMac::IsTwtPsQueueEmpty (void) const
{
  if (!m_twtAgreement)
  {
    return true;
  }
  else
  {
    return (m_twtPsBuffer->GetNPackets() == 0);
  }
  
}


void 
StaWifiMac::StartTwtSchedule (WifiTwtAgreement agreement)
{
  NS_LOG_FUNCTION (this<<agreement);
  NS_LOG_DEBUG ("StartTwtSchedule called at STA MAC "<<this);
  NS_ASSERT_MSG (IsAssociated(), "STA not yet associated to an AP");
  NS_ASSERT_MSG (agreement.GetFlowId() < 8 && agreement.GetFlowId() >= 0, "Flow ID must be between 0 and 7");
  NS_ASSERT_MSG (agreement.IsIndividualAgreement(), "Broadcast TWT not supported yet");
  TwtFlowMacPair thisPairFlowMacPair (agreement.GetFlowId(), agreement.GetPeerMacAddress());
  if (m_twtAgreementMap.find (thisPairFlowMacPair) != m_twtAgreementMap.end())
  {
    // TwtFlowMacPair already exists - erase this agreement
    NS_LOG_DEBUG ("TWT agreement already exists for flowId "<<+agreement.GetFlowId()<<"; with MAC: "<<agreement.GetPeerMacAddress()<<". Replacing it with new agreement");
    m_twtAgreementMap.erase (thisPairFlowMacPair);
    // decrement m_twtAgreementCount for this peerAddress
    m_twtAgreementCount[agreement.GetPeerMacAddress()]--;
    
  }
  m_twtAgreementMap.insert ({thisPairFlowMacPair, agreement});
  // increment m_twtAgreementCount for this peerAddress
  m_twtAgreementCount[agreement.GetPeerMacAddress()]++;


  // From previous implementation
  m_twtAgreement = true;
  m_twtActive = true;
  m_twtTriggerBased = agreement.IsTriggerBasedAgreement();
  m_twtAnnounced = !agreement.GetFlowType();  //flowType is true for unannounced, false for announced
  
  Simulator::Schedule (agreement.GetNextTwt(), &StaWifiMac::WakeUpForTwtSp, this, agreement.GetWakeInterval(),  agreement.GetNominalWakeDuration());
  //TODO: put STA to sleep after TWT agreement is created
  // Simulator::Schedule (minTwtWakeDuration, &StaWifiMac::EndOfTwtSpRoutine, this); 
  return;
}

void
StaWifiMac::CreateTwtAgreement (uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt)
{
  NS_LOG_FUNCTION (this<<flowId<<peerMacAddress<<isRequestingNode<<isImplicitAgreement<<flowType<<isTriggerBasedAgreement<<isIndividualAgreement<<twtChannel<<wakeInterval<<nominalWakeDuration<<nextTwt);
  NS_ASSERT_MSG (IsAssociated(), "STA not yet associated to an AP");
  NS_ASSERT_MSG (flowId < 8 && flowId >= 0, "Flow ID must be between 0 and 7");
  NS_ASSERT_MSG (isRequestingNode == true, "At STA side, isRequestingNode must be true");
  NS_ASSERT_MSG (isIndividualAgreement, "Broadcast TWT not supported yet");
  // TODO: Add limit checks for wakeInterval and nominalWakeDuration
  
  // create WifiTwtAgreement object
  WifiTwtAgreement *agreement = new WifiTwtAgreement (flowId, peerMacAddress, isRequestingNode, isImplicitAgreement, flowType, isTriggerBasedAgreement, isIndividualAgreement, twtChannel, wakeInterval, nominalWakeDuration, nextTwt);

  // print created agreement to NS_LOG_DEBUG
  NS_LOG_DEBUG ("Created TWT agreement: "<<*agreement);


  // Find if TwtFlowMacPair already exists, if true, replace with new agreement
  TwtFlowMacPair thisPairFlowMacPair (flowId, peerMacAddress);
  if (m_twtAgreementMap.find (thisPairFlowMacPair) != m_twtAgreementMap.end())
  {
    // TwtFlowMacPair already exists - erase this agreement
    NS_LOG_DEBUG ("TWT agreement already exists for flowId "<<(int)flowId<<"; with MAC: "<<peerMacAddress<<". Replacing it with new agreement");
    m_twtAgreementMap.erase (thisPairFlowMacPair);
    // decrement m_twtAgreementCount for this peerAddress
    m_twtAgreementCount[peerMacAddress]--;
    
  }
  m_twtAgreementMap.insert ({thisPairFlowMacPair, *agreement});
  // increment m_twtAgreementCount for this peerAddress
  m_twtAgreementCount[peerMacAddress]++;


  // From previous implementation
  m_twtAgreement = true;
  m_twtActive = true;
  m_twtTriggerBased = isTriggerBasedAgreement;
  m_twtAnnounced = !flowType;  //flowType is true for unannounced, false for announced

  Simulator::Schedule (nextTwt, &StaWifiMac::WakeUpForTwtSp, this, wakeInterval,  nominalWakeDuration);
  //TODO: put STA to sleep after TWT agreement is created
  // Simulator::Schedule (minTwtWakeDuration, &StaWifiMac::EndOfTwtSpRoutine, this); 
  return;
}

void 
StaWifiMac::SetUpTwt (Time twtPeriod, Time minTwtWakeDuration , bool twtUplinkOutsideSp, bool twtWakeForBeacons, bool twtAnnounced, bool twtTriggerBased)
{

  // TODO - shyam - If STA does not support TWT requester, return with error
  NS_LOG_FUNCTION (this << twtPeriod<< minTwtWakeDuration );
  NS_LOG_DEBUG ("twtUplinkOutsideSp:"<<twtUplinkOutsideSp<<";twtWakeForBeacons:"<<twtWakeForBeacons<<";twtAnnounced:"<<twtAnnounced<<"; twtTriggerBased:"<<twtTriggerBased);
  m_twtAgreement = true;
  m_twtActive = true;
  m_awakeForTwtSp = true;
  // m_twtUplinkOutsideSp = twtUplinkOutsideSp;
  m_twtWakeForBeacons = twtWakeForBeacons;
  m_twtTriggerBased = twtTriggerBased;
  m_twtAnnounced = twtAnnounced;
  m_twtSpNow = true;
  if (twtAnnounced)
    {
      m_wakeUpForNextTwt = false;
      NS_LOG_DEBUG ("m_wakeUpForNextTwt : "<<m_wakeUpForNextTwt);
    }
  else
    {
      m_wakeUpForNextTwt = true;
      NS_LOG_DEBUG ("m_wakeUpForNextTwt : "<<m_wakeUpForNextTwt);
    }
  NS_LOG_FUNCTION ("m_awakeForTwtSp is now: "<<m_awakeForTwtSp);
  NS_LOG_FUNCTION ("TWT setup at STA with twtTriggerBased : "<<twtTriggerBased);
  NS_LOG_DEBUG ("m_twtSpNow is now: "<<m_twtSpNow);
  Simulator::Schedule (twtPeriod, &StaWifiMac::WakeUpForTwtSp, this, twtPeriod,  minTwtWakeDuration);
  Simulator::Schedule (minTwtWakeDuration, &StaWifiMac::EndOfTwtSpRoutine, this); 
  return;
}

void
StaWifiMac::WakeUpForTwtSp ( Time twtPeriod, Time minTwtWakeDuration)
{
  NS_LOG_FUNCTION (this  << twtPeriod<< minTwtWakeDuration);
  NS_LOG_DEBUG ("m_twtActive: "<<m_twtActive<<"; m_twtAnnounced: "<<m_twtAnnounced<<"; m_wakeUpForNextTwt: "<<m_wakeUpForNextTwt );
  m_twtSpNow = true;
  NS_LOG_DEBUG ("m_twtSpNow is now: "<<m_twtSpNow);
  
  // Scheduling Next TWT SP wake up
  Simulator::Schedule (twtPeriod, &StaWifiMac::WakeUpForTwtSp, this, twtPeriod, minTwtWakeDuration);
  // Scheduling Sleep at end of minTwtWakeDuration
  if (m_twtSpEndRoutineEvent.IsRunning())
  {
    // In case TWT ST is overlapping with itself
    m_twtSpEndRoutineEvent.Cancel();
  }
  m_twtSpEndRoutineEvent = Simulator::Schedule (minTwtWakeDuration, &StaWifiMac::EndOfTwtSpRoutine, this);

  if (m_twtActive && !m_twtAnnounced)
    {
      // If unannounced, wake up automatically
      m_awakeForTwtSp = true;
      NS_LOG_DEBUG ("Waking up for unannounced TWT SP: m_awakeForTwtSp is now: "<<m_awakeForTwtSp);
    }
  if (m_twtActive && m_twtAnnounced && m_wakeUpForNextTwt)
    {
      // If announced, and m_wakeUpForNextTwt is true, wake up
      m_awakeForTwtSp = true;
      NS_LOG_DEBUG ("Waking up for Announced TWT SP: m_awakeForTwtSp is now: "<<m_awakeForTwtSp);
    }


  if (m_awakeForTwtSp )
  {
    // Waking up from Sleep for TWT SP
    NS_LOG_FUNCTION ("Twt agreement active - waking up. Before switching: PHY IsStateSleep "<<m_phy->IsStateSleep());
    m_phy->ResumeFromSleep();
    // m_awakeForTwtSp = true;
    NS_LOG_FUNCTION ("m_awakeForTwtSp is now: "<<m_awakeForTwtSp);
    NS_LOG_FUNCTION ("After switching: PHY IsStateSleep "<<m_phy->IsStateSleep());





    // Check if TWT uplink buffer has packets
    uint32_t numPacketsTwtBuffer = m_twtPsBuffer->GetNPackets();
    NS_LOG_DEBUG ("#packets in twt uplink buffer is "<< numPacketsTwtBuffer );
    // NS_LOG_DEBUG ("Not transmitting packets from buffer at STA. Waiting for Trigger frame.");
    

    // Temporary logic to send UL frames without trigger - #shyam
    // for (uint32_t counterTwtPackets = 0; counterTwtPackets < numPacketsTwtBuffer; counterTwtPackets++)
    //   {
    //     Ptr<WifiMacQueueItem> TwtQueueItem;
    //     NS_LOG_DEBUG ("Sending Packet #"<<(counterTwtPackets+1)<<" of #"<<numPacketsTwtBuffer <<" to AP");
    //     TwtQueueItem = m_twtPsBuffer->Dequeue ();
    //     Ptr<Packet> packet = TwtQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet
    //     Enqueue (packet, TwtQueueItem->GetHeader().GetAddr1());
    //   }
  }
  return;
}

void 
StaWifiMac::SleepPsIfQueuesEmpty ()
{


  NS_LOG_FUNCTION (this);
  if ( !(m_PSM) )
  {
    NS_LOG_DEBUG ("STA not in PS. STA will not enter sleep even if Mac Queues are empty.");
    return;
  }
  else if (m_phy->IsStateSleep())
  {
    NS_LOG_DEBUG ("STA already in sleep. Returning without any action.");
    return;
  }
  
  else
  {
    NS_LOG_DEBUG(this<<"m_phy->IsStateTx() is "<<m_phy->IsStateTx());
    NS_LOG_DEBUG(this<<"m_PsNeedToBeAwake is "<<m_PsNeedToBeAwake);
    NS_LOG_DEBUG(this<<"IsEveryEdcaQueueEmpty() is "<< IsEveryEdcaQueueEmpty());
    // bool isTxTimerRunning = GetFrameExchangeManager()->GetWifiTxTimer ().IsRunning();
    if ( IsEveryEdcaQueueEmpty() && IsDcfQueueEmpty() &&  (m_PsNeedToBeAwake == false))
    {
      NS_LOG_DEBUG ("Qos and non Qos queues are empty and STA is in Power Save, and m_PsNeedToBeAwake is false; Switching STA to SLEEP if txTimer is not running.");
      // m_phy->SetSleepMode();
      SetPhyToSleep ();
      return;
    }
    // else
    // {
    
    //   Time difs = 2 * m_phy->GetSlot() + m_phy->GetSifs();     // shyam - hardcoded number here
    //   Time delay = 10 * difs;
    //   NS_LOG_DEBUG ("STA m_phy->IsStateTx() is "<<m_phy->IsStateTx()<<"\n(Mac queues are not empty and STA is in Power Save) or (STA is in Tx state). Rescheduling to check after : "<<delay << " m_PsNeedToBeAwake ="<<m_PsNeedToBeAwake);
    //   NS_LOG_DEBUG ("Delay until next idle is "<<m_phy->GetDelayUntilIdle().GetNanoSeconds());
    //   Simulator::Schedule(delay, &StaWifiMac::SleepPsIfQueuesEmpty, this);
    //   return;
    // }
  }
}

bool
StaWifiMac::IsAssociated (void) const
{
  return m_state == ASSOCIATED;
}

bool 
StaWifiMac::IsEveryEdcaQueueEmpty () const
{
  if ( !( GetQosSupported() ))
  {
    NS_LOG_DEBUG ("STA does not support Qos. Qos queues are empty. ");
    return true;
  }
  bool hasPackets = false;
  for (uint8_t tid = 0; tid < 8 ; tid++)
  {
    // GetQosTxop(tid)->GetWifiMacQueue()->Peek();
    if (GetQosTxop(tid)->HasFramesToTransmit())
    {
      hasPackets = true;
      NS_LOG_DEBUG ("Non empty Qos Queue found for tid = "<<unsigned (tid));
      std::stringstream tempMpduString;
//      GetQosTxop(tid)->PeekNextMpdu()->Print(tempMpduString);
//      NS_LOG_DEBUG ("MPDU: "<<tempMpduString.str());
      
      break;
    }
  } 
  // if hasPackets is false, then return true
  return !(hasPackets);
}

bool 
StaWifiMac::IsDcfQueueEmpty () const
{
  if (m_txop == nullptr)
  {
    NS_LOG_DEBUG ("Empty DCF queue since pointer does not exist.");
    return true; 
  }
  else
  {
    return !(m_txop->HasFramesToTransmit());  
  }
  // return !(GetTxop()->HasFramesToTransmit());
}


bool
StaWifiMac::IsWaitAssocResp (void) const
{
  return m_state == WAIT_ASSOC_RESP;
}

void
StaWifiMac::Enqueue (Ptr<Packet> packet, Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << to);
  // bool sendTwtResume = false;
  if (!IsAssociated ())
    {
      NotifyTxDrop (packet);
      TryToEnsureAssociated ();
      return;
    }

  
  // In case STA is in Power Save, there is no TWT agreement, and PHY is in SLEEP, wake while keeping m_PSM = 1
  if ( m_PSM == 1 && m_phy->IsStateSleep() && !m_twtAgreement )
    {
      NS_LOG_DEBUG ("A MAC packet was enqueued while STA was in m_PSM = 1 and PHY was in SLEEP. Waking up PHY while keeping m_PSM = 1");
    
      
      ResumePhyFromSleep();
      NS_LOG_DEBUG ("m_uApsd for STA is "<<m_uApsd);
      if (!m_uApsd)
      {
        m_sleepAfterAck = true;  
      }

      if (m_uApsd)
      {
        m_uApsdWaitingForEospFrame = true;
      }
      // if (m_uApsd)
      // {
      //   // m_apsdEospRxTimeoutEvent = Simulator::Schedule ();
        
      // }
      

      
    
    }
  
    
  WifiMacHeader hdr;

  //If we are not a QoS AP then we definitely want to use AC_BE to
  //transmit the packet. A TID of zero will map to AC_BE (through \c
  //QosUtilsMapTidToAc()), so we use that as our default here.
  uint8_t tid = 0;

  //For now, an AP that supports QoS does not support non-QoS
  //associations, and vice versa. In future the AP model should
  //support simultaneously associated QoS and non-QoS STAs, at which
  //point there will need to be per-association QoS state maintained
  //by the association state machine, and consulted here.
  if (GetQosSupported ())
    {
      hdr.SetType (WIFI_MAC_QOSDATA);
      hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
      hdr.SetQosNoEosp ();
      hdr.SetQosNoAmsdu ();
      //Transmission of multiple frames in the same TXOP is not
      //supported for now
      hdr.SetQosTxopLimit (0);

      //Fill in the QoS control field in the MAC header
      tid = QosUtilsGetTidForPacket (packet);
      //Any value greater than 7 is invalid and likely indicates that
      //the packet had no QoS tag, so we revert to zero, which'll
      //mean that AC_BE is used.
      if (tid > 7)
        {
          tid = 0;
        }
      hdr.SetQosTid (tid);
    }
  else
    {
      hdr.SetType (WIFI_MAC_DATA);
    }
  if (GetQosSupported ())
    {
      hdr.SetNoOrder (); // explicitly set to 0 for the time being since HT control field is not yet implemented (set it to 1 when implemented)
    }

  hdr.SetAddr1 (GetBssid ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (to);
  hdr.SetDsNotFrom ();
  hdr.SetDsTo ();
  if (m_PSM && !m_twtAgreement )
  {
    // If only PSM is used and not TWT, set PM to 1
    hdr.SetPowerMgt ();
  }

  if (GetQosSupported ())
    {
      // NS_LOG_DEBUG ("Before enqueuing Qos packet, IsEveryEdcaQueueEmpty() returns: "<< IsEveryEdcaQueueEmpty());
      //Sanity check that the TID is valid
      // NS_LOG_DEBUG ("Enqueuing packet in QoS queue with tid: "<<unsigned(tid));
      NS_ASSERT (tid < 8);
      
      
      m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);

    }
  else
    {

      // NS_LOG_DEBUG ("Before enqueuing non Qos packet, IsDcfQueueEmpty() returns: "<< IsDcfQueueEmpty());
      NS_LOG_DEBUG ("Enqueuing packet in Non-QoS queue");

      m_txop->Queue (packet, hdr);

    }

}

void
StaWifiMac::Receive (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  const WifiMacHeader* hdr = &mpdu->GetHeader ();
  Ptr<const Packet> packet = mpdu->GetPacket ();
  NS_ASSERT (!hdr->IsCtl () || hdr->IsAck()); // shyam
  NS_LOG_DEBUG ("hdr->GetAddr1 ()="<<hdr->GetAddr1 ());
  if (hdr->GetAddr3 () == GetAddress ())
    {
      NS_LOG_LOGIC ("packet sent by us.");
      
      // If packet is sent by us and is being sent back by AP
      // ------------------------------------------- PSM - multicast from self case 
      if (hdr->GetAddr2 () == GetBssid () && hdr->GetAddr1 ().IsGroup () &&  m_PSM && !(hdr->IsMoreData()))
      {
        NS_LOG_DEBUG ("STA is in PS = 1; Received a group addressed packet from same BSSID from myself with more data field = 0;");
        
        if (m_pendingPsPoll)
        {
          NS_LOG_DEBUG ("m_pendingPsPoll was true - not going to sleep - sending PsPoll");
          SendPsPoll();
        }
        else
        {
          NS_LOG_DEBUG ("m_pendingPsPoll was false - going to sleep.");
          StaWifiMac::SleepAfterAckIfRequired ();

        }
        
      }
      // ------------------------------------------- 


      return;
    }
  else if (hdr->GetAddr1 () != GetAddress ()
           && !hdr->GetAddr1 ().IsGroup ())
    {

      // if (m_PSM && m_PsWaitingForPolledFrame)
      // {
      //   // Packet sent by AP but not for us
      //   NS_LOG_DEBUG ("Other frame transmissions detected while STA is waiting for response for PS-POLL. Resending PS-POLL");
      //   SendPsPoll ();

      // }
      NS_LOG_LOGIC ("packet is not for us");
      NotifyRxDrop (packet);
      return;
    }

  if (hdr->IsAck())
  {
    NS_LOG_DEBUG ("Passed up an ACK to STA MAC higher layer - do nothing.");
    return;
  }
  
  // ------------------------------------------- PSM - unicast case - assume any multicast, if present was already received
  if (hdr->GetAddr2 () == GetBssid () && hdr->GetAddr1 () == GetAddress () && m_PSM && hdr->IsMoreData())   
  {
    // Packet is for us, STA is in PSM and More data field is 1
    // NS_LOG_DEBUG ("Received unicast packet with More data = 1; Sending Ps Poll.");
    NS_LOG_DEBUG ("Received unicast packet with More data = 1; Staying awake");
    m_PsWaitingForPolledFrame = false;
    if (!m_uApsd)
    {
      SendPsPoll();   //comment for dynamicPSM
    }
    
    m_PsNeedToBeAwake = true;
  }
  if (hdr->GetAddr2 () == GetBssid () && hdr->GetAddr1 () == GetAddress () && m_PSM && !(hdr->IsMoreData()))   
  {
    // Packet is for us, STA is in PSM and More data field is 0 
    NS_LOG_DEBUG ("Received unicast packet with More data = 1; Staying awake");
    m_PsWaitingForPolledFrame = false;
        
    m_PsNeedToBeAwake = false;
    m_sleepAfterAck = true;
  }

  else if (hdr->GetAddr2 () == GetBssid () && hdr->GetAddr1 () == GetAddress () && m_PSM  && m_uApsd && hdr->IsQosData())   
  {
    
    if (hdr->IsQosEosp())
    { 
      NS_LOG_DEBUG ("Received unicast packet with EOSP = 1 while in APSD;");
      m_uApsdWaitingForEospFrame = false;
      
      m_sleepAfterAck = true;
      m_pendingPsPoll = false;
      m_PsNeedToBeAwake = false;
      m_PsWaitingForPolledFrame = false;

      m_uApsdWaitingToSleepAfterAck = true;
      
    }
    
    
  }
  // -------------------------------------------  
  
  if (hdr->IsData ())
    {
      if (!IsAssociated ())
        {
          NS_LOG_LOGIC ("Received data frame while not associated: ignore");
          NotifyRxDrop (packet);
          return;
        }
      if (!(hdr->IsFromDs () && !hdr->IsToDs ()))
        {
          NS_LOG_LOGIC ("Received data frame not from the DS: ignore");
          NotifyRxDrop (packet);
          return;
        }
      if (hdr->GetAddr2 () != GetBssid ())
        {
          NS_LOG_LOGIC ("Received data frame not from the BSS we are associated with: ignore");
          NotifyRxDrop (packet);
          return;
        }
      if (hdr->IsQosData ())
        {
          if (hdr->IsQosAmsdu ())
            {
              NS_ASSERT (hdr->GetAddr3 () == GetBssid ());
              DeaggregateAmsduAndForward (mpdu);
              packet = 0;
            }
          else
            {
              ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
            }
        }
      else if (hdr->HasData ())
        {
          ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
        }
      

      // ----------------------------------- PSM multicast case 
      if (hdr->GetAddr2 () == GetBssid () && hdr->GetAddr1 ().IsGroup () &&  m_PSM && !(hdr->IsMoreData()))
      {
        NS_LOG_DEBUG ("STA is in PS = 1; Received a group addressed packet from same BSSID with more data field = 0;");
        
        if (m_pendingPsPoll)
        {
          NS_LOG_DEBUG ("m_pendingPsPoll was true - not going to sleep - sending PsPoll");
          SendPsPoll();
          m_PsNeedToBeAwake = true;
        }
        else
        {
          NS_LOG_DEBUG ("m_pendingPsPoll was false - going to sleep.");

          StaWifiMac::SleepAfterAckIfRequired ();

          m_PsNeedToBeAwake = false;
        }

      }
      // -----------------------------------
      // // -----------------------------------TWT EOSP handling
      // if (hdr->IsQosData () && hdr->IsQosEosp() && GetTwtAgreement() && m_twtSpNow)
      // {
      //   NS_LOG_DEBUG ("TWT SP active now - received QoS EOSP frame. Entering sleep.");
        
      //   SleepWithinTwtSpNow ();
      // }

      // // -----------------------------------TWT EOSP handling

      return;
    }
  else if (hdr->IsProbeReq ()
           || hdr->IsAssocReq ()
           || hdr->IsReassocReq ())
    {
      //This is a frame aimed at an AP, so we can safely ignore it.
      NotifyRxDrop (packet);
      return;
    }
  else if (hdr->IsBeacon ())
    {
      NS_LOG_DEBUG ("Beacon received");
      MgtBeaconHeader beacon;
      Ptr<Packet> copy = packet->Copy ();
      copy->RemoveHeader (beacon);
      CapabilityInformation capabilities = beacon.GetCapabilities ();
      NS_ASSERT (capabilities.IsEss ());
      bool goodBeacon = false;
      if (GetSsid ().IsBroadcast ()
          || beacon.GetSsid ().IsEqual (GetSsid ()))
        {
          NS_LOG_LOGIC ("Beacon is for our SSID");
          goodBeacon = true;
        }
      SupportedRates rates = beacon.GetSupportedRates ();
      bool bssMembershipSelectorMatch = false;
      auto selectorList = m_phy->GetBssMembershipSelectorList ();
      for (const auto & selector : selectorList)
        {
          if (rates.IsBssMembershipSelectorRate (selector))
            {
              NS_LOG_LOGIC ("Beacon is matched to our BSS membership selector");
              bssMembershipSelectorMatch = true;
            }
        }
      if (selectorList.size () > 0 && bssMembershipSelectorMatch == false)
        {
          NS_LOG_LOGIC ("No match for BSS membership selector");
          goodBeacon = false;
        }
      if ((IsWaitAssocResp () || IsAssociated ()) && hdr->GetAddr3 () != GetBssid ())
        {
          NS_LOG_LOGIC ("Beacon is not for us");
          goodBeacon = false;
        }
      if (goodBeacon && m_state == ASSOCIATED)
        {
          NS_LOG_DEBUG ("Received good beacon while associated");
          m_beaconArrival (Simulator::Now ());
          Time delay = MicroSeconds (beacon.GetBeaconIntervalUs () * m_maxMissedBeacons);
          Time beaconTimeStamp = MicroSeconds (beacon.GetTimestamp());
          // Setting m_expectedRemainingTimeTillNextBeacon
          Time expectedRemainingTimeTillNextBeacon = beaconTimeStamp + MicroSeconds (beacon.GetBeaconIntervalUs ()) -  Simulator::Now();
          m_nextExpectedBeaconGenerationEvent = Simulator::Schedule (expectedRemainingTimeTillNextBeacon, &StaWifiMac::NextExpectedBeaconGeneration, this);
          // Time nextBeaconArrivalAdjusted = beaconTimeStamp + MicroSeconds (beacon.GetBeaconIntervalUs ());
          Time nextBeaconArrivalAdjusted = beaconTimeStamp + MicroSeconds (beacon.GetBeaconIntervalUs ()) - m_advanceWakeupPS;  // This is required to avoid a bug with preamble detection
          
          NS_LOG_DEBUG ("At STA, received beaconTimeStamp in us = "<<beaconTimeStamp.GetMicroSeconds());
          NS_LOG_DEBUG ("Next beacon should arrive no earlier than at time="<<nextBeaconArrivalAdjusted);
          
          Time nextBeaconWakeUp = nextBeaconArrivalAdjusted -  Simulator::Now();
          RestartBeaconWatchdog (delay);
          UpdateApInfoFromBeacon (beacon, hdr->GetAddr2 (), hdr->GetAddr3 ());
          if (nextBeaconWakeUp.IsNegative())
          {
            NS_LOG_DEBUG ("warning: nextBeaconWakeUp is negative.");
            nextBeaconWakeUp = Seconds (0);
          }
          
          NS_LOG_DEBUG ("m_PSM="<<m_PSM<<"; m_twtAgreement"<<m_twtAgreement<<"; m_twtWakeForBeacons="<<m_twtWakeForBeacons);

          // if (m_PSM || (m_twtAgreement && m_twtWakeForBeacons))
            // {
          NS_LOG_DEBUG ("STA, if in PSM, is scheduled to wake up after "<<nextBeaconWakeUp.GetNanoSeconds()<<" nanoseconds from now for next beacon Rx.");
          m_beaconWakeUpPsEvent = Simulator::Schedule (nextBeaconWakeUp, &StaWifiMac::WakeForNextBeaconPS, this);
          if (m_PSM || m_twtAgreement)
          {
            Simulator::Schedule ((nextBeaconWakeUp + m_PsBeaconTimeOut), &StaWifiMac::CheckBeaconTimeOutPS, this, MicroSeconds (beacon.GetBeaconIntervalUs ()), m_PsBeaconTimeOut);
          }
          
            // }
          
          // Check beacon for TIM 
          NS_LOG_DEBUG ("Multicast bit in beacon is: "<<beacon.GetTim().GetMulticastBuffered());

          bool waitForMulticast = beacon.GetTim().GetMulticastBuffered() && (beacon.GetTim().GetDtimCount() == 0);    // If waitForMulticast is true, STA has to stay awake till all multicast packets re delivered from AP
          if (m_twtAgreement && !waitForMulticast && !m_twtSpNow) 
          {
            // TWT case
            // Simulator::Schedule (MicroSeconds (1),&StaWifiMac::SetPhyToSleep, this);
            m_phy->SetSleepMode();
            NS_LOG_DEBUG("STA has TWT agreement. SP is not active now. Beacon received. Multicast bit is 0 in beacon (if it is DTIM beacon). Beacon is received successfully. STA going back to Sleep. STA m_phy->IsStateSleep() is now "<<m_phy->IsStateSleep());
            m_PsNeedToBeAwake = false;
          }
          
          if (m_PSM && !(IsAidInTim (beacon.GetTim ())) && !waitForMulticast && !m_uApsdWaitingForEospFrame ) 
          {
            m_phy->SetSleepMode();
            NS_LOG_DEBUG("STA is in PSM or U-APSD. Beacon received. STA is not waiting for EOSP QoS frame. AID was not in the TIM and Multicast bit is 0 in beacon (if it is DTIM beacon). Beacon is received successfully. STA going back to Sleep. STA m_phy->IsStateSleep() is now "<<m_phy->IsStateSleep());
            m_PsNeedToBeAwake = false;
          }
          else if (IsAidInTim (beacon.GetTim ()) && !waitForMulticast )
          {
            // shyam
            // NS_LOG_DEBUG ("AID found in TIM of beacon and no multicast buffered data. Sending one PS-POLL if no TWT agreement.");
            NS_LOG_DEBUG ("AID found in TIM of beacon and no multicast buffered data. Switching to awake state and sending NDP.");
            if (!m_twtAgreement)
              {

                
                SendPsPoll();    // instantly queue Ps poll
                //dynamicPSM
                // SendNullFramePS (false);   // Temporarily entering awake state
                m_PsNeedToBeAwake = true;
              }
          }
          else if (IsAidInTim (beacon.GetTim ()) && waitForMulticast )
          {
            NS_LOG_DEBUG ("AID found in TIM of beacon but multicast buffered data also detected. setting flag m_pendingPsPoll.");
            if (!m_twtAgreement)
              {
                NS_LOG_DEBUG ("Setting m_pendingPsPoll = true");
                m_pendingPsPoll = true;
              }
            
            m_PsNeedToBeAwake = true;
          }
        }
      if (goodBeacon && m_state == WAIT_BEACON)
        {
          NS_LOG_DEBUG ("Beacon received while scanning from " << hdr->GetAddr2 ());
          SnrTag snrTag;
          bool removed = copy->RemovePacketTag (snrTag);
          NS_ASSERT (removed);
          ApInfo apInfo;
          apInfo.m_apAddr = hdr->GetAddr2 ();
          apInfo.m_bssid = hdr->GetAddr3 ();
          apInfo.m_activeProbing = false;
          apInfo.m_snr = snrTag.Get ();
          apInfo.m_beacon = beacon;
          UpdateCandidateApList (apInfo);
        }
      return;
    }
  else if (hdr->IsProbeResp ())
    {
      if (m_state == WAIT_PROBE_RESP)
        {
          NS_LOG_DEBUG ("Probe response received while scanning from " << hdr->GetAddr2 ());
          MgtProbeResponseHeader probeResp;
          Ptr<Packet> copy = packet->Copy ();
          copy->RemoveHeader (probeResp);
          if (!probeResp.GetSsid ().IsEqual (GetSsid ()))
            {
              NS_LOG_DEBUG ("Probe response is not for our SSID");
              return;
            }
          SnrTag snrTag;
          bool removed = copy->RemovePacketTag (snrTag);
          NS_ASSERT (removed);
          ApInfo apInfo;
          apInfo.m_apAddr = hdr->GetAddr2 ();
          apInfo.m_bssid = hdr->GetAddr3 ();
          apInfo.m_activeProbing = true;
          apInfo.m_snr = snrTag.Get ();
          apInfo.m_probeResp = probeResp;
          UpdateCandidateApList (apInfo);
        }
      return;
    }
  // else if (hdr->IsAction ())
  //   {
      
  //     return;
  //   }
  else if (hdr->IsAssocResp () || hdr->IsReassocResp ())
    {
      if (m_state == WAIT_ASSOC_RESP)
        {
          MgtAssocResponseHeader assocResp;
          packet->PeekHeader (assocResp);
          if (m_assocRequestEvent.IsRunning ())
            {
              m_assocRequestEvent.Cancel ();
            }
          if (assocResp.GetStatusCode ().IsSuccess ())
            {
              SetState (ASSOCIATED);
              m_aid = assocResp.GetAssociationId ();
              //shyam - for random ps poll Tx
              // m_uniformRandomVariable->SetStream (GetAssociationId());
              //shyam - for random ps poll Tx
              if (hdr->IsReassocResp ())
                {
                  NS_LOG_DEBUG ("reassociation done");
                }
              else
                {
                  NS_LOG_DEBUG ("association completed");
                }
              UpdateApInfoFromAssocResp (assocResp, hdr->GetAddr2 ());
              if (!m_linkUp.IsNull ())
                {
                  m_linkUp ();
                }
            }
          else
            {
              NS_LOG_DEBUG ("association refused");
              if (m_candidateAps.empty ())
                {
                  SetState (REFUSED);
                }
              else
                {
                  ScanningTimeout ();
                }
            }
        }
      return;
    }
  // Case: STA has TWT agreement, received frame is a basic trigger frame
  // else if (hdr->IsTrigger () && m_twtActive)
  //   {
  //     NS_LOG_DEBUG ("STA with TWT agreement received a Trigger frame.");
      
  //   }

  //Invoke the receive handler of our parent class to deal with any
  //other frames. Specifically, this will handle Block Ack-related
  //Management Action frames.
  RegularWifiMac::Receive (Create<WifiMacQueueItem> (packet, *hdr));
}

void
StaWifiMac::UpdateCandidateApList (ApInfo newApInfo)
{
  NS_LOG_FUNCTION (this << newApInfo.m_bssid << newApInfo.m_apAddr << newApInfo.m_snr << newApInfo.m_activeProbing << newApInfo.m_beacon << newApInfo.m_probeResp);
  // Remove duplicate ApInfo entry
  for (std::vector<ApInfo>::iterator i = m_candidateAps.begin(); i != m_candidateAps.end(); ++i)
    {
      if (newApInfo.m_bssid == (*i).m_bssid)
        {
          m_candidateAps.erase(i);
          break;
        }
    }
  // Insert before the entry with lower SNR
  for (std::vector<ApInfo>::iterator i = m_candidateAps.begin(); i != m_candidateAps.end(); ++i)
    {
      if (newApInfo.m_snr > (*i).m_snr)
        {
          m_candidateAps.insert (i, newApInfo);
          return;
        }
    }
  // If new ApInfo is the lowest, insert at back
  m_candidateAps.push_back(newApInfo);
}

void
StaWifiMac::UpdateApInfoFromBeacon (MgtBeaconHeader beacon, Mac48Address apAddr, Mac48Address bssid)
{
  NS_LOG_FUNCTION (this << beacon << apAddr << bssid);
  SetBssid (bssid);
  CapabilityInformation capabilities = beacon.GetCapabilities ();
  SupportedRates rates = beacon.GetSupportedRates ();
  for (const auto & mode : m_phy->GetModeList ())
    {
      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
        {
          m_stationManager->AddSupportedMode (apAddr, mode);
        }
    }
  bool isShortPreambleEnabled = capabilities.IsShortPreamble ();
  if (GetErpSupported ())
    {
      ErpInformation erpInformation = beacon.GetErpInformation ();
      isShortPreambleEnabled &= !erpInformation.GetBarkerPreambleMode ();
      if (erpInformation.GetUseProtection () != 0)
        {
          m_stationManager->SetUseNonErpProtection (true);
        }
      else
        {
          m_stationManager->SetUseNonErpProtection (false);
        }
      if (capabilities.IsShortSlotTime () == true)
        {
          //enable short slot time
          m_phy->SetSlot (MicroSeconds (9));
        }
      else
        {
          //disable short slot time
          m_phy->SetSlot (MicroSeconds (20));
        }
    }
  if (GetQosSupported ())
    {
      bool qosSupported = false;
      EdcaParameterSet edcaParameters = beacon.GetEdcaParameterSet ();
      if (edcaParameters.IsQosSupported ())
        {
          qosSupported = true;
          //The value of the TXOP Limit field is specified as an unsigned integer, with the least significant octet transmitted first, in units of 32 μs.
          SetEdcaParameters (AC_BE, edcaParameters.GetBeCWmin (), edcaParameters.GetBeCWmax (), edcaParameters.GetBeAifsn (), 32 * MicroSeconds (edcaParameters.GetBeTxopLimit ()));
          SetEdcaParameters (AC_BK, edcaParameters.GetBkCWmin (), edcaParameters.GetBkCWmax (), edcaParameters.GetBkAifsn (), 32 * MicroSeconds (edcaParameters.GetBkTxopLimit ()));
          SetEdcaParameters (AC_VI, edcaParameters.GetViCWmin (), edcaParameters.GetViCWmax (), edcaParameters.GetViAifsn (), 32 * MicroSeconds (edcaParameters.GetViTxopLimit ()));
          SetEdcaParameters (AC_VO, edcaParameters.GetVoCWmin (), edcaParameters.GetVoCWmax (), edcaParameters.GetVoAifsn (), 32 * MicroSeconds (edcaParameters.GetVoTxopLimit ()));
        }
      m_stationManager->SetQosSupport (apAddr, qosSupported);
    }
  if (GetHtSupported ())
    {
      HtCapabilities htCapabilities = beacon.GetHtCapabilities ();
      if (!htCapabilities.IsSupportedMcs (0))
        {
          m_stationManager->RemoveAllSupportedMcs (apAddr);
        }
      else
        {
          m_stationManager->AddStationHtCapabilities (apAddr, htCapabilities);
        }
    }
  if (GetVhtSupported ())
    {
      VhtCapabilities vhtCapabilities = beacon.GetVhtCapabilities ();
      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
        {
          m_stationManager->AddStationVhtCapabilities (apAddr, vhtCapabilities);
          VhtOperation vhtOperation = beacon.GetVhtOperation ();
          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_VHT))
            {
              if (vhtCapabilities.IsSupportedRxMcs (mcs.GetMcsValue ()))
                {
                  m_stationManager->AddSupportedMcs (apAddr, mcs);
                }
            }
        }
    }
  if (GetHtSupported ())
    {
      ExtendedCapabilities extendedCapabilities = beacon.GetExtendedCapabilities ();
      //TODO: to be completed
    }
  if (GetHeSupported ())
    {
      HeCapabilities heCapabilities = beacon.GetHeCapabilities ();
      if (heCapabilities.GetSupportedMcsAndNss () != 0)
        {
          m_stationManager->AddStationHeCapabilities (apAddr, heCapabilities);
          HeOperation heOperation = beacon.GetHeOperation ();
          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HE))
            {
              if (heCapabilities.IsSupportedRxMcs (mcs.GetMcsValue ()))
                {
                  m_stationManager->AddSupportedMcs (apAddr, mcs);
                }
            }
        }

      MuEdcaParameterSet muEdcaParameters = beacon.GetMuEdcaParameterSet ();
      if (muEdcaParameters.IsPresent ())
        {
          SetMuEdcaParameters (AC_BE, muEdcaParameters.GetMuCwMin (AC_BE), muEdcaParameters.GetMuCwMax (AC_BE),
                               muEdcaParameters.GetMuAifsn (AC_BE), muEdcaParameters.GetMuEdcaTimer (AC_BE));
          SetMuEdcaParameters (AC_BK, muEdcaParameters.GetMuCwMin (AC_BK), muEdcaParameters.GetMuCwMax (AC_BK),
                               muEdcaParameters.GetMuAifsn (AC_BK), muEdcaParameters.GetMuEdcaTimer (AC_BK));
          SetMuEdcaParameters (AC_VI, muEdcaParameters.GetMuCwMin (AC_VI), muEdcaParameters.GetMuCwMax (AC_VI),
                               muEdcaParameters.GetMuAifsn (AC_VI), muEdcaParameters.GetMuEdcaTimer (AC_VI));
          SetMuEdcaParameters (AC_VO, muEdcaParameters.GetMuCwMin (AC_VO), muEdcaParameters.GetMuCwMax (AC_VO),
                               muEdcaParameters.GetMuAifsn (AC_VO), muEdcaParameters.GetMuEdcaTimer (AC_VO));
        }
    }
  m_stationManager->SetShortPreambleEnabled (isShortPreambleEnabled);
  m_stationManager->SetShortSlotTimeEnabled (capabilities.IsShortSlotTime ());
}

void
StaWifiMac::UpdateApInfoFromProbeResp (MgtProbeResponseHeader probeResp, Mac48Address apAddr, Mac48Address bssid)
{
  NS_LOG_FUNCTION (this << probeResp << apAddr << bssid);
  CapabilityInformation capabilities = probeResp.GetCapabilities ();
  SupportedRates rates = probeResp.GetSupportedRates ();
  for (const auto & selector : m_phy->GetBssMembershipSelectorList ())
    {
      if (!rates.IsBssMembershipSelectorRate (selector))
        {
          NS_LOG_DEBUG ("Supported rates do not fit with the BSS membership selector");
          return;
        }
    }
  for (const auto & mode : m_phy->GetModeList ())
    {
      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
        {
          m_stationManager->AddSupportedMode (apAddr, mode);
          if (rates.IsBasicRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
            {
              m_stationManager->AddBasicMode (mode);
            }
        }
    }

  bool isShortPreambleEnabled = capabilities.IsShortPreamble ();
  if (GetErpSupported ())
    {
      bool isErpAllowed = false;
      for (const auto & mode : m_phy->GetModeList (WIFI_MOD_CLASS_ERP_OFDM))
        {
          if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
            {
              isErpAllowed = true;
              break;
            }
        }
      if (!isErpAllowed)
        {
          //disable short slot time and set cwMin to 31
          m_phy->SetSlot (MicroSeconds (20));
          ConfigureContentionWindow (31, 1023);
        }
      else
        {
          ErpInformation erpInformation = probeResp.GetErpInformation ();
          isShortPreambleEnabled &= !erpInformation.GetBarkerPreambleMode ();
          if (m_stationManager->GetShortSlotTimeEnabled ())
            {
              //enable short slot time
              m_phy->SetSlot (MicroSeconds (9));
            }
          else
            {
              //disable short slot time
              m_phy->SetSlot (MicroSeconds (20));
            }
          ConfigureContentionWindow (15, 1023);
        }
    }
  m_stationManager->SetShortPreambleEnabled (isShortPreambleEnabled);
  m_stationManager->SetShortSlotTimeEnabled (capabilities.IsShortSlotTime ());
  SetBssid (bssid);
}

void
StaWifiMac::UpdateApInfoFromAssocResp (MgtAssocResponseHeader assocResp, Mac48Address apAddr)
{
  NS_LOG_FUNCTION (this << assocResp << apAddr);
  CapabilityInformation capabilities = assocResp.GetCapabilities ();
  SupportedRates rates = assocResp.GetSupportedRates ();
  bool isShortPreambleEnabled = capabilities.IsShortPreamble ();
  if (GetErpSupported ())
    {
      bool isErpAllowed = false;
      for (const auto & mode : m_phy->GetModeList (WIFI_MOD_CLASS_ERP_OFDM))
        {
          if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
            {
              isErpAllowed = true;
              break;
            }
        }
      if (!isErpAllowed)
        {
          //disable short slot time and set cwMin to 31
          m_phy->SetSlot (MicroSeconds (20));
          ConfigureContentionWindow (31, 1023);
        }
      else
        {
          ErpInformation erpInformation = assocResp.GetErpInformation ();
          isShortPreambleEnabled &= !erpInformation.GetBarkerPreambleMode ();
          if (m_stationManager->GetShortSlotTimeEnabled ())
            {
              //enable short slot time
              m_phy->SetSlot (MicroSeconds (9));
            }
          else
            {
              //disable short slot time
              m_phy->SetSlot (MicroSeconds (20));
            }
          ConfigureContentionWindow (15, 1023);
        }
    }
  m_stationManager->SetShortPreambleEnabled (isShortPreambleEnabled);
  m_stationManager->SetShortSlotTimeEnabled (capabilities.IsShortSlotTime ());
  if (GetQosSupported ())
    {
      bool qosSupported = false;
      EdcaParameterSet edcaParameters = assocResp.GetEdcaParameterSet ();
      if (edcaParameters.IsQosSupported ())
        {
          qosSupported = true;
          //The value of the TXOP Limit field is specified as an unsigned integer, with the least significant octet transmitted first, in units of 32 μs.
          SetEdcaParameters (AC_BE, edcaParameters.GetBeCWmin (), edcaParameters.GetBeCWmax (), edcaParameters.GetBeAifsn (), 32 * MicroSeconds (edcaParameters.GetBeTxopLimit ()));
          SetEdcaParameters (AC_BK, edcaParameters.GetBkCWmin (), edcaParameters.GetBkCWmax (), edcaParameters.GetBkAifsn (), 32 * MicroSeconds (edcaParameters.GetBkTxopLimit ()));
          SetEdcaParameters (AC_VI, edcaParameters.GetViCWmin (), edcaParameters.GetViCWmax (), edcaParameters.GetViAifsn (), 32 * MicroSeconds (edcaParameters.GetViTxopLimit ()));
          SetEdcaParameters (AC_VO, edcaParameters.GetVoCWmin (), edcaParameters.GetVoCWmax (), edcaParameters.GetVoAifsn (), 32 * MicroSeconds (edcaParameters.GetVoTxopLimit ()));
        }
      m_stationManager->SetQosSupport (apAddr, qosSupported);
    }
  if (GetHtSupported ())
    {
      HtCapabilities htCapabilities = assocResp.GetHtCapabilities ();
      if (!htCapabilities.IsSupportedMcs (0))
        {
          m_stationManager->RemoveAllSupportedMcs (apAddr);
        }
      else
        {
          m_stationManager->AddStationHtCapabilities (apAddr, htCapabilities);
        }
    }
  if (GetVhtSupported ())
    {
      VhtCapabilities vhtCapabilities = assocResp.GetVhtCapabilities ();
      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
        {
          m_stationManager->AddStationVhtCapabilities (apAddr, vhtCapabilities);
          VhtOperation vhtOperation = assocResp.GetVhtOperation ();
        }
    }
  if (GetHeSupported ())
    {
      HeCapabilities hecapabilities = assocResp.GetHeCapabilities ();
      if (hecapabilities.GetSupportedMcsAndNss () != 0)
        {
          m_stationManager->AddStationHeCapabilities (apAddr, hecapabilities);
          HeOperation heOperation = assocResp.GetHeOperation ();
          GetHeConfiguration ()->SetAttribute ("BssColor", UintegerValue (heOperation.GetBssColor ()));
        }

      MuEdcaParameterSet muEdcaParameters = assocResp.GetMuEdcaParameterSet ();
      if (muEdcaParameters.IsPresent ())
        {
          SetMuEdcaParameters (AC_BE, muEdcaParameters.GetMuCwMin (AC_BE), muEdcaParameters.GetMuCwMax (AC_BE),
                               muEdcaParameters.GetMuAifsn (AC_BE), muEdcaParameters.GetMuEdcaTimer (AC_BE));
          SetMuEdcaParameters (AC_BK, muEdcaParameters.GetMuCwMin (AC_BK), muEdcaParameters.GetMuCwMax (AC_BK),
                               muEdcaParameters.GetMuAifsn (AC_BK), muEdcaParameters.GetMuEdcaTimer (AC_BK));
          SetMuEdcaParameters (AC_VI, muEdcaParameters.GetMuCwMin (AC_VI), muEdcaParameters.GetMuCwMax (AC_VI),
                               muEdcaParameters.GetMuAifsn (AC_VI), muEdcaParameters.GetMuEdcaTimer (AC_VI));
          SetMuEdcaParameters (AC_VO, muEdcaParameters.GetMuCwMin (AC_VO), muEdcaParameters.GetMuCwMax (AC_VO),
                               muEdcaParameters.GetMuAifsn (AC_VO), muEdcaParameters.GetMuEdcaTimer (AC_VO));
        }
    }
  for (const auto & mode : m_phy->GetModeList ())
    {
      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
        {
          m_stationManager->AddSupportedMode (apAddr, mode);
          if (rates.IsBasicRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
            {
              m_stationManager->AddBasicMode (mode);
            }
        }
    }
  if (GetHtSupported ())
    {
      HtCapabilities htCapabilities = assocResp.GetHtCapabilities ();
      for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HT))
        {
          if (htCapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
            {
              m_stationManager->AddSupportedMcs (apAddr, mcs);
              //here should add a control to add basic MCS when it is implemented
            }
        }
    }
  if (GetVhtSupported ())
    {
      VhtCapabilities vhtcapabilities = assocResp.GetVhtCapabilities ();
      for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_VHT))
        {
          if (vhtcapabilities.IsSupportedRxMcs (mcs.GetMcsValue ()))
            {
              m_stationManager->AddSupportedMcs (apAddr, mcs);
              //here should add a control to add basic MCS when it is implemented
            }
        }
    }
  if (GetHtSupported ())
    {
      ExtendedCapabilities extendedCapabilities = assocResp.GetExtendedCapabilities ();
      //TODO: to be completed
    }
  if (GetHeSupported ())
    {
      HeCapabilities heCapabilities = assocResp.GetHeCapabilities ();
      for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HE))
        {
          if (heCapabilities.IsSupportedRxMcs (mcs.GetMcsValue ()))
            {
              m_stationManager->AddSupportedMcs (apAddr, mcs);
              //here should add a control to add basic MCS when it is implemented
            }
        }
    }
}

SupportedRates
StaWifiMac::GetSupportedRates (void) const
{
  SupportedRates rates;
  for (const auto & mode : m_phy->GetModeList ())
    {
      uint64_t modeDataRate = mode.GetDataRate (m_phy->GetChannelWidth ());
      NS_LOG_DEBUG ("Adding supported rate of " << modeDataRate);
      rates.AddSupportedRate (modeDataRate);
    }
  if (GetHtSupported ())
    {
      for (const auto & selector : m_phy->GetBssMembershipSelectorList ())
        {
          rates.AddBssMembershipSelectorRate (selector);
        }
    }
  return rates;
}

CapabilityInformation
StaWifiMac::GetCapabilities (void) const
{
  CapabilityInformation capabilities;
  capabilities.SetShortPreamble (m_phy->GetShortPhyPreambleSupported () || GetErpSupported ());
  capabilities.SetShortSlotTime (GetShortSlotTimeSupported () && GetErpSupported ());
  return capabilities;
}

void
StaWifiMac::SetState (MacState value)
{
  if (value == ASSOCIATED
      && m_state != ASSOCIATED)
    {
      m_assocLogger (GetBssid ());
    }
  else if (value != ASSOCIATED
           && m_state == ASSOCIATED)
    {
      m_deAssocLogger (GetBssid ());
    }
  m_state = value;
}


void 
StaWifiMac::SetPhyToSleep ()
{
  NS_LOG_FUNCTION (this);
  bool isTxTimerRunning = GetFrameExchangeManager()->GetWifiTxTimer ().IsRunning();
  // If STA is currently receiving frames, do not sleep - sleep after NormalOrBlockAckTransmitted()
  NS_LOG_DEBUG ("PHY m_phy->IsStateRx() is: "<<m_phy->IsStateRx());  
  if (m_phy->IsStateRx())
  {
    NS_LOG_DEBUG ("STA is currently receiving frames. PHY will go to sleep after NormalOrBlockAckTransmitted. TODO - handle cases that do not need ACK");
    m_sleepAfterAck = true;
    return;
  }
  // if an Ack tx is pending, do not sleep - sleep after NormalOrBlockAckTransmitted()
  if (GetFrameExchangeManager()->IsAckTxPending())
  {
    NS_LOG_DEBUG ("STA has an Ack tx pending. PHY will go to sleep after NormalOrBlockAckTransmitted. TODO - handle cases that do not need ACK");
    m_sleepAfterAck = true;
    return;
  }
  // m_phy->IsStateRx()
  // if (m_phy->IsStateRx () || m_phy->IsStateSwitchingToRx () || m_phy->IsStateSwitchingToTx () || m_phy->IsStateTx ())
  // {
  //   NS_LOG_DEBUG ("STA is currently receiving frames. Not sleeping now.");
  //   return;
  // }
  if (isTxTimerRunning)
  {
    NS_LOG_DEBUG ("isTxTimerRunning is true. Setting sleepAfterAck = 1 and not sleeping now.");
    m_sleepAfterAck = true;  
  }
  else
  {
    NS_LOG_DEBUG("isTxTimerRunning is false. Setting m_sleepAfterAck =0. Setting PHY to sleep");
    m_sleepAfterAck = false;
    m_phy->SetSleepMode();
    m_uApsdWaitingToSleepAfterAck = false;    // In case this is an APSD STA
  }

}
void 
StaWifiMac::ResumePhyFromSleep ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG("Resuming PHY from sleep if sleeping");
  m_phy->ResumeFromSleep();
}

void
StaWifiMac::EndOfTwtSpRoutine ()
{
  NS_LOG_FUNCTION (this);
  StaWifiMac::SetPhyToSleep ();
  NS_ASSERT (m_twtAgreement);
  m_twtSpNow = false;  

  m_awakeForTwtSp = false;
  NS_LOG_FUNCTION ("m_awakeForTwtSp is now: "<<m_awakeForTwtSp);

  // TODO Have to change this later
  if (m_twtAnnounced)
  {
    // For announced TWT, go back to default m_wakeUpForNextTwt 0 - do not wake up for next TWT unless required
    NS_LOG_FUNCTION ("m_wakeUpForNextTwt is : "<<m_wakeUpForNextTwt);
    m_wakeUpForNextTwt = false;
    NS_LOG_FUNCTION ("m_wakeUpForNextTwt changed to : "<<m_wakeUpForNextTwt);
  }

  // if (m_twtTriggerBased && m_twtUplinkOutsideSp && (m_twtPsBuffer->GetNPackets () > 0))
  // {
  //   NS_LOG_DEBUG ("For trigger based TWT, At end of TWT SP there is >0 buffered frame and m_twtUplinkOutsideSp is true, transferring one frame from TWT buffer to MAC buffer");
  //   NS_LOG_DEBUG ("m_twtPsBuffer->GetNPackets () = "<<m_twtPsBuffer->GetNPackets ());
  //   Ptr<WifiMacQueueItem> TwtQueueItem;
  //   NS_LOG_DEBUG ("Sending 1 Packet to AP");
  //   TwtQueueItem = m_twtPsBuffer->Dequeue ();
  //   Ptr<Packet> packet = TwtQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet
  //   Enqueue (packet, TwtQueueItem->GetHeader().GetAddr1());
  //   if (m_twtAnnounced)
  //   {
  //     // If it is announced TWT, wake up for next SP
  //     m_wakeUpForNextTwt = true;
  //   }
  // }

}

void
StaWifiMac::SleepWithinTwtSpNow ()
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_twtAgreement);
  
  StaWifiMac::SetPhyToSleep ();
  
  m_awakeForTwtSp = false;
  NS_LOG_FUNCTION ("m_awakeForTwtSp is now: "<<m_awakeForTwtSp);
  

}

void 
StaWifiMac::SetSleepAfterAck ()
{
  NS_LOG_FUNCTION (this);
  m_sleepAfterAck = true;
}

void 
StaWifiMac::ReceivedUnicastNotForMe (void)
{
  // If this is a HE TWT STA with an active TWT agreement, and it is outside its SP, then set Phy to sleep
  NS_LOG_FUNCTION (this);
  if (m_twtAgreement && !m_twtSpNow)
  {
    NS_LOG_DEBUG ("ReceivedUnicastNotForMe called for a TWT STA outside its SP. Setting PHY to sleep");
    SetPhyToSleep ();
  }
}

void
StaWifiMac::NormalOrBlockAckTransmitted ()
{
  NS_LOG_FUNCTION (this);
  bool isTxTimerRunning = GetFrameExchangeManager()->GetWifiTxTimer ().IsRunning();
  if (isTxTimerRunning)
  {
    NS_LOG_DEBUG ("isTxTimerRunning is true. Setting sleepAfterAck = 1 and not sleeping now.");
    m_sleepAfterAck = true;  
    return;
  }
  if (m_sleepAfterAck)
  {
    NS_LOG_DEBUG ("NormalOrBlockAckTransmitted called; m_sleepAfterAck is true. Setting PHY to sleep");
    m_sleepAfterAck = false;
    SetPhyToSleep();
    
  }
  if (m_PSM && m_uApsd && m_uApsdWaitingToSleepAfterAck && IsEveryEdcaQueueEmpty() && !isTxTimerRunning)
  {
    NS_LOG_DEBUG ("STA uses APSD, Received EOSP frame, No queued frames, and Txtimer not running. Setting to sleep.");
    m_uApsdWaitingToSleepAfterAck = false;
    SetPhyToSleep();
  }
}

void 
StaWifiMac::SleepAfterAckIfRequired ()
{
  NS_LOG_FUNCTION (this);
  // NS_LOG_DEBUG ("m_sleepAfterAck is checked only for TWT. For PSM, STA is put to sleep if MAC queue is empty.");

  if (m_waitingToSwitchToPSM)
  {
    m_PSM = true;
    m_waitingToSwitchToPSM = false;
  }

  if (m_waitingToSwitchOutOfPSM)
  {
    m_PSM = false;
    m_waitingToSwitchOutOfPSM = false;
  }

  if (m_uApsd)
  {
    NS_LOG_DEBUG ("m_uApsd is true for this STA and SleepAfterAckIfRequired was called. Not Sleeping");
    m_sleepAfterAck = false;
    return;
  }

  if (m_PSM)
  {
    NS_LOG_DEBUG ("SleepAfterAckIfRequired () called for an STA in PSM");
    NS_LOG_DEBUG ("Checking if MAC queue is empty");
    bool isMacQueueEmpty = StaWifiMac::IsEveryEdcaQueueEmpty () && StaWifiMac::IsDcfQueueEmpty ();
    NS_LOG_DEBUG ("isMacQueueEmpty: "<<isMacQueueEmpty);
    NS_LOG_DEBUG ("m_PsNeedToBeAwake: "<<m_PsNeedToBeAwake);
    NS_LOG_DEBUG ("m_PsWaitingForPolledFrame: "<<m_PsWaitingForPolledFrame);
    NS_LOG_DEBUG ("m_pendingPsPoll: "<<m_pendingPsPoll);
    NS_LOG_DEBUG ("m_beaconWakeUpPsEvent.IsRunning() = "<<m_beaconWakeUpPsEvent.IsRunning());

    // uplink only change below - beacons are ignored for wake ups
    // if (isMacQueueEmpty && m_beaconWakeUpPsEvent.IsRunning() && !m_PsNeedToBeAwake && !m_PsWaitingForPolledFrame && !m_pendingPsPoll)
    // shyam #fixAsap - #hardcoded - above line makes sure STA wakes up for beacons - below line ignores beacon wake ups
    if (isMacQueueEmpty  && !m_PsNeedToBeAwake && !m_PsWaitingForPolledFrame && !m_pendingPsPoll)
    {
      NS_LOG_DEBUG ("MAC queue is empty and there is an event scheduled to wake up for next beacon. STA is put to SLEEP regardless of m_sleepAfterAck");
      SetPhyToSleep();
      m_sleepAfterAck = false;
      return;
    }
    else
    {
      NS_LOG_DEBUG ("PSM STA not set to SLEEP");
      return;
    }

  }
  if (m_twtAgreement)
  {
    NS_LOG_DEBUG ("SleepAfterAckIfRequired () called for an STA in TWT");
    if (!m_twtSpNow)
    {
      StaWifiMac::SetPhyToSleep ();
      m_awakeForTwtSp = false;
      return;
    }
    
    // bool isMacQueueEmpty = StaWifiMac::IsEveryEdcaQueueEmpty () && StaWifiMac::IsDcfQueueEmpty ();


    // if (m_sleepAfterAck && isMacQueueEmpty)
    // {
    //   NS_LOG_DEBUG ("m_sleepAfterAck is true and MAC queue is empty. Setting STA to SLEEP");
    //   // SetPhyToSleep();
    //   SleepWithinTwtSpNow ();
    // }
    // else if (m_sleepAfterAck && !isMacQueueEmpty)
    // {
    //   NS_LOG_DEBUG ("m_sleepAfterAck is true but MAC queue is NOT empty. Keeping m_sleepAfterAck as true and not setting PHY to sleep.");
    //   return; 
    // }

    m_sleepAfterAck = false;
  }
  return;
}

void
StaWifiMac::SendOneTwtUlPpduNow ()
{
  NS_LOG_FUNCTION (this);
  // bool isMacQueueEmpty = StaWifiMac::IsEveryEdcaQueueEmpty () && StaWifiMac::IsDcfQueueEmpty ();
  
  // NS_ASSERT_MSG (!isMacQueueEmpty, "MAC queue is not empty. Check TWT TB STA implementation. Packets should be in TWT buffer");
  if (m_twtAgreement && m_twtPsBuffer->GetNPackets () == 0)
  {
    NS_LOG_DEBUG ("Received TF from AP. TWT Ps queue is empty. Sending Null frame with PM = 1.");
    // WifiTxVector m_NullTxVector;
    // Ptr<WifiPsdu> m_nullFrame;    

    Ptr<Packet> pkt = Create<Packet> ();
    WifiMacHeader hdr;
    hdr.SetType (WIFI_MAC_QOSDATA_NULL);
    // hdr.SetType (WIFI_MAC_QOSDATA);
    // hdr.SetType (WIFI_MAC_DATA_NULL);
    hdr.SetAddr1 (GetBssid ());
    hdr.SetAddr2 (GetAddress ());
    hdr.SetAddr3 (GetBssid ());
    hdr.SetDsTo ();
    hdr.SetDsNotFrom ();
    hdr.SetPowerMgt ();
    // TR3: Sequence numbers for transmitted QoS (+)Null frames may be set
    // to any value. (Table 10-3 of 802.11-2016)
    hdr.SetSequenceNumber (0);
    // Set the EOSP bit so that NotifyTxToEdca will add the Queue Size
    hdr.SetQosEosp ();
    hdr.SetQosTid (0);

    // m_nullFrame = Create<WifiMacQueueItem> (pkt, hdr);
    if (!GetQosSupported ())
    {
      m_txop->Queue (pkt, hdr);
      // m_feManager->StartTransmission (m_txop);
    }
    else
    {
      GetBEQueue ()->Queue (pkt, hdr);
      // m_feManager->StartTransmission (GetBEQueue ());
    }

    // NS_LOG_DEBUG ("Setting m_sleepAfterAck to true");
    // m_sleepAfterAck = true;
    
    return;
  }
  if (m_twtPsBuffer->GetNPackets () == 0)
  {
    // No UL packets in buffer and TWT is suspendable
    NS_LOG_DEBUG ("Received TF from AP. TWT Ps queue is empty. Sending TWT Info frame to suspend TWT");
    EnqueueTwtInfoFrameToApNow (1, 0, 0, 0, 0);
    if (!GetQosSupported ())
    { 
      m_feManager->StartTransmission (m_txop);
    }
    else
    {
      m_feManager->StartTransmission (GetBEQueue ());
    }

    NS_LOG_DEBUG ("Setting m_sleepAfterAck to true");
    m_sleepAfterAck = true;
    return;
  }

}

void 
StaWifiMac::EnqueueTwtInfoFrameToApNow (uint8_t twtFlowId, uint8_t responseRequested, uint8_t nextTwtRequest, uint8_t nextTwtSubfieldSize, uint8_t allTwt, uint64_t nextTwt)
{
  NS_LOG_FUNCTION(this<<twtFlowId<<responseRequested<<nextTwtRequest<<nextTwtSubfieldSize<<allTwt<<nextTwt);
  NS_LOG_DEBUG ("Enqueing TWT Info Frame to " << GetBssid ()); 

  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_ACTION);
  hdr.SetAddr1 (GetBssid ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetBssid ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.unprotectedS1GAction = WifiActionHeader::TWT_INFORMATION;
  actionHdr.SetAction (WifiActionHeader::UNPROTECTED_S1G, action);

  Ptr<Packet> packet = Create<Packet> ();
  MgtUnprotectedTwtInfoHeader twtInfoHdr (twtFlowId, responseRequested, nextTwtRequest, nextTwtSubfieldSize, allTwt, nextTwt);
  packet->AddHeader (twtInfoHdr);
  packet->AddHeader (actionHdr);

  // Ptr<WifiMacQueueItem> mpdu = Create<WifiMacQueueItem> (packet, hdr);
  if (GetQosSupported ())
    {
      m_edca[QosUtilsMapTidToAc (0)]->Queue (packet, hdr);
    }
  else
    {
      NS_LOG_DEBUG ("Enqueuing packet in Non-QoS queue");
      m_txop->Queue (packet, hdr);
    }
}


void StaWifiMac::ResumeSuspendTwtIfRequired (u_int8_t twtId, uint64_t nextTwt)
{
  NS_LOG_FUNCTION(this<<twtId<<nextTwt);
  if (m_twtAgreement)
  {
    NS_LOG_DEBUG ("m_twtActive is now :"<<m_twtActive);
    NS_LOG_DEBUG ("TWT Info Frame ACk'ed for nextTwt = "<<nextTwt);
    
    if (nextTwt == 0)
    {
      m_twtActive = false;
      m_wakeUpForNextTwt = false;
      SleepWithinTwtSpNow ();
    }
    else
    {
      m_twtActive = true;
      if (!m_twtAnnounced)
      {
        m_wakeUpForNextTwt = true;
      }
      
    }
  }
  NS_LOG_DEBUG ("m_twtActive: "<< m_twtActive<<";m_wakeUpForNextTwt : "<<m_wakeUpForNextTwt);
}

void
StaWifiMac::SetEdcaParameters (AcIndex ac, uint32_t cwMin, uint32_t cwMax, uint8_t aifsn, Time txopLimit)
{
  Ptr<QosTxop> edca = GetQosTxop (ac);
  edca->SetMinCw (cwMin);
  edca->SetMaxCw (cwMax);
  edca->SetAifsn (aifsn);
  edca->SetTxopLimit (txopLimit);
}

void
StaWifiMac::SetMuEdcaParameters (AcIndex ac, uint16_t cwMin, uint16_t cwMax, uint8_t aifsn, Time muEdcaTimer)
{
  Ptr<QosTxop> edca = GetQosTxop (ac);
  edca->SetMuCwMin (cwMin);
  edca->SetMuCwMax (cwMax);
  edca->SetMuAifsn (aifsn);
  edca->SetMuEdcaTimer (muEdcaTimer);
}

void
StaWifiMac::PhyCapabilitiesChanged (void)
{
  NS_LOG_FUNCTION (this);
  if (IsAssociated ())
    {
      NS_LOG_DEBUG ("PHY capabilities changed: send reassociation request");
      SetState (WAIT_ASSOC_RESP);
      SendAssociationRequest (true);
    }
}

} //namespace ns3
