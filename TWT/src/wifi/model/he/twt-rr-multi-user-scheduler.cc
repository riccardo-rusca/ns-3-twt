/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "twt-rr-multi-user-scheduler.h"
#include "ns3/wifi-protection.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/wifi-psdu.h"
#include "he-frame-exchange-manager.h"
#include "he-configuration.h"
#include "he-phy.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TwtRrMultiUserScheduler");

NS_OBJECT_ENSURE_REGISTERED (TwtRrMultiUserScheduler);

TypeId
TwtRrMultiUserScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TwtRrMultiUserScheduler")
    .SetParent<MultiUserScheduler> ()
    .SetGroupName ("Wifi")
    .AddConstructor<TwtRrMultiUserScheduler> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in a DL MU OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&TwtRrMultiUserScheduler::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("EnableTxopSharing",
                   "If enabled, allow A-MPDUs of different TIDs in a DL MU PPDU.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&TwtRrMultiUserScheduler::m_enableTxopSharing),
                   MakeBooleanChecker ())
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_MU_TX even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TwtRrMultiUserScheduler::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_MU_TX if DL_MU_TX was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&TwtRrMultiUserScheduler::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableBsrp",
                   "If enabled, send a BSRP Trigger Frame before an UL MU transmission.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&TwtRrMultiUserScheduler::m_enableBsrp),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The default size in bytes of the solicited PSDU (to be sent in a TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&TwtRrMultiUserScheduler::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("UseCentral26TonesRus",
                   "If enabled, central 26-tone RUs are allocated, too, when the "
                   "selected RU type is at least 52 tones.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TwtRrMultiUserScheduler::m_useCentral26TonesRus),
                   MakeBooleanChecker ())
    .AddAttribute ("MaxCredits",
                   "Maximum amount of credits a station can have. When transmitting a DL MU PPDU, "
                   "the amount of credits received by each station equals the TX duration (in "
                   "microseconds) divided by the total number of stations. Stations that are the "
                   "recipient of the DL MU PPDU have to pay a number of credits equal to the TX "
                   "duration (in microseconds) times the allocated bandwidth share",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&TwtRrMultiUserScheduler::m_maxCredits),
                   MakeTimeChecker ())
  ;
  return tid;
}

TwtRrMultiUserScheduler::TwtRrMultiUserScheduler ()
{
  NS_LOG_FUNCTION (this);
}

TwtRrMultiUserScheduler::~TwtRrMultiUserScheduler ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
TwtRrMultiUserScheduler::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_apMac != nullptr);
  m_apMac->TraceConnectWithoutContext ("AssociatedSta",
                                       MakeCallback (&TwtRrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceConnectWithoutContext ("DeAssociatedSta",
                                       MakeCallback (&TwtRrMultiUserScheduler::NotifyStationDeassociated, this));
  for (const auto& ac : wifiAcList)
    {
      m_staList.insert ({ac.first, {}});
    }
  MultiUserScheduler::DoInitialize ();
}

void
TwtRrMultiUserScheduler::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
  m_candidates.clear ();
  m_txParams.Clear ();
  m_apMac->TraceDisconnectWithoutContext ("AssociatedSta",
                                          MakeCallback (&TwtRrMultiUserScheduler::NotifyStationAssociated, this));
  m_apMac->TraceDisconnectWithoutContext ("DeAssociatedSta",
                                          MakeCallback (&TwtRrMultiUserScheduler::NotifyStationDeassociated, this));
  MultiUserScheduler::DoDispose ();
}

MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::SelectTxFormat (void)
{
  NS_LOG_FUNCTION (this);
  uint16_t nStaExpectingTf = GetWifiRemoteStationManager ()->GetNStaExpectingTf ();
  NS_LOG_DEBUG ("GetNStaExpectingTf = "<< nStaExpectingTf);
  
  bool wasLastTriggerBsrp = (m_trigger.GetType () == TriggerFrameType::BSRP_TRIGGER);
  NS_LOG_DEBUG ("wasLastTriggerBsrp = "<<wasLastTriggerBsrp);

  // To send QoS EOSP frames if all BSRs are 0 - error since this frame never leaves the Mac Queue even after ACK
  // if (m_lastTxFormat == UL_MU_TX && wasLastTriggerBsrp && !m_candidates.empty())
  // {
  //   NS_LOG_DEBUG("Last sequence was BSRP UL and m_candidates is not empty. If ALL OF m_candidates have no buffered data, send QoS frame with EOSP set");
  //   bool sendNullEospForAll = true;
  //   for (const auto& candidate : m_candidates)
  //   {
  //     Mac48Address staMacAddress = candidate.first->address;

  //     uint8_t queueSize = m_apMac->GetMaxBufferStatus (staMacAddress);
  //     if (queueSize != 0)
  //     {
  //       NS_LOG_DEBUG ("staMacAddress: "<<staMacAddress<<" has queue size !=0. Not sending EOSP to any STA");
  //       sendNullEospForAll = false;
  //       break;
  //     }
  //   }
  //   if (sendNullEospForAll)
  //   {
  //     NS_LOG_DEBUG ("Queueing Null Eosp frames for all of m_candidates");
  //     // Mac48Address apMacAddress = GetWifiRemoteStationManager() -> GetMac () -> GetAddress ();
  //     for (const auto& candidate : m_candidates)
  //     {
  //       Mac48Address staMacAddress = candidate.first->address;
  //       NS_LOG_DEBUG ("Queueing Null QoS EOSP frame for this STA "<<staMacAddress<<" from AP");
  //       m_apMac->SendQoSEospNullByAddress(staMacAddress);
  //     }
  //   }
  // }
  // If MPDUs are queued, always send them down first
  Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();
  // if (mpdu != 0 && GetWifiRemoteStationManager ()->GetTwtAgreement (mpdu->GetHeader ().GetAddr1 ()))
  if (mpdu != 0)
  {
    NS_LOG_DEBUG ("MPDUs queued. Trying DL_MU_TX");
    return TrySendingDlMuPpdu ();
  }

  if (m_lastTxFormat == DL_MU_TX)
  {
    NS_LOG_DEBUG ("Last format was DL_MU_TX - clearing m_candidates; Else it confuses sending extra Basic TFs");
    m_candidates.clear();
  }

  // Checking if next Tx should be BSRP
  
  
  if (mpdu == 0 && (nStaExpectingTf>0) && m_enableUlOfdma && m_enableBsrp)
  {
    bool needBsrp = false;
    bool atLeastOneStaAwake = false;

      // NS_LOG_DEBUG ("m_candidates is not empty. No DL packets queued. If at least one expecting STA has unknown or >0 BSR, send BSRP if there is no pending Basic TF");
    NS_LOG_DEBUG ("No DL packets queued. If at least one expecting and awake STA has unknown BSR, send BSRP to those STAs only");
    AcIndex primaryAc = m_edca->GetAccessCategory ();
    auto staIt = m_staList[primaryAc].begin ();
    while (staIt != m_staList[primaryAc].end ())
    {
      Mac48Address staMacAddress = staIt->address ;
      if (GetWifiRemoteStationManager() -> GetExpectingBasicTf (staMacAddress) && GetWifiRemoteStationManager() -> GetTwtAwakeForSp (staMacAddress))
      // if (GetWifiRemoteStationManager() -> GetExpectingBasicTf (staMacAddress))
      {
        NS_LOG_DEBUG ("This STA "<<staMacAddress<<" is awake and is expecting TF");
        atLeastOneStaAwake = true;
        uint8_t queueSize = m_apMac->GetMaxBufferStatus (staMacAddress);
        NS_LOG_DEBUG ("Queue size of STA "<<staMacAddress<<" is "<<+queueSize);
        
        if (queueSize == 255)
        {
          NS_LOG_DEBUG ("STA "<<staMacAddress<<" has unknown BSR. BSRP is required");
          needBsrp = true;
          break;
        }

      }
      staIt++;
    }
    if (needBsrp)
    {
      AcIndex primaryAc = m_edca->GetAccessCategory ();
      NS_LOG_DEBUG ("Gained Txop for AC = "<<primaryAc*1 );
   
      TxFormat txTemp = ForceCreateDlMuInfo ();   // NOTE: considers only awake TWT STAs for which BSR = 255
      NS_ASSERT ((txTemp == DL_MU_TX) || (txTemp == NO_TX));   
      if (txTemp == NO_TX)
      {
        return txTemp;
      }
      else
      {
        NS_LOG_DEBUG ("m_dlInfo computed succesfully");
        m_lastTxFormat = DL_MU_TX;
        return TrySendingBsrpTf ();
      }
    }
    // BSRP not needed - send BasicTf if at least one TWT STA is awake if necessary
    if (atLeastOneStaAwake)
    {
      TxFormat txFormat = TrySendingBasicTf ();

      if (txFormat != DL_MU_TX)
      {
        return txFormat;
      }
    }
    
  }


  // Logic for sending BSRP
  // If no mpdu is queued, and GetNStaExpectingTf > 0, last trigger was not BSRP
  

  // If last frame was Basic TF and at least one of the STAs has more data, send basic TF again
  // bool sendBasicTfAgain = false;
  // if (m_enableUlOfdma && m_trigger.GetType () == TriggerFrameType::BASIC_TRIGGER && !m_candidates.empty() && GetLastTxFormat () == UL_MU_TX)
  // {
  //   NS_LOG_DEBUG ("Last Trigger frame was BASIC_TRIGGER; Last format was UL_MU_TX. Checking max buffer of same STA candidate list.");
  //   uint32_t maxBufferSize = 0;
  //   bool atLeastOneStaAwake = false;    // If at least of of the STAs with buffered data at STA is awake, set to true
  //   for (const auto& candidate : m_candidates)
  //     {
  //       Mac48Address staMacAddress = candidate.first->address;

  //       uint8_t queueSize = m_apMac->GetMaxBufferStatus (staMacAddress);
  //       if (queueSize == 255)
  //         {
  //           NS_LOG_DEBUG ("Buffer status of station " << staMacAddress << " is unknown");
            
  //         }
  //       else if (queueSize == 254)
  //         {
  //           NS_LOG_DEBUG ("Buffer status of station " << staMacAddress << " is not limited");
  //           maxBufferSize = 0xffffffff;
  //           if (GetWifiRemoteStationManager()->GetTwtAwakeForSp(staMacAddress))
  //           {
  //             atLeastOneStaAwake = true;
  //           }
  //         }
  //       else
  //         {
  //           NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is " << +queueSize);
  //           maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
  //           if (GetWifiRemoteStationManager()->GetTwtAwakeForSp(staMacAddress))
  //           {
  //             atLeastOneStaAwake = true;
  //           }
  //         }
  //     }
  //   if (maxBufferSize>0 && atLeastOneStaAwake)
  //   {
  //     sendBasicTfAgain = true;
  //     NS_LOG_DEBUG ("Sending Basic TF again because at least one of the AWAKE TWT STAs has more data buffered");
  //     TxFormat txFormat = TrySendingBasicTf ();

  //     if (txFormat != DL_MU_TX)
  //       {
  //         return txFormat;
  //       }
  //   }
  //   NS_LOG_DEBUG ("sendBasicTfAgain is now "<<sendBasicTfAgain);

  // }


  // if (mpdu == 0 && (nStaExpectingTf>0) && !(wasLastTriggerBsrp) && m_enableUlOfdma && m_enableBsrp)
  // To send BSRP
  // if (!sendBasicTfAgain && needBsrp)
  // {
  //   NS_LOG_DEBUG ("no mpdu is queued at AP, and GetNStaExpectingTf > 0 --> BSRP should be sent now");
    
    
  //   // uint8_t nStasAdded = 0;
  //   // Mac48Address apMacAddress = GetWifiRemoteStationManager() -> GetMac () -> GetAddress ();
  //   // auto candidateIt = m_candidates.begin ();
  //   AcIndex primaryAc = m_edca->GetAccessCategory ();
  //   NS_LOG_DEBUG ("Gained Txop for AC = "<<primaryAc*1 );
   
  //   TxFormat txTemp = ForceCreateDlMuInfo ();   // NOTE: considers only awake TWT STAs for which BSR = 255
  //   NS_ASSERT ((txTemp == DL_MU_TX) || (txTemp == NO_TX));   
  //   if (txTemp == NO_TX)
  //   {
  //     return txTemp;
  //   }
  //   else
  //   {
  //     NS_LOG_DEBUG ("m_dlInfo computed succesfully");
  //     m_lastTxFormat = DL_MU_TX;
  //     return TrySendingBsrpTf ();
  //   }
    
    
  // }

  // if (m_enableUlOfdma && (GetLastTxFormat () == DL_MU_TX
  //                         || m_trigger.GetType () == TriggerFrameType::BSRP_TRIGGER))
  //   {
  //     NS_LOG_DEBUG ("GetLastTxFormat () == DL_MU_TX ==>"<<(GetLastTxFormat () == DL_MU_TX));
  //     NS_LOG_DEBUG ("m_trigger.GetType ()==>"<<m_trigger.GetType ());

  //     TxFormat txFormat = TrySendingBasicTf ();

  //     if (txFormat != DL_MU_TX)
  //       {
  //         return txFormat;
  //       }
  //   }

    
  if (mpdu != 0 && !GetWifiRemoteStationManager ()->GetHeSupported (mpdu->GetHeader ().GetAddr1 ()))
    {
      
      return SU_TX;
    }

  // if (m_enableUlOfdma && m_enableBsrp && GetLastTxFormat () == DL_MU_TX)
  //   {
  //     NS_LOG_DEBUG ("GetLastTxFormat () == DL_MU_TX ==>"<<(GetLastTxFormat () == DL_MU_TX));
  //     NS_LOG_DEBUG ("m_enableUlOfdma==>"<<m_enableUlOfdma);
  //     NS_LOG_DEBUG ("m_enableBsrp==>"<<m_enableBsrp);
  //     return TrySendingBsrpTf ();
  //   }


  // if (TrySendingDlMuPpdu () == TxFormat::DL_MU_TX && GetLastTxFormat () != DL_MU_TX)
  // { 
  //   return TrySendingDlMuPpdu ();
  // }


  return TrySendingDlMuPpdu ();
}

MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::TrySendingBsrpTf (void)
{
  NS_LOG_FUNCTION (this);
  
  m_trigger = CtrlTriggerHeader (TriggerFrameType::BSRP_TRIGGER, GetDlMuInfo ().txParams.m_txVector);

  WifiTxVector txVector = GetDlMuInfo ().txParams.m_txVector;
  txVector.SetGuardInterval (m_trigger.GetGuardInterval ());

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (m_trigger);

  Mac48Address receiver = Mac48Address::GetBroadcast ();
  if (m_trigger.GetNUserInfoFields () == 1)
    {
      NS_ASSERT (m_apMac->GetStaList ().find (m_trigger.begin ()->GetAid12 ()) != m_apMac->GetStaList ().end ());
      receiver = m_apMac->GetStaList ().at (m_trigger.begin ()->GetAid12 ());
    }

  m_triggerMacHdr = WifiMacHeader (WIFI_MAC_CTL_TRIGGER);
  m_triggerMacHdr.SetAddr1 (receiver);
  m_triggerMacHdr.SetAddr2 (m_apMac->GetAddress ());
  m_triggerMacHdr.SetDsNotTo ();
  m_triggerMacHdr.SetDsNotFrom ();

  Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, m_triggerMacHdr);

  m_txParams.Clear ();
  // set the TXVECTOR used to send the Trigger Frame
  m_txParams.m_txVector = m_apMac->GetWifiRemoteStationManager ()->GetRtsTxVector (receiver);

  if (!m_heFem->TryAddMpdu (item, m_txParams, m_availableTime))
    {
      // sending the BSRP Trigger Frame is not possible, hence return NO_TX. In
      // this way, no transmission will occur now and the next time we will
      // try again sending a BSRP Trigger Frame.
      NS_LOG_DEBUG ("Remaining TXOP duration is not enough for BSRP TF exchange");
      return NO_TX;
    }

  // Compute the time taken by each station to transmit 8 QoS Null frames
  Time qosNullTxDuration = Seconds (0);
  for (const auto& userInfo : m_trigger)
    {
      Time duration = WifiPhy::CalculateTxDuration (m_sizeOf8QosNull, txVector,
                                                    m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                    userInfo.GetAid12 ());
      qosNullTxDuration = Max (qosNullTxDuration, duration);
    }

  if (m_availableTime != Time::Min ())
    {
      // TryAddMpdu only considers the time to transmit the Trigger Frame
      NS_ASSERT (m_txParams.m_protection && m_txParams.m_protection->protectionTime != Time::Min ());
      NS_ASSERT (m_txParams.m_acknowledgment && m_txParams.m_acknowledgment->acknowledgmentTime.IsZero ());
      NS_ASSERT (m_txParams.m_txDuration != Time::Min ());

      if (m_txParams.m_protection->protectionTime
          + m_txParams.m_txDuration     // BSRP TF tx time
          + m_apMac->GetWifiPhy ()->GetSifs ()
          + qosNullTxDuration
          > m_availableTime)
        {
          NS_LOG_DEBUG ("Remaining TXOP duration is not enough for BSRP TF exchange");
          return NO_TX;
        }
    }

  NS_LOG_DEBUG ("Duration of QoS Null frames: " << qosNullTxDuration.As (Time::MS));
  m_trigger.SetUlLength (HePhy::ConvertHeTbPpduDurationToLSigLength (qosNullTxDuration,
                                                                     m_apMac->GetWifiPhy ()->GetPhyBand ()));

  return UL_MU_TX;
}

MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::TrySendingBasicTf (void)
{
  NS_LOG_FUNCTION (this);

  // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
  NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

  AcIndex primaryAc = m_edca->GetAccessCategory ();
  NS_LOG_DEBUG ("Assigning queue sizes for this AC to all STAs with max buffer size. If unknown, 0 is assigned");
    for (auto& sta : m_staList[primaryAc])
    {
      sta.queueSize = 0;
      uint8_t queueSizeKnown = m_apMac->GetMaxBufferStatus (sta.address);
      if (queueSizeKnown < 255)
      {
        sta.queueSize = queueSizeKnown;
      }
      NS_LOG_DEBUG ("Queue Size assigned for STA "<<sta.address<<" is "<<sta.queueSize * 1.0); 
    }

  
  NS_LOG_DEBUG ("Sorting m_staList in descending order of known queue size");
  // sort the list in decreasing order of credits
  m_staList[primaryAc].sort ([] (const MasterInfo& a, const MasterInfo& b)
                              { return a.queueSize > b.queueSize; });

  for (auto& sta : m_staList[primaryAc])
  {
    
    NS_LOG_DEBUG ("After sorting, Queue Size assigned for STA "<<sta.address<<" is "<<sta.queueSize * 1.0); 
  }

  // determine which of the stations served in DL have UL traffic
  uint32_t maxBufferSize = 0;
  // candidates sorted in decreasing order of queue size
  std::multimap<uint8_t, CandidateInfo, std::greater<uint8_t>> ulCandidates;


  // to do: create dlMuInfo with STAs for which queue size is known, create bsrp info for those m_candidates, call computeUlInfo

  // Step 1: Creating DlMuINfo for STAs with known buffer status
  TxFormat txTemp = ForceCreateDlMuInfoKnownStaOnly ();   // NOTE: considers only awake TWT STAs for which is known and non-empty
  TxFormat txTemp2;
  NS_ASSERT ((txTemp == DL_MU_TX) || (txTemp == NO_TX));   
  if (txTemp == NO_TX)
  {
    return txTemp;
  }
  
  // txTemp == DL_MU_TX
  NS_LOG_DEBUG ("m_dlInfo computed successfully");
  m_lastTxFormat = DL_MU_TX;
txTemp2 = TrySendingBsrpTf ();
  

  // txTemp2 should now be NO_TX or UL_MU_TX since TrySendingBsrpTf was called
  NS_ASSERT ((txTemp2 == UL_MU_TX) || (txTemp == NO_TX));  
  if (txTemp2 == NO_TX)
  {
    return txTemp2;
  }
  // Dummy trigger details are now ready for the dummy BSRP
  m_ulInfo = ComputeUlMuInfo ();

  
  // Created new m_candidates list and m_ulInfo based on STAs with known non-zero BSR

  for (const auto& candidate : m_candidates)
    {
      if (!(GetWifiRemoteStationManager()->GetTwtAwakeForSp(candidate.first->address)) && GetWifiRemoteStationManager()->GetTwtAgreement(candidate.first->address))
      {
        NS_LOG_DEBUG ("This STA: "<<candidate.first->address<<" has a TWT agreement and is not awake now. It is not considered for Basic TF.");
        continue;
      }
      uint8_t queueSize = m_apMac->GetMaxBufferStatus (candidate.first->address);
      if (queueSize == 255)
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is unknown");
          maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
        }
      else if (queueSize == 254)
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is not limited");
          maxBufferSize = 0xffffffff;
        }
      else
        {
          NS_LOG_DEBUG ("Buffer status of station " << candidate.first->address << " is " << +queueSize);
          maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
        }
      // serve the station if its queue size is not null
      // if (queueSize > 0)   // original
      if (queueSize > 0)
        {
          NS_LOG_DEBUG ("Candidate: "<<candidate.first->address<<" is added to ulCandidates");
          // ulCandidates.emplace (queueSize, candidate);
          
          NS_LOG_DEBUG ("Inserting Ul Candidate = "<<candidate.first->address <<"; Queue size = "<<queueSize*1.0);
          ulCandidates.insert (std::make_pair(queueSize, candidate));
        }
    }


  // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
  if (maxBufferSize > 0)
    {
      NS_ASSERT (!ulCandidates.empty ());

      NS_LOG_DEBUG ("ulCandidates created and is not empty for Basic TF. Printing as stored:");
      // -------------------------------
      auto candidatePrint = ulCandidates.begin ();
      while (candidatePrint != ulCandidates.end ())
      {
        NS_LOG_DEBUG ("Printing Ul Candidate = "<<candidatePrint->second.first->address<<"; Queue size = "<<(candidatePrint->first * 1.0));
        candidatePrint++;
      }


      // -------------------------------

      std::size_t count = ulCandidates.size ();
      std::size_t nCentral26TonesRus;
      HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (),
                                                               count, nCentral26TonesRus);
      if (!m_useCentral26TonesRus || ulCandidates.size () == count)
        {
          nCentral26TonesRus = 0;
        }
      else
        {
          nCentral26TonesRus = std::min (ulCandidates.size () - count, nCentral26TonesRus);
        }

      WifiTxVector txVector;
      txVector.SetPreambleType (WIFI_PREAMBLE_HE_TB);
      auto candidateIt = ulCandidates.begin ();

      if (GetLastTxFormat () == DL_MU_TX)
        {
          txVector.SetChannelWidth (GetDlMuInfo ().txParams.m_txVector.GetChannelWidth ());
          txVector.SetGuardInterval (CtrlTriggerHeader ().GetGuardInterval ());

          for (std::size_t i = 0; i < count + nCentral26TonesRus; i++)
            {
              NS_ASSERT (candidateIt != ulCandidates.end ());
              uint16_t staId = candidateIt->second.first->aid;
              // AssignRuIndices will be called below to set RuSpec
              txVector.SetHeMuUserInfo (staId,
                                        {{(i < count ? ruType : HeRu::RU_26_TONE), 1, false},
                                        GetDlMuInfo ().txParams.m_txVector.GetMode (staId),
                                        GetDlMuInfo ().txParams.m_txVector.GetNss (staId)});

              candidateIt++;
            }
        }
      else
        {
          txVector.SetChannelWidth (GetUlMuInfo ().trigger.GetUlBandwidth ());
          txVector.SetGuardInterval (GetUlMuInfo ().trigger.GetGuardInterval ());

          for (std::size_t i = 0; i < count + nCentral26TonesRus; i++)
            {
              NS_ASSERT (candidateIt != ulCandidates.end ());
              uint16_t staId = candidateIt->second.first->aid;
              auto userInfoIt = GetUlMuInfo ().trigger.FindUserInfoWithAid (staId);
              NS_ASSERT (userInfoIt != GetUlMuInfo ().trigger.end ());
              // AssignRuIndices will be called below to set RuSpec
              txVector.SetHeMuUserInfo (staId,
                                        {{(i < count ? ruType : HeRu::RU_26_TONE), 1, false},
                                        HePhy::GetHeMcs (userInfoIt->GetUlMcs ()),
                                        userInfoIt->GetNss ()});

              candidateIt++;
            }
        }

      // remove candidates that will not be served
      ulCandidates.erase (candidateIt, ulCandidates.end ());
      AssignRuIndices (txVector);

      m_trigger = CtrlTriggerHeader (TriggerFrameType::BASIC_TRIGGER, txVector);
      Ptr<Packet> packet = Create<Packet> ();
      packet->AddHeader (m_trigger);

      Mac48Address receiver = Mac48Address::GetBroadcast ();
      if (ulCandidates.size () == 1)
        {
          receiver = ulCandidates.begin ()->second.first->address;
        }
      // shyam - SetExpectingBasicTf to 0 for STAs in this list
      // auto candidateItTemp = ulCandidates.begin ();
      // while (candidateItTemp != ulCandidates.end ())
      // {
      //   Mac48Address staMacAddr = candidateItTemp->second.first->address;
      //   NS_LOG_DEBUG ("Setting expectingBasicTf to false for STA: "<<staMacAddr);
      //   m_apMac->GetWifiRemoteStationManager ()->SetExpectingBasicTf (staMacAddr, false);
      //   candidateItTemp++;
      // }

      

      m_triggerMacHdr = WifiMacHeader (WIFI_MAC_CTL_TRIGGER);
      m_triggerMacHdr.SetAddr1 (receiver);
      m_triggerMacHdr.SetAddr2 (m_apMac->GetAddress ());
      m_triggerMacHdr.SetDsNotTo ();
      m_triggerMacHdr.SetDsNotFrom ();

      Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, m_triggerMacHdr);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (txVector.GetPreambleType ());

      m_txParams.Clear ();
      // set the TXVECTOR used to send the Trigger Frame
      m_txParams.m_txVector = m_apMac->GetWifiRemoteStationManager ()->GetRtsTxVector (receiver);

      if (!m_heFem->TryAddMpdu (item, m_txParams, m_availableTime))
        {
          // an UL OFDMA transmission is not possible, hence return NO_TX. In
          // this way, no transmission will occur now and the next time we will
          // try again performing an UL OFDMA transmission.
          NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
          return NO_TX;
        }

      if (m_availableTime != Time::Min ())
        {
          // TryAddMpdu only considers the time to transmit the Trigger Frame
          NS_ASSERT (m_txParams.m_protection && m_txParams.m_protection->protectionTime != Time::Min ());
          NS_ASSERT (m_txParams.m_acknowledgment && m_txParams.m_acknowledgment->acknowledgmentTime != Time::Min ());
          NS_ASSERT (m_txParams.m_txDuration != Time::Min ());

          maxDuration = Min (maxDuration, m_availableTime
                                          - m_txParams.m_protection->protectionTime
                                          - m_txParams.m_txDuration
                                          - m_apMac->GetWifiPhy ()->GetSifs ()
                                          - m_txParams.m_acknowledgment->acknowledgmentTime);
          if (maxDuration.IsNegative ())
            {
              NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
              return NO_TX;
            }
        }

      // Compute the time taken by each station to transmit a frame of maxBufferSize size
      Time bufferTxTime = Seconds (0);
      for (const auto& userInfo : m_trigger)
        {
          Time duration = WifiPhy::CalculateTxDuration (maxBufferSize, txVector,
                                                        m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                        userInfo.GetAid12 ());
          bufferTxTime = Max (bufferTxTime, duration);
        }

      if (bufferTxTime < maxDuration)
        {
          // the maximum buffer size can be transmitted within the allowed time
          maxDuration = bufferTxTime;
        }
      else
        {
          // maxDuration may be a too short time. If it does not allow any station to
          // transmit at least m_ulPsduSize bytes, give up the UL MU transmission for now
          Time minDuration = Seconds (0);
          for (const auto& userInfo : m_trigger)
            {
              Time duration = WifiPhy::CalculateTxDuration (m_ulPsduSize, txVector,
                                                            m_apMac->GetWifiPhy ()->GetPhyBand (),
                                                            userInfo.GetAid12 ());
              minDuration = (minDuration.IsZero () ? duration : Min (minDuration, duration));
            }

          if (maxDuration < minDuration)
            {
              // maxDuration is a too short time, hence return NO_TX. In this way,
              // no transmission will occur now and the next time we will try again
              // performing an UL OFDMA transmission.
              NS_LOG_DEBUG ("Available time " << maxDuration.As (Time::MS) << " is too short");
              return NO_TX;
            }
        }

      // maxDuration is the time to grant to the stations. Finalize the Trigger Frame
      NS_LOG_DEBUG ("TB PPDU duration: " << maxDuration.As (Time::MS));
      m_trigger.SetUlLength (HePhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                         m_apMac->GetWifiPhy ()->GetPhyBand ()));
      // set Preferred AC to the AC that gained channel access
      for (auto& userInfo : m_trigger)
        {
          userInfo.SetBasicTriggerDepUserInfo (0, 0, m_edca->GetAccessCategory ());

        }

      return UL_MU_TX;
    }
  NS_LOG_DEBUG ("Basic TF was not sent since maxBufferSize = 0 for all candidate STAs");
  NS_LOG_DEBUG ("m_candidates was cleared");
  m_candidates.clear();
  return DL_MU_TX;
}

void
TwtRrMultiUserScheduler::NotifyStationAssociated (uint16_t aid, Mac48Address address)
{
  NS_LOG_FUNCTION (this << aid << address);

  if (GetWifiRemoteStationManager ()->GetHeSupported (address))
    {
      for (auto& staList : m_staList)
        {
          staList.second.push_back (MasterInfo {aid, address, 0.0, 0});
        }
    }
}

void
TwtRrMultiUserScheduler::NotifyStationDeassociated (uint16_t aid, Mac48Address address)
{
  NS_LOG_FUNCTION (this << aid << address);

  if (GetWifiRemoteStationManager ()->GetHeSupported (address))
    {
      for (auto& staList : m_staList)
        {
          staList.second.remove_if ([&aid, &address] (const MasterInfo& info)
                                    { return info.aid == aid && info.address == address; });
        }
    }
}

MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::TrySendingDlMuPpdu (void)
{
  NS_LOG_FUNCTION (this);

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  if (m_staList[primaryAc].empty ())
    {
      NS_LOG_DEBUG ("No HE stations associated: return SU_TX");
      return TxFormat::SU_TX;
    }

  // std::size_t count = std::min (static_cast<std::size_t> (m_nStations), m_staList[primaryAc].size ());
  std::size_t count = std::min (static_cast<std::size_t> (8), m_staList[primaryAc].size ());    // shyam
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);
  NS_ASSERT (count >= 1);

  if (!m_useCentral26TonesRus)
    {
      nCentral26TonesRus = 0;
    }

  uint8_t currTid = wifiAcList.at (primaryAc).GetHighTid ();

  Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();

  if (mpdu != nullptr && mpdu->GetHeader ().IsQosData ())
    {
      currTid = mpdu->GetHeader ().GetQosTid ();
    }

  // determine the list of TIDs to check
  std::vector<uint8_t> tids;

  if (m_enableTxopSharing)
    {
      for (auto acIt = wifiAcList.find (primaryAc); acIt != wifiAcList.end (); acIt++)
        {
          uint8_t firstTid = (acIt->first == primaryAc ? currTid : acIt->second.GetHighTid ());
          tids.push_back (firstTid);
          tids.push_back (acIt->second.GetOtherTid (firstTid));
        }
    }
  else
    {
      tids.push_back (currTid);
    }

  Ptr<HeConfiguration> heConfiguration = m_apMac->GetHeConfiguration ();
  NS_ASSERT (heConfiguration != 0);

  m_txParams.Clear ();
  m_txParams.m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txParams.m_txVector.SetChannelWidth (m_apMac->GetWifiPhy ()->GetChannelWidth ());
  m_txParams.m_txVector.SetGuardInterval (heConfiguration->GetGuardInterval ().GetNanoSeconds ());
  m_txParams.m_txVector.SetBssColor (heConfiguration->GetBssColor ());

  // The TXOP limit can be exceeded by the TXOP holder if it does not transmit more
  // than one Data or Management frame in the TXOP and the frame is not in an A-MPDU
  // consisting of more than one MPDU (Sec. 10.22.2.8 of 802.11-2016).
  // For the moment, we are considering just one MPDU per receiver.
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);
  NS_LOG_DEBUG ("actualAvailableTime=>"<<actualAvailableTime);

  // iterate over the associated stations until an enough number of stations is identified
  auto staIt = m_staList[primaryAc].begin ();
  m_candidates.clear ();

  // while (staIt != m_staList[primaryAc].end ()
  //        && m_candidates.size () < std::min (static_cast<std::size_t> (m_nStations), count + nCentral26TonesRus))
  while (staIt != m_staList[primaryAc].end ()
         && m_candidates.size () < std::min (static_cast<std::size_t> (8), count + nCentral26TonesRus))   // shyam
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << staIt->address << ", AID=" << staIt->aid << ")");

      HeRu::RuType currRuType = (m_candidates.size () < count ? ruType : HeRu::RU_26_TONE);

      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : tids)
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          NS_ASSERT (ac >= primaryAc);
          NS_LOG_DEBUG ("Checking tid ="<<tid*1.0);
          NS_LOG_DEBUG ("Is BA established?  ="<<m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid));
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid))
            {
              mpdu = m_apMac->GetQosTxop (ac)->PeekNextMpdu (tid, staIt->address);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ()),
                               txVectorCopy = m_txParams.m_txVector;

                  m_txParams.m_txVector.SetHeMuUserInfo (staIt->aid,
                                                         {{currRuType, 1, false},
                                                          suTxVector.GetMode (),
                                                          suTxVector.GetNss ()});
                  // to debug
                  
                  std::stringstream mpduStr, paramStr;
                  mpdu->Print( mpduStr );
                  NS_LOG_DEBUG("mpdu is not null. mpdu==>"<<mpduStr.str());
                  m_txParams.Print(paramStr);
                  NS_LOG_DEBUG("m_txParams==>"<<paramStr.str());
                  
                  
                  // ----------------------------------------

                  if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
                    {
                      NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
                      m_txParams.m_txVector = txVectorCopy;
                    }
                  else
                    {
                      // the frame meets the constraints
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << staIt->address << ", AID="
                                    << staIt->aid << ") TID=" << +tid);
                      m_candidates.push_back ({staIt, mpdu});
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << staIt->address << " with TID=" << +tid);
                }
            }
        }

      // move to the next station in the list
      staIt++;
    }

  if (m_candidates.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NO_TX");
          return NO_TX;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return SU_TX");
      return SU_TX;
    }

  return TxFormat::DL_MU_TX;
}

MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::ForceCreateDlMuInfo (void)
{
  NS_LOG_FUNCTION (this);
  uint64_t maxSTAsPerBsrp = 8;

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  NS_ASSERT (!m_staList[primaryAc].empty ());
  // if (m_staList[primaryAc].empty ())
  //   {
  //     NS_LOG_DEBUG ("No HE stations associated: return SU_TX");
  //     return TxFormat::SU_TX;
  //   }

  // std::size_t count = std::min (static_cast<std::size_t> (m_nStations), m_staList[primaryAc].size ());
  std::size_t count = std::min (static_cast<std::size_t> (maxSTAsPerBsrp), m_staList[primaryAc].size ());
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);
  NS_ASSERT (count >= 1);

  if (!m_useCentral26TonesRus)
    {
      nCentral26TonesRus = 0;
    }

  // uint8_t currTid = wifiAcList.at (primaryAc).GetHighTid ();
  uint8_t currTid = 0;

  // Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();

  // if (mpdu != nullptr && mpdu->GetHeader ().IsQosData ())
  //   {
  //     currTid = mpdu->GetHeader ().GetQosTid ();
  //   }

  // determine the list of TIDs to check
  std::vector<uint8_t> tids;
  tids.push_back (currTid);


  Ptr<HeConfiguration> heConfiguration = m_apMac->GetHeConfiguration ();
  NS_ASSERT (heConfiguration != 0);

  m_txParams.Clear ();
  m_txParams.m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txParams.m_txVector.SetChannelWidth (m_apMac->GetWifiPhy ()->GetChannelWidth ());
  m_txParams.m_txVector.SetGuardInterval (heConfiguration->GetGuardInterval ().GetNanoSeconds ());
  m_txParams.m_txVector.SetBssColor (heConfiguration->GetBssColor ());

  // The TXOP limit can be exceeded by the TXOP holder if it does not transmit more
  // than one Data or Management frame in the TXOP and the frame is not in an A-MPDU
  // consisting of more than one MPDU (Sec. 10.22.2.8 of 802.11-2016).
  // For the moment, we are considering just one MPDU per receiver.
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);
  NS_LOG_DEBUG ("actualAvailableTime=>"<<actualAvailableTime);

  // iterate over the associated stations until an enough number of stations is identified
  auto staIt = m_staList[primaryAc].begin ();
  m_candidates.clear ();
  Mac48Address apMacAddress = GetWifiRemoteStationManager() -> GetMac () -> GetAddress ();
  
  // while (staIt != m_staList[primaryAc].end ()
  //        && m_candidates.size () < std::min (static_cast<std::size_t> (m_nStations), count + nCentral26TonesRus))
  while (staIt != m_staList[primaryAc].end ()
         && m_candidates.size () < std::min (static_cast<std::size_t> (maxSTAsPerBsrp), count + nCentral26TonesRus))
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << staIt->address << ", AID=" << staIt->aid << ")");
      
      Mac48Address staMacAddress = staIt->address ;
      uint8_t queueSize = m_apMac->GetMaxBufferStatus (staMacAddress);

      if (GetWifiRemoteStationManager() -> GetExpectingBasicTf (staMacAddress) && GetWifiRemoteStationManager()->GetTwtAwakeForSp(staMacAddress) && (queueSize==255))
      {
        NS_LOG_DEBUG ("STA is expecting TF and is awake for TWT");
        HeRu::RuType currRuType = (m_candidates.size () < count ? ruType : HeRu::RU_26_TONE);

        // check if the AP has at least one frame to be sent to the current station
        for (uint8_t tid : tids)
          {
            AcIndex ac = QosUtilsMapTidToAc (tid);
            NS_ASSERT (ac >= primaryAc);
            NS_LOG_DEBUG ("Checking tid ="<<tid*1.0);
            NS_LOG_DEBUG ("Is BA established?  ="<<m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid));
            // check that a BA agreement is established with the receiver for the
            // considered TID, since ack sequences for DL MU PPDUs require block ack
            if (m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid))
            {
              
              NS_LOG_DEBUG ("Creating (not queueing) one Null AC_BE frame for this STA "<<staMacAddress);
              
              
              Ptr<Packet> pkt = Create<Packet> ();
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_QOSDATA_NULL);
              
              hdr.SetAddr1 (staMacAddress);
              
              hdr.SetAddr2 (apMacAddress);
              hdr.SetAddr3 (apMacAddress);
              hdr.SetDsTo ();
              hdr.SetDsNotFrom ();
              hdr.SetQosTid (0);
              hdr.SetSequenceNumber (0);
              
              Ptr<WifiMacQueueItem> mpdu;
              mpdu = Create<WifiMacQueueItem> (pkt, hdr);
              mpdu->ForceSetQueueAc (AC_BE);    // Set queue AC without actually queueing to MAC
              // mpdu = m_apMac->GetQosTxop (ac)->PeekNextMpdu (tid, staIt->address);


              // Use a temporary TX vector including only the STA-ID of the
              // candidate station to check if the MPDU meets the size and time limits.
              // An RU of the computed size is tentatively assigned to the candidate
              // station, so that the TX duration can be correctly computed.
              WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ()),
                          txVectorCopy = m_txParams.m_txVector;

              m_txParams.m_txVector.SetHeMuUserInfo (staIt->aid,
                                                    {{currRuType, 1, false},
                                                      suTxVector.GetMode (),
                                                      suTxVector.GetNss ()});
              // to debug
              
              std::stringstream mpduStr, paramStr;
              mpdu->Print( mpduStr );
              NS_LOG_DEBUG("mpdu is not null. mpdu==>"<<mpduStr.str());
              m_txParams.Print(paramStr);
              NS_LOG_DEBUG("m_txParams==>"<<paramStr.str());
              
              
              // ----------------------------------------

              if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
                {
                  NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
                  m_txParams.m_txVector = txVectorCopy;
                }
              else
                {
                  // the frame meets the constraints
                  NS_LOG_DEBUG ("Adding candidate STA (MAC=" << staIt->address << ", AID="
                                << staIt->aid << ") TID=" << +tid);
                  m_candidates.push_back ({staIt, mpdu});
                  break;    // terminate the for loop
                }

            }
          }
      }



      // move to the next station in the list
      staIt++;
    }

  if (m_candidates.empty ())
    {
      
      NS_LOG_DEBUG ("return NO_TX");
      return NO_TX;


    }

  // Computing DlMuInfo with dummy frames
  // Modified from ComputeDlMuInfo() --------------------- below
  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_txParams.GetPsduInfoMap ().size ();
  // std::size_t nCentral26TonesRus;
  ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);

  NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

  if (!m_useCentral26TonesRus || m_candidates.size () == nRusAssigned)
    {
      nCentral26TonesRus = 0;
    }
  else
    {
      nCentral26TonesRus = std::min (m_candidates.size () - nRusAssigned, nCentral26TonesRus);
      NS_LOG_DEBUG (nCentral26TonesRus << " stations are being assigned a 26-tones RU");
    }

  DlMuInfo dlMuInfo;
  NS_LOG_DEBUG ("Beginning computation of dummy dlMuInfo");

  // We have to update the TXVECTOR
  dlMuInfo.txParams.m_txVector.SetPreambleType (m_txParams.m_txVector.GetPreambleType ());
  dlMuInfo.txParams.m_txVector.SetChannelWidth (m_txParams.m_txVector.GetChannelWidth ());
  dlMuInfo.txParams.m_txVector.SetGuardInterval (m_txParams.m_txVector.GetGuardInterval ());
  dlMuInfo.txParams.m_txVector.SetBssColor (m_txParams.m_txVector.GetBssColor ());

  auto candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      // AssignRuIndices will be called below to set RuSpec
      dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
      candidateIt++;
    }

  // remove candidates that will not be served
  m_candidates.erase (candidateIt, m_candidates.end ());

  AssignRuIndices (dlMuInfo.txParams.m_txVector);
  NS_LOG_DEBUG ("AssignRuIndices completed");
  m_txParams.Clear ();
  // Ptr<const WifiMacQueueItem> mpdu;

  // Compute the TX params (again) by using the stored MPDUs and the final TXVECTOR
  actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);
  Ptr<const WifiMacQueueItem> mpdu;
  for (const auto& candidate : m_candidates)
    {
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);

      [[maybe_unused]] bool ret = m_heFem->TryAddMpdu (mpdu, dlMuInfo.txParams, actualAvailableTime);
      NS_ASSERT_MSG (ret, "Weird that an MPDU does not meet constraints when "
                          "transmitted over a larger RU");
    }

  // We have to complete the PSDUs to send
  Ptr<WifiMacQueue> queue;
  Mac48Address receiver;
  for (const auto& candidate : m_candidates)
    {
      // Let us try first A-MSDU aggregation if possible
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);
      // uint8_t tid = mpdu->GetHeader ().GetQosTid ();
      receiver = mpdu->GetHeader ().GetAddr1 ();
      NS_LOG_DEBUG ("Current STA being processed for PSDU: "<<receiver);
      NS_ASSERT (receiver == candidate.first->address);
      NS_LOG_DEBUG ("Asserted"); 
      // NS_ASSERT (mpdu->IsQueued ());
      
      // Ptr<WifiMacQueueItem> item = mpdu->GetItem ();
      WifiMacHeader hdr1 = mpdu->GetHeader();
      Ptr <Packet> pkt1 = mpdu->GetPacket()->Copy();
      Ptr<WifiMacQueueItem> mpdu1;
      mpdu1 = Create<WifiMacQueueItem> (pkt1, hdr1);


      // mpdu = Create<WifiMacQueueItem> (pkt, hdr);
      NS_LOG_DEBUG ("Got MPDU item");
      // if (!mpdu->GetHeader ().IsRetry ())
      //   {
      //     item = mpdu->GetItem ();
      //     // m_apMac->GetQosTxop (QosUtilsMapTidToAc (tid))->AssignSequenceNumber (item);
      //   }

      
      // std::vector<Ptr<WifiMacQueueItem>> mpduList = m_heFem->GetMpduAggregator ()->GetNextAmpdu (item, dlMuInfo.txParams, m_availableTime);

      
      // dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (item, true);
      dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (mpdu1, true);
        
    }
  // dlMuInfo is now ready
  m_dlInfo = dlMuInfo;
  // Modified from ComputeDlMuInfo() --------------------- above
  NS_LOG_DEBUG ("Computed dlMuInfo; Returning DL_MU_TX");
  return TxFormat::DL_MU_TX;
}
MultiUserScheduler::TxFormat
TwtRrMultiUserScheduler::ForceCreateDlMuInfoKnownStaOnly (void)
{
  NS_LOG_FUNCTION (this);

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  NS_ASSERT (!m_staList[primaryAc].empty ());
  // if (m_staList[primaryAc].empty ())
  //   {
  //     NS_LOG_DEBUG ("No HE stations associated: return SU_TX");
  //     return TxFormat::SU_TX;
  //   }

  // std::size_t count = std::min (static_cast<std::size_t> (40), m_staList[primaryAc].size ());   // shyam
  std::size_t count = std::min (static_cast<std::size_t> (m_nStations), m_staList[primaryAc].size ());
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (m_apMac->GetWifiPhy ()->GetChannelWidth (), count,
                                                           nCentral26TonesRus);
  NS_ASSERT (count >= 1);

  if (!m_useCentral26TonesRus)
    {
      nCentral26TonesRus = 0;
    }

  // uint8_t currTid = wifiAcList.at (primaryAc).GetHighTid ();
  uint8_t currTid = 0;

  // Ptr<const WifiMacQueueItem> mpdu = m_edca->PeekNextMpdu ();

  // if (mpdu != nullptr && mpdu->GetHeader ().IsQosData ())
  //   {
  //     currTid = mpdu->GetHeader ().GetQosTid ();
  //   }

  // determine the list of TIDs to check
  std::vector<uint8_t> tids;
  tids.push_back (currTid);


  Ptr<HeConfiguration> heConfiguration = m_apMac->GetHeConfiguration ();
  NS_ASSERT (heConfiguration != 0);

  m_txParams.Clear ();
  m_txParams.m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txParams.m_txVector.SetChannelWidth (m_apMac->GetWifiPhy ()->GetChannelWidth ());
  m_txParams.m_txVector.SetGuardInterval (heConfiguration->GetGuardInterval ().GetNanoSeconds ());
  m_txParams.m_txVector.SetBssColor (heConfiguration->GetBssColor ());

  // The TXOP limit can be exceeded by the TXOP holder if it does not transmit more
  // than one Data or Management frame in the TXOP and the frame is not in an A-MPDU
  // consisting of more than one MPDU (Sec. 10.22.2.8 of 802.11-2016).
  // For the moment, we are considering just one MPDU per receiver.
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);
  NS_LOG_DEBUG ("actualAvailableTime=>"<<actualAvailableTime);

  // iterate over the associated stations until an enough number of stations is identified
  auto staIt = m_staList[primaryAc].begin ();
  m_candidates.clear ();
  Mac48Address apMacAddress = GetWifiRemoteStationManager() -> GetMac () -> GetAddress ();
  
  while (staIt != m_staList[primaryAc].end ()
         && m_candidates.size () < std::min (static_cast<std::size_t> (m_nStations), count + nCentral26TonesRus))
  // while (staIt != m_staList[primaryAc].end ()
  //        && m_candidates.size () < std::min (static_cast<std::size_t> (40), count + nCentral26TonesRus))    // shyam
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << staIt->address << ", AID=" << staIt->aid << ")");
      
      Mac48Address staMacAddress = staIt->address ;
      uint8_t queueSize = m_apMac->GetMaxBufferStatus (staMacAddress);

      // if (GetWifiRemoteStationManager() -> GetExpectingBasicTf (staMacAddress) && GetWifiRemoteStationManager()->GetTwtAwakeForSp(staMacAddress) && (queueSize!=255) && (queueSize>0))
      if (GetWifiRemoteStationManager()->GetTwtAwakeForSp(staMacAddress) && (queueSize!=255) && (queueSize>0))
      {
        NS_LOG_DEBUG ("STA has buffered packets and is awake for TWT");
        HeRu::RuType currRuType = (m_candidates.size () < count ? ruType : HeRu::RU_26_TONE);

        // check if the AP has at least one frame to be sent to the current station
        for (uint8_t tid : tids)
          {
            AcIndex ac = QosUtilsMapTidToAc (tid);
            NS_ASSERT (ac >= primaryAc);
            NS_LOG_DEBUG ("Checking tid ="<<tid*1.0);
            NS_LOG_DEBUG ("Is BA established?  ="<<m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid));
            // check that a BA agreement is established with the receiver for the
            // considered TID, since ack sequences for DL MU PPDUs require block ack
            if (m_apMac->GetQosTxop (ac)->GetBaAgreementEstablished (staIt->address, tid))
            {
              
              NS_LOG_DEBUG ("Creating (not queueing) one Null AC_BE frame for this STA "<<staMacAddress);
              
              
              Ptr<Packet> pkt = Create<Packet> ();
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_QOSDATA_NULL);
              
              hdr.SetAddr1 (staMacAddress);
              
              hdr.SetAddr2 (apMacAddress);
              hdr.SetAddr3 (apMacAddress);
              hdr.SetDsTo ();
              hdr.SetDsNotFrom ();
              hdr.SetQosTid (0);
              hdr.SetSequenceNumber (0);
              
              Ptr<WifiMacQueueItem> mpdu;
              mpdu = Create<WifiMacQueueItem> (pkt, hdr);
              mpdu->ForceSetQueueAc (AC_BE);    // Set queue AC without actually queueing to MAC
              // mpdu = m_apMac->GetQosTxop (ac)->PeekNextMpdu (tid, staIt->address);


              // Use a temporary TX vector including only the STA-ID of the
              // candidate station to check if the MPDU meets the size and time limits.
              // An RU of the computed size is tentatively assigned to the candidate
              // station, so that the TX duration can be correctly computed.
              WifiTxVector suTxVector = GetWifiRemoteStationManager ()->GetDataTxVector (mpdu->GetHeader ()),
                          txVectorCopy = m_txParams.m_txVector;

              m_txParams.m_txVector.SetHeMuUserInfo (staIt->aid,
                                                    {{currRuType, 1, false},
                                                      suTxVector.GetMode (),
                                                      suTxVector.GetNss ()});
              // to debug
              
              std::stringstream mpduStr, paramStr;
              mpdu->Print( mpduStr );
              NS_LOG_DEBUG("mpdu is not null. mpdu==>"<<mpduStr.str());
              m_txParams.Print(paramStr);
              NS_LOG_DEBUG("m_txParams==>"<<paramStr.str());
              
              
              // ----------------------------------------

              if (!m_heFem->TryAddMpdu (mpdu, m_txParams, actualAvailableTime))
                {
                  NS_LOG_DEBUG ("Adding the peeked frame violates the time constraints");
                  m_txParams.m_txVector = txVectorCopy;
                }
              else
                {
                  // the frame meets the constraints
                  NS_LOG_DEBUG ("Adding candidate STA (MAC=" << staIt->address << ", AID="
                                << staIt->aid << ") TID=" << +tid);
                  m_candidates.push_back ({staIt, mpdu});
                  break;    // terminate the for loop
                }

            }
          }
      }



      // move to the next station in the list
      staIt++;
    }

  if (m_candidates.empty ())
    {
      
      NS_LOG_DEBUG ("return NO_TX");
      return NO_TX;


    }

  // Computing DlMuInfo with dummy frames
  // Modified from ComputeDlMuInfo() --------------------- below
  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_txParams.GetPsduInfoMap ().size ();
  // std::size_t nCentral26TonesRus;
  ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);

  NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

  if (!m_useCentral26TonesRus || m_candidates.size () == nRusAssigned)
    {
      nCentral26TonesRus = 0;
    }
  else
    {
      nCentral26TonesRus = std::min (m_candidates.size () - nRusAssigned, nCentral26TonesRus);
      NS_LOG_DEBUG (nCentral26TonesRus << " stations are being assigned a 26-tones RU");
    }

  DlMuInfo dlMuInfo;
  NS_LOG_DEBUG ("Beginning computation of dummy dlMuInfo");

  // We have to update the TXVECTOR
  dlMuInfo.txParams.m_txVector.SetPreambleType (m_txParams.m_txVector.GetPreambleType ());
  dlMuInfo.txParams.m_txVector.SetChannelWidth (m_txParams.m_txVector.GetChannelWidth ());
  dlMuInfo.txParams.m_txVector.SetGuardInterval (m_txParams.m_txVector.GetGuardInterval ());
  dlMuInfo.txParams.m_txVector.SetBssColor (m_txParams.m_txVector.GetBssColor ());

  auto candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      // AssignRuIndices will be called below to set RuSpec
      dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
      candidateIt++;
    }

  // remove candidates that will not be served
  m_candidates.erase (candidateIt, m_candidates.end ());

  AssignRuIndices (dlMuInfo.txParams.m_txVector);
  NS_LOG_DEBUG ("AssignRuIndices completed");
  m_txParams.Clear ();
  // Ptr<const WifiMacQueueItem> mpdu;

  // Compute the TX params (again) by using the stored MPDUs and the final TXVECTOR
  actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);
  Ptr<const WifiMacQueueItem> mpdu;
  for (const auto& candidate : m_candidates)
    {
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);

      [[maybe_unused]] bool ret = m_heFem->TryAddMpdu (mpdu, dlMuInfo.txParams, actualAvailableTime);
      NS_ASSERT_MSG (ret, "Weird that an MPDU does not meet constraints when "
                          "transmitted over a larger RU");
    }

  // We have to complete the PSDUs to send
  Ptr<WifiMacQueue> queue;
  Mac48Address receiver;
  for (const auto& candidate : m_candidates)
    {
      // Let us try first A-MSDU aggregation if possible
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);
      // uint8_t tid = mpdu->GetHeader ().GetQosTid ();
      receiver = mpdu->GetHeader ().GetAddr1 ();
      NS_LOG_DEBUG ("Current STA being processed for PSDU: "<<receiver);
      NS_ASSERT (receiver == candidate.first->address);
      NS_LOG_DEBUG ("Asserted"); 
      // NS_ASSERT (mpdu->IsQueued ());
      
      // Ptr<WifiMacQueueItem> item = mpdu->GetItem ();
      WifiMacHeader hdr1 = mpdu->GetHeader();
      Ptr <Packet> pkt1 = mpdu->GetPacket()->Copy();
      Ptr<WifiMacQueueItem> mpdu1;
      mpdu1 = Create<WifiMacQueueItem> (pkt1, hdr1);


      // mpdu = Create<WifiMacQueueItem> (pkt, hdr);
      NS_LOG_DEBUG ("Got MPDU item");
      // if (!mpdu->GetHeader ().IsRetry ())
      //   {
      //     item = mpdu->GetItem ();
      //     // m_apMac->GetQosTxop (QosUtilsMapTidToAc (tid))->AssignSequenceNumber (item);
      //   }

      
      // std::vector<Ptr<WifiMacQueueItem>> mpduList = m_heFem->GetMpduAggregator ()->GetNextAmpdu (item, dlMuInfo.txParams, m_availableTime);

      
      // dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (item, true);
      dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (mpdu1, true);
        
    }
  // dlMuInfo is now ready
  m_dlInfo = dlMuInfo;
  // Modified from ComputeDlMuInfo() --------------------- above
  NS_LOG_DEBUG ("Computed dlMuInfo; Returning DL_MU_TX");
  return TxFormat::DL_MU_TX;
}

MultiUserScheduler::DlMuInfo
TwtRrMultiUserScheduler::ComputeDlMuInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_candidates.empty ())
    {
      return DlMuInfo ();
    }

  uint16_t bw = m_apMac->GetWifiPhy ()->GetChannelWidth ();

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_txParams.GetPsduInfoMap ().size ();
  std::size_t nCentral26TonesRus;
  HeRu::RuType ruType = HeRu::GetEqualSizedRusForStations (bw, nRusAssigned, nCentral26TonesRus);

  NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

  if (!m_useCentral26TonesRus || m_candidates.size () == nRusAssigned)
    {
      nCentral26TonesRus = 0;
    }
  else
    {
      nCentral26TonesRus = std::min (m_candidates.size () - nRusAssigned, nCentral26TonesRus);
      NS_LOG_DEBUG (nCentral26TonesRus << " stations are being assigned a 26-tones RU");
    }

  DlMuInfo dlMuInfo;

  // We have to update the TXVECTOR
  dlMuInfo.txParams.m_txVector.SetPreambleType (m_txParams.m_txVector.GetPreambleType ());
  dlMuInfo.txParams.m_txVector.SetChannelWidth (m_txParams.m_txVector.GetChannelWidth ());
  dlMuInfo.txParams.m_txVector.SetGuardInterval (m_txParams.m_txVector.GetGuardInterval ());
  dlMuInfo.txParams.m_txVector.SetBssColor (m_txParams.m_txVector.GetBssColor ());

  auto candidateIt = m_candidates.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      uint16_t staId = candidateIt->first->aid;
      // AssignRuIndices will be called below to set RuSpec
      dlMuInfo.txParams.m_txVector.SetHeMuUserInfo (staId,
                                                    {{(i < nRusAssigned ? ruType : HeRu::RU_26_TONE), 1, false},
                                                      m_txParams.m_txVector.GetMode (staId),
                                                      m_txParams.m_txVector.GetNss (staId)});
      candidateIt++;
    }

  // remove candidates that will not be served
  m_candidates.erase (candidateIt, m_candidates.end ());

  AssignRuIndices (dlMuInfo.txParams.m_txVector);
  m_txParams.Clear ();

  Ptr<const WifiMacQueueItem> mpdu;

  // Compute the TX params (again) by using the stored MPDUs and the final TXVECTOR
  Time actualAvailableTime = (m_initialFrame ? Time::Min () : m_availableTime);

  for (const auto& candidate : m_candidates)
    {
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);

      [[maybe_unused]] bool ret = m_heFem->TryAddMpdu (mpdu, dlMuInfo.txParams, actualAvailableTime);
      NS_ASSERT_MSG (ret, "Weird that an MPDU does not meet constraints when "
                          "transmitted over a larger RU");
    }

  // We have to complete the PSDUs to send
  Ptr<WifiMacQueue> queue;
  Mac48Address receiver;

  for (const auto& candidate : m_candidates)
    {
      // Let us try first A-MSDU aggregation if possible
      mpdu = candidate.second;
      NS_ASSERT (mpdu != nullptr);
      uint8_t tid = mpdu->GetHeader ().GetQosTid ();
      receiver = mpdu->GetHeader ().GetAddr1 ();
      NS_ASSERT (receiver == candidate.first->address);

      NS_ASSERT (mpdu->IsQueued ());
      Ptr<WifiMacQueueItem> item = mpdu->GetItem ();

      if (!mpdu->GetHeader ().IsRetry ())
        {
          // this MPDU must have been dequeued from the AC queue and we can try
          // A-MSDU aggregation
          item = m_heFem->GetMsduAggregator ()->GetNextAmsdu (mpdu, dlMuInfo.txParams, m_availableTime);

          if (item == nullptr)
            {
              // A-MSDU aggregation failed or disabled
              item = mpdu->GetItem ();
            }
          m_apMac->GetQosTxop (QosUtilsMapTidToAc (tid))->AssignSequenceNumber (item);
        }

      // Now, let's try A-MPDU aggregation if possible
      std::vector<Ptr<WifiMacQueueItem>> mpduList = m_heFem->GetMpduAggregator ()->GetNextAmpdu (item, dlMuInfo.txParams, m_availableTime);

      if (mpduList.size () > 1)
        {
          // A-MPDU aggregation succeeded, update psduMap
          dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (std::move (mpduList));
        }
      else
        {
          dlMuInfo.psduMap[candidate.first->aid] = Create<WifiPsdu> (item, true);
        }
    }

  AcIndex primaryAc = m_edca->GetAccessCategory ();

  // The amount of credits received by each station equals the TX duration (in
  // microseconds) divided by the number of stations.
  double creditsPerSta = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
                        / m_staList[primaryAc].size ();
  // Transmitting stations have to pay a number of credits equal to the TX duration
  // (in microseconds) times the allocated bandwidth share.
  double debitsPerMhz = dlMuInfo.txParams.m_txDuration.ToDouble (Time::US)
                        / (nRusAssigned * HeRu::GetBandwidth (ruType)
                          + nCentral26TonesRus * HeRu::GetBandwidth (HeRu::RU_26_TONE));

  // assign credits to all stations
  for (auto& sta : m_staList[primaryAc])
    {
      sta.credits += creditsPerSta;
      sta.credits = std::min (sta.credits, m_maxCredits.ToDouble (Time::US));
    }

  // subtract debits to the selected stations
  candidateIt = m_candidates.begin ();

  for (std::size_t i = 0; i < nRusAssigned + nCentral26TonesRus; i++)
    {
      NS_ASSERT (candidateIt != m_candidates.end ());

      candidateIt->first->credits -= debitsPerMhz * HeRu::GetBandwidth (i < nRusAssigned ? ruType : HeRu::RU_26_TONE);

      candidateIt++;
    }

  // sort the list in decreasing order of credits
  m_staList[primaryAc].sort ([] (const MasterInfo& a, const MasterInfo& b)
                              { return a.credits > b.credits; });

  NS_LOG_DEBUG ("Next station to serve has AID=" << m_staList[primaryAc].front ().aid);

  return dlMuInfo;
}

void
TwtRrMultiUserScheduler::AssignRuIndices (WifiTxVector& txVector)
{
  NS_LOG_FUNCTION (this << txVector);

  uint8_t bw = txVector.GetChannelWidth ();

  // find the RU types allocated in the TXVECTOR
  std::set<HeRu::RuType> ruTypeSet;
  for (const auto& userInfo : txVector.GetHeMuUserInfoMap ())
    {
      ruTypeSet.insert (userInfo.second.ru.GetRuType ());
    }

  std::vector<HeRu::RuSpec> ruSet, central26TonesRus;

  // This scheduler allocates equal sized RUs and optionally the remaining 26-tone RUs
  if (ruTypeSet.size () == 2)
    {
      // central 26-tone RUs have been allocated
      NS_ASSERT (ruTypeSet.find (HeRu::RU_26_TONE) != ruTypeSet.end ());
      ruTypeSet.erase (HeRu::RU_26_TONE);
      NS_ASSERT (ruTypeSet.size () == 1);
      central26TonesRus = HeRu::GetCentral26TonesRus (bw, *ruTypeSet.begin ());
    }

  NS_ASSERT (ruTypeSet.size () == 1);
  ruSet = HeRu::GetRusOfType (bw, *ruTypeSet.begin ());

  auto ruSetIt = ruSet.begin ();
  auto central26TonesRusIt = central26TonesRus.begin ();

  for (const auto& userInfo : txVector.GetHeMuUserInfoMap ())
    {
      if (userInfo.second.ru.GetRuType () == *ruTypeSet.begin ())
        {
          NS_ASSERT (ruSetIt != ruSet.end ());
          txVector.SetRu (*ruSetIt, userInfo.first);
          ruSetIt++;
        }
      else
        {
          NS_ASSERT (central26TonesRusIt != central26TonesRus.end ());
          txVector.SetRu (*central26TonesRusIt, userInfo.first);
          central26TonesRusIt++;
        }
    }
}

MultiUserScheduler::UlMuInfo
TwtRrMultiUserScheduler::ComputeUlMuInfo (void)
{
  return UlMuInfo {m_trigger, m_triggerMacHdr, std::move (m_txParams)};
}

} //namespace ns3
