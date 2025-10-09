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
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/random-variable-stream.h"
#include "ap-wifi-mac.h"
#include "channel-access-manager.h"
#include "mac-tx-middle.h"
#include "mac-rx-middle.h"
#include "mgt-headers.h"
#include "msdu-aggregator.h"
#include "amsdu-subframe-header.h"
#include "wifi-phy.h"
#include "wifi-net-device.h"
#include "wifi-mac-queue.h"
#include "ns3/ht-configuration.h"
#include "ns3/he-configuration.h"
#include "ns3/ctrl-headers.h"
#include "ns3/he-phy.h"
#include "ns3/wifi-psdu.h"
#include "wifi-twt-agreement.h"

#include "ns3/he-frame-exchange-manager.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ApWifiMac");

NS_OBJECT_ENSURE_REGISTERED (ApWifiMac);

TypeId
ApWifiMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ApWifiMac")
    .SetParent<RegularWifiMac> ()
    .SetGroupName ("Wifi")
    .AddConstructor<ApWifiMac> ()
    .AddAttribute ("BeaconInterval",
                   "Delay between two beacons",
                   TimeValue (MicroSeconds (102400)),
                   MakeTimeAccessor (&ApWifiMac::GetBeaconInterval,
                                     &ApWifiMac::SetBeaconInterval),
                   MakeTimeChecker ())
    .AddAttribute ("DtimPeriod",
                   "DTIM period - one in DtimPeriod beacons is a DTIM beacon",
                   UintegerValue (1),
                   MakeUintegerAccessor (&ApWifiMac::m_dtimPeriod),
                   MakeUintegerChecker<uint8_t> ())                   
    .AddAttribute ("BeaconJitter",
                   "A uniform random variable to cause the initial beacon starting time (after simulation time 0) "
                   "to be distributed between 0 and the BeaconInterval.",
                   StringValue ("ns3::UniformRandomVariable"),
                   MakePointerAccessor (&ApWifiMac::m_beaconJitter),
                   MakePointerChecker<UniformRandomVariable> ())
    .AddAttribute ("EnableBeaconJitter",
                   "If beacons are enabled, whether to jitter the initial send event.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ApWifiMac::m_enableBeaconJitter),
                   MakeBooleanChecker ())
    .AddAttribute ("BeaconGeneration",
                   "Whether or not beacons are generated.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ApWifiMac::SetBeaconGeneration),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableNonErpProtection", "Whether or not protection mechanism should be used when non-ERP STAs are present within the BSS."
                   "This parameter is only used when ERP is supported by the AP.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ApWifiMac::m_enableNonErpProtection),
                   MakeBooleanChecker ())
    .AddAttribute ("BsrLifetime",
                   "Lifetime of Buffer Status Reports received from stations.",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&ApWifiMac::m_bsrLifetime),
                   MakeTimeChecker ())
    .AddAttribute ("PsUnicastBufferSize",
                   "Size of Queue/buffer created for PS STAs for buffering packets when STAs are in PS till retrieved or dropped.",
                   QueueSizeValue (QueueSize ("10000p")),
                   MakeQueueSizeAccessor (&ApWifiMac::m_psBufferSize),
                   MakeQueueSizeChecker ())                   
    .AddAttribute ("PsUnicastBufferDropDelay",
                   "Any packet in the PS queue/buffer for STAs longer than PsStaBufferDropDelay will be dropped.",
                   TimeValue (MilliSeconds (50000)),
                   MakeTimeAccessor (&ApWifiMac::m_psBufferDropDelay),
                   MakeTimeChecker ())                     
    .AddTraceSource ("AssociatedSta",
                     "A station associated with this access point.",
                     MakeTraceSourceAccessor (&ApWifiMac::m_assocLogger),
                     "ns3::ApWifiMac::AssociationCallback")
    .AddTraceSource ("DeAssociatedSta",
                     "A station lost association with this access point.",
                     MakeTraceSourceAccessor (&ApWifiMac::m_deAssocLogger),
                     "ns3::ApWifiMac::AssociationCallback")
    .AddAttribute ("EnableUApsd", "If True, U-APSD is assumed instead of Basic PSM for STAs setting PM = 1; If false, PSM is used",
                   BooleanValue (false),
                   MakeBooleanAccessor (&ApWifiMac::m_psmOrApsd),
                   MakeBooleanChecker ())                     
  ;
  return tid;
}

ApWifiMac::ApWifiMac ()
  : m_enableBeaconGeneration (false),
    m_numNonErpStations (0),
    m_numNonHtStations (0),
    m_shortSlotTimeEnabled (false),
    m_shortPreambleEnabled (false),
    // m_dtimPeriod (1),  // Added as attribute
    m_dtimCount (0)
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop = CreateObject<Txop> (CreateObject<WifiMacQueue> (AC_BEACON));
  m_beaconTxop->SetWifiMac (this);
  m_beaconTxop->SetAifsn (1);
  m_beaconTxop->SetMinCw (0);
  m_beaconTxop->SetMaxCw (0);
  m_beaconTxop->SetChannelAccessManager (m_channelAccessManager);
  m_beaconTxop->SetTxMiddle (m_txMiddle);

  //Let the lower layers know that we are acting as an AP.
  SetTypeOfStation (AP);


  // Creating empty Unicast buffer for PS stations
  m_psUnicastBuffer = CreateObject<WifiMacQueue> (AC_BE);
  // m_psUnicastBuffer->SetMaxSize (m_psBufferSize);
  m_psUnicastBuffer->SetMaxSize (QueueSize ("10000p"));    // shyam - make this an attribute - current attribute not working
  // m_psUnicastBuffer->SetMaxDelay (m_psBufferDropDelay);
  m_psUnicastBuffer->SetMaxDelay (Time (MilliSeconds(20000)));   // shyam - make this an attribute - current attribute not working
  m_psUnicastBuffer->SetAttribute ("DropPolicy", EnumValue (WifiMacQueue::DROP_OLDEST));
  NS_LOG_DEBUG ("Empty Unicast buffer created upon AP initialization. Max delay of Buffer is "<< m_psUnicastBuffer->GetMaxDelay() );
  NS_LOG_DEBUG ("Number of packets in Unicast buffer now is "<< m_psUnicastBuffer->GetNPackets() );
  NS_LOG_DEBUG ("Max Size of Unicast buffer is "<< m_psUnicastBuffer->GetMaxSize () );


  // Creating empty Multicast buffer for PS stations
  m_psMulticastBuffer = CreateObject<WifiMacQueue> (AC_BE);
  // m_psMulticastBuffer->SetMaxSize (m_psBufferSize);
  m_psMulticastBuffer->SetMaxSize (QueueSize ("10000p"));    // shyam - make this an attribute - current attribute not working
  // m_psMulticastBuffer->SetMaxDelay (m_psBufferDropDelay);
  m_psMulticastBuffer->SetMaxDelay (Time (MilliSeconds(2000)));   // shyam - make this an attribute - current attribute not working
  m_psMulticastBuffer->SetAttribute ("DropPolicy", EnumValue (WifiMacQueue::DROP_OLDEST));
  NS_LOG_DEBUG ("Empty Multicast buffer created upon AP initialization. Max delay of Buffer is "<< m_psMulticastBuffer->GetMaxDelay() );
  NS_LOG_DEBUG ("Number of packets in Multicast buffer now is "<< m_psMulticastBuffer->GetNPackets() );
  NS_LOG_DEBUG ("Max Size of Multicast buffer is "<< m_psMulticastBuffer->GetMaxSize () );
}

ApWifiMac::~ApWifiMac ()
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
}

void
ApWifiMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop->Dispose ();
  m_beaconTxop = 0;
  m_enableBeaconGeneration = false;
  m_beaconEvent.Cancel ();
  RegularWifiMac::DoDispose ();
}

void
ApWifiMac::SetAddress (Mac48Address address)
{
  NS_LOG_FUNCTION (this << address);
  //As an AP, our MAC address is also the BSSID. Hence we are
  //overriding this function and setting both in our parent class.
  RegularWifiMac::SetAddress (address);
  RegularWifiMac::SetBssid (address);
}

Ptr<WifiMacQueue>
ApWifiMac::GetTxopQueue (AcIndex ac) const
{
  if (ac == AC_BEACON)
    {
      return m_beaconTxop->GetWifiMacQueue ();
    }
  return RegularWifiMac::GetTxopQueue (ac);
}

void
ApWifiMac::SetBeaconGeneration (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  if (!enable)
    {
      m_beaconEvent.Cancel ();
    }
  else if (enable && !m_enableBeaconGeneration)
    {
      m_beaconEvent = Simulator::ScheduleNow (&ApWifiMac::SendOneBeacon, this);
    }
  m_enableBeaconGeneration = enable;
}

Time
ApWifiMac::GetBeaconInterval (void) const
{
  NS_LOG_FUNCTION (this);
  return m_beaconInterval;
}

void
ApWifiMac::SetLinkUpCallback (Callback<void> linkUp)
{
  NS_LOG_FUNCTION (this << &linkUp);
  RegularWifiMac::SetLinkUpCallback (linkUp);

  //The approach taken here is that, from the point of view of an AP,
  //the link is always up, so we immediately invoke the callback if
  //one is set
  linkUp ();
}

void
ApWifiMac::SetBeaconInterval (Time interval)
{
  NS_LOG_FUNCTION (this << interval);
  if ((interval.GetMicroSeconds () % 1024) != 0)
    {
      NS_FATAL_ERROR ("beacon interval should be multiple of 1024us (802.11 time unit), see IEEE Std. 802.11-2012");
    }
  if (interval.GetMicroSeconds () > (1024 * 65535))
    {
      NS_FATAL_ERROR ("beacon interval should be smaller then or equal to 65535 * 1024us (802.11 time unit)");
    }
  m_beaconInterval = interval;
}

int64_t
ApWifiMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_beaconJitter->SetStream (stream);
  return 1;
}

void
ApWifiMac::UpdateShortSlotTimeEnabled (void)
{
  NS_LOG_FUNCTION (this);
  if (GetErpSupported () && GetShortSlotTimeSupported () && (m_numNonErpStations == 0))
    {
      for (const auto& sta : m_staList)
        {
          if (!m_stationManager->GetShortSlotTimeSupported (sta.second))
            {
              m_shortSlotTimeEnabled = false;
              return;
            }
        }
      m_shortSlotTimeEnabled = true;
    }
  else
    {
      m_shortSlotTimeEnabled = false;
    }
}

void
ApWifiMac::UpdateShortPreambleEnabled (void)
{
  NS_LOG_FUNCTION (this);
  if (GetErpSupported () && m_phy->GetShortPhyPreambleSupported ())
    {
      for (const auto& sta : m_staList)
        {
          if (!m_stationManager->GetErpOfdmSupported (sta.second) ||
              !m_stationManager->GetShortPreambleSupported (sta.second))
            {
              m_shortPreambleEnabled = false;
              return;
            }
        }
      m_shortPreambleEnabled = true;
    }
  else
    {
      m_shortPreambleEnabled = false;
    }
}

uint16_t
ApWifiMac::GetVhtOperationalChannelWidth (void) const
{
  uint16_t channelWidth = m_phy->GetChannelWidth ();
  for (const auto& sta : m_staList)
    {
      if (m_stationManager->GetVhtSupported (sta.second))
        {
          if (m_stationManager->GetChannelWidthSupported (sta.second) < channelWidth)
            {
              channelWidth = m_stationManager->GetChannelWidthSupported (sta.second);
            }
        }
    }
  return channelWidth;
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << from << to);
  //If we are not a QoS AP then we definitely want to use AC_BE to
  //transmit the packet. A TID of zero will map to AC_BE (through \c
  //QosUtilsMapTidToAc()), so we use that as our default here.
  uint8_t tid = 0;

  //If we are a QoS AP then we attempt to get a TID for this packet
  if (GetQosSupported ())
    {
      tid = QosUtilsGetTidForPacket (packet);
      //Any value greater than 7 is invalid and likely indicates that
      //the packet had no QoS tag, so we revert to zero, which'll
      //mean that AC_BE is used.
      if (tid > 7)
        {
          tid = 0;
        }
    }

  ForwardDown (packet, from, to, tid);
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to, uint8_t tid)
{
  NS_LOG_FUNCTION (this << packet << from << to << +tid);
  WifiMacHeader hdr;

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
      //Transmission of multiple frames in the same Polled TXOP is not supported for now
      hdr.SetQosTxopLimit (0);
      //Fill in the QoS control field in the MAC header
      hdr.SetQosTid (tid);
      // If this STA has APSD and there are no more buffered frames, set EOSP = 1
      // if ( m_psUnicastBuffer->GetNPacketsByAddress (to) == 0 && m_stationManager->GetPSM (to) && m_psmOrApsd)
      // {
      //   NS_LOG_DEBUG ("this STA "<<to<<" has APSD and there are no more buffered frames, setting EOSP = 1 for this frame");
      //   hdr.SetQosEosp ();
      // }
      // else
      // {
      //   hdr.SetQosNoEosp ();
      // }
    }
  else
    {
      hdr.SetType (WIFI_MAC_DATA);
    }

  if (GetQosSupported ())
    {
      hdr.SetNoOrder (); // explicitly set to 0 for the time being since HT control field is not yet implemented (set it to 1 when implemented)
    }
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (from);
  hdr.SetDsFrom ();
  hdr.SetDsNotTo ();
  
  if (GetQosSupported ())
    {
      //Sanity check that the TID is valid
      NS_ASSERT (tid < 8);
      m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
    }
  else
    {
      m_txop->Queue (packet, hdr);
    }
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to, bool moreData, bool pushFront)
{
  NS_LOG_FUNCTION (this << packet << from << to);
  //If we are not a QoS AP then we definitely want to use AC_BE to
  //transmit the packet. A TID of zero will map to AC_BE (through \c
  //QosUtilsMapTidToAc()), so we use that as our default here.
  uint8_t tid = 0;

  //If we are a QoS AP then we attempt to get a TID for this packet
  if (GetQosSupported ())
    {
      tid = QosUtilsGetTidForPacket (packet);
      //Any value greater than 7 is invalid and likely indicates that
      //the packet had no QoS tag, so we revert to zero, which'll
      //mean that AC_BE is used.
      if (tid > 7)
        {
          tid = 0;
        }
    }

  ForwardDown (packet, from, to, tid, moreData, pushFront);
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to, uint8_t tid, bool moreData, bool pushFront)
{
  NS_LOG_FUNCTION (this << packet << from << to << +tid);
  WifiMacHeader hdr;

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
      //Transmission of multiple frames in the same Polled TXOP is not supported for now
      hdr.SetQosTxopLimit (0);
      //Fill in the QoS control field in the MAC header
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
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (from);
  hdr.SetDsFrom ();
  hdr.SetDsNotTo ();

  if (moreData)
  {
    hdr.SetMoreData();
  }
  
  if (GetQosSupported ())
    {
      //Sanity check that the TID is valid
      NS_ASSERT (tid < 8);
      if (pushFront)
      {
        m_edca[QosUtilsMapTidToAc (tid)]->QueueFront (packet, hdr);
      }
      else
      {
        m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
      }
      
    }
  else
    {
      if (pushFront)
      {
        m_txop->QueueFront (packet, hdr);
      }
      else
      {
        m_txop->Queue (packet, hdr);
      }
      
    }
}

uint32_t 
ApWifiMac::MoveAllPacketsToPsBufferByAddress (Mac48Address address)
{
  NS_LOG_FUNCTION (this << address);
  uint32_t nPacketsMoved = 0;
  for (uint8_t tid = 0; tid < 8; tid++)
  {
    ns3::Ptr<const ns3::WifiMacQueueItem> queueItem = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->PeekByAddress (address);
    while (queueItem != 0)
    {
      // If queueItem is in flight, peek next item
      if (queueItem->IsInFlight())
      {
        NS_LOG_DEBUG ("Packet is in flight. Moving to next packet");
        queueItem = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->PeekByAddress (address, queueItem);
      }
      else if (queueItem->GetHeader().IsRetry())
      {
        NS_LOG_DEBUG ("Packet is a retry packet. Discarding packet. Moving to next packet");
        GetWifiRemoteStationManager ()->ReportFinalDataFailed (queueItem);
        GetFrameExchangeManager()->NotifyPacketDiscarded (queueItem);
        ns3::Ptr<const ns3::WifiMacQueueItem> queueItem_next = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->PeekByAddress (address, queueItem);
        m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->DequeueIfQueued(queueItem);
        
        queueItem = queueItem_next;
        
      }
      else
      {
        // This mpdu is not in flight, move it to PS buffer
        NS_LOG_DEBUG ("Packet is not in flight. Moving to PS buffer ");
        NS_LOG_DEBUG ("Sequence number of packet: "<<queueItem->GetHeader().GetSequenceNumber());
        // make a duplicate of queueItem
        ns3::Ptr<ns3::WifiMacQueueItem> queueItem_dup = Create<WifiMacQueueItem> (queueItem->GetPacket(), queueItem->GetHeader());
        m_psUnicastBuffer->Enqueue (queueItem_dup);
        nPacketsMoved++;
        ns3::Ptr<const ns3::WifiMacQueueItem> queueItem_next = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->PeekByAddress (address, queueItem);
        m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->DequeueIfQueued(queueItem);
        queueItem = queueItem_next;
      }
    }
  //   while (m_edca[QosUtilsMapTidToAc (tid)]->PeekNextMpdu(QosUtilsMapTidToAc (tid), address) != 0)
  //   {
  //     NS_LOG_DEBUG ("Moving packet from AC "<< +tid <<" to PS buffer for STA: "<<address);
  //     // Dequeue only if the packet is not in flight

  //     ns3::Ptr<ns3::WifiMacQueueItem> queueItem = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->PeekByAddress (address);
  //     bool isInFlight = queueItem->IsInFlight();

  //     // ns3::Ptr<ns3::WifiMacQueueItem> queueItem = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->DequeueByAddress (address);
  //     // ns3::Ptr<ns3::WifiMacQueueItem> queueItem = m_edca[QosUtilsMapTidToAc (tid)]->GetWifiMacQueue()->
      
  //     NS_LOG_DEBUG ("isInFlight = "<<isInFlight);
  //     m_psUnicastBuffer->Enqueue (queueItem);
  //     nPacketsMoved++;
  //   }
  }
  NS_LOG_DEBUG ("For STA :"<<address<<", nPacketsMoved = "<<nPacketsMoved);
  return nPacketsMoved;
}

void
ApWifiMac::Enqueue (Ptr<Packet> packet, Mac48Address to, Mac48Address from)
{
  NS_LOG_FUNCTION (this << packet << to << from);

  NS_LOG_FUNCTION ("m_stationManager->GetPSM (to) for "<<to<<"  is "<<m_stationManager->GetPSM (to));
  NS_LOG_FUNCTION ("m_stationManager->GetTwtAgreement (to) for "<<to<<"  is "<<m_stationManager->GetTwtAgreement (to));
  NS_LOG_FUNCTION ("m_stationManager->GetTwtAwakeForSp (to) for "<<to<<" is "<<m_stationManager->GetTwtAwakeForSp (to));

  // If this STA is in PS or if it has a TWT agreement, put it into PS buffer
  if (
      ( m_stationManager->IsAssociated (to) && m_stationManager->GetPSM (to) )|| 
      ( m_stationManager->IsAssociated (to) && m_stationManager->GetTwtAgreement (to) && !(m_stationManager->GetTwtAwakeForSp (to)) )
      // ( m_stationManager->IsAssociated (to) && m_stationManager->GetTwtAgreement (to) )
    )
  {
    NS_LOG_DEBUG ("AP received a packet to send to an inactive STA in PSM/TWT. MAC of STA: "<< to <<"; Received from: "<<from);
    // Create a wifi mac queue item from this packet
    WifiMacHeader header;
    header.SetType (WIFI_MAC_QOSDATA);
    header.SetQosTid (0);
    header.SetAddr1 (to);
    header.SetAddr2 (GetAddress ());
    header.SetAddr3 (from);

    Ptr<WifiMacQueueItem> PsQueueItem = Create<WifiMacQueueItem> (packet, header);
    m_psUnicastBuffer->Enqueue (PsQueueItem);
  
  
    // Ptr<WifiMacQueueItem> PsQueueItem2 = Create<WifiMacQueueItem> (packet, header);         // duplicate to test
    // m_psUnicastBuffer->Enqueue (PsQueueItem2);    
    // Ptr<WifiMacQueueItem> PsQueueItem3 = Create<WifiMacQueueItem> (packet, header);       // duplicate to test
    // m_psUnicastBuffer->Enqueue (PsQueueItem3);
  
  
    NS_LOG_DEBUG ("Packet received for PS STA added to m_psUnicastBuffer. nPackets of m_psUnicastBuffer: "<<m_psUnicastBuffer->GetNPackets ());
    // NS_LOG_DEBUG ("Max delay of m_psUnicastBuffer is "<< (m_psUnicastBuffer->GetMaxDelay()).GetMilliSeconds()<<" milliseconds" );
    // NS_LOG_DEBUG ("Max Size of m_psUnicastBuffer is "<< (m_psUnicastBuffer->GetMaxSize ()).GetValue() << " packets" );
    NS_LOG_DEBUG ("Returning without sending packet. Packet is safe in the m_psUnicastBuffer.");
    return;

  }

  NS_LOG_DEBUG ("Packet received: to.IsGroup () = "<< to.IsGroup ());
  NS_LOG_DEBUG ("m_stationManager->GetNStaInPs() = "<< m_stationManager->GetNStaInPs());
  if (to.IsGroup ()  && ( m_stationManager->GetNStaInPs() > 0 ))
  {
    // Received a group addressed packet and there is at least 1 STA in PS
    NS_LOG_DEBUG ("AP received a group addressed packet and there is at least 1 STA in PS. To address of Packet: "<< to);
    // Create a wifi mac queue item from this packet
    WifiMacHeader header;
    header.SetType (WIFI_MAC_QOSDATA);
    header.SetQosTid (0);
    header.SetAddr1 (to);
    header.SetAddr2 (GetAddress ());
    header.SetAddr3 (from);

    Ptr<WifiMacQueueItem> PsQueueItem = Create<WifiMacQueueItem> (packet, header);
    m_psMulticastBuffer->Enqueue (PsQueueItem);
    
    // Ptr<WifiMacQueueItem> PsQueueItem_temp = Create<WifiMacQueueItem> (packet, header);
    // m_psMulticastBuffer->Enqueue (PsQueueItem_temp);     // Remove - shyam - for testing - duplicate multicast
    // Ptr<WifiMacQueueItem> PsQueueItem_temp2 = Create<WifiMacQueueItem> (packet, header);
    // m_psMulticastBuffer->Enqueue (PsQueueItem_temp2);     // Remove - shyam - for testing - duplicate multicast

    NS_LOG_DEBUG ("Group addressed packet received while there are PS STAs and added to PsBuffer. nPackets of m_psMulticastBuffer: "<<m_psMulticastBuffer->GetNPackets ());
    NS_LOG_DEBUG ("Max delay of Buffer is "<< (m_psMulticastBuffer->GetMaxDelay()).GetMilliSeconds()<<" milliseconds" );
    NS_LOG_DEBUG ("Max Size of buffer is "<< (m_psMulticastBuffer->GetMaxSize ()).GetValue() << " packets" );
    NS_LOG_DEBUG ("Returning without sending packet. Packet is safe in the PS buffer.");
    return;
  }

  if (to.IsGroup () || m_stationManager->IsAssociated (to))
    {
      // If it was a group addressed packet, it is forwarded down only if there are no PS stations. Else, it goes into m_psMulticastBuffer
      ForwardDown (packet, from, to);
    }
  else
    {
      NotifyTxDrop (packet);
    }
}

void
ApWifiMac::Enqueue (Ptr<Packet> packet, Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << to);
  //We're sending this packet with a from address that is our own. We
  //get that address from the lower MAC and make use of the
  //from-spoofing Enqueue() method to avoid duplicated code.
  Enqueue (packet, to, GetAddress ());
}

bool
ApWifiMac::SupportsSendFrom (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

SupportedRates
ApWifiMac::GetSupportedRates (void) const
{
  NS_LOG_FUNCTION (this);
  SupportedRates rates;
  //Send the set of supported rates and make sure that we indicate
  //the Basic Rate set in this set of supported rates.
  for (const auto & mode : m_phy->GetModeList ())
    {
      uint64_t modeDataRate = mode.GetDataRate (m_phy->GetChannelWidth ());
      NS_LOG_DEBUG ("Adding supported rate of " << modeDataRate);
      rates.AddSupportedRate (modeDataRate);
      //Add rates that are part of the BSSBasicRateSet (manufacturer dependent!)
      //here we choose to add the mandatory rates to the BSSBasicRateSet,
      //except for 802.11b where we assume that only the non HR-DSSS rates are part of the BSSBasicRateSet
      if (mode.IsMandatory () && (mode.GetModulationClass () != WIFI_MOD_CLASS_HR_DSSS))
        {
          NS_LOG_DEBUG ("Adding basic mode " << mode.GetUniqueName ());
          m_stationManager->AddBasicMode (mode);
        }
    }
  //set the basic rates
  for (uint8_t j = 0; j < m_stationManager->GetNBasicModes (); j++)
    {
      WifiMode mode = m_stationManager->GetBasicMode (j);
      uint64_t modeDataRate = mode.GetDataRate (m_phy->GetChannelWidth ());
      NS_LOG_DEBUG ("Setting basic rate " << mode.GetUniqueName ());
      rates.SetBasicRate (modeDataRate);
    }
  //If it is a HT AP, then add the BSSMembershipSelectorSet
  //The standard says that the BSSMembershipSelectorSet
  //must have its MSB set to 1 (must be treated as a Basic Rate)
  //Also the standard mentioned that at least 1 element should be included in the SupportedRates the rest can be in the ExtendedSupportedRates
  if (GetHtSupported ())
    {
      for (const auto & selector : m_phy->GetBssMembershipSelectorList ())
        {
          rates.AddBssMembershipSelectorRate (selector);
        }
    }
  return rates;
}

DsssParameterSet
ApWifiMac::GetDsssParameterSet (void) const
{
  NS_LOG_FUNCTION (this);
  DsssParameterSet dsssParameters;
  if (GetDsssSupported ())
    {
      dsssParameters.SetDsssSupported (1);
      dsssParameters.SetCurrentChannel (m_phy->GetChannelNumber ());
    }
  return dsssParameters;
}

Tim ApWifiMac::GetTim (void) const
{
  NS_LOG_FUNCTION (this);

  // shyam - add code to check multicast buffer and set bitmap control - Even if an STA came out of PS, multicast PS buffer may have some packets since it is not forwarded till DTIM beacon

  // NOTE: Multicast bit set to 1 if there are packets in that buffer. Checking if dtim beacon is done in TIM class

  bool isMulticastBuffered = (m_dtimCount == 0 ) && ( m_psMulticastBuffer->GetNBytes() > 0 );    
  // If there are no PS stations or if the unicast buffer m_psUnicastBuffer is empty or if APSD is enabled -- #shyam it is assumed that all data is BE class
  // if (m_stationManager->GetNStaInPs() == 0 || m_psUnicastBuffer->IsEmpty())
  NS_LOG_DEBUG ("For AP, m_psmOrApsd = "<<m_psmOrApsd);
  if (m_stationManager->GetNStaInPs() == 0 || m_psUnicastBuffer->IsEmpty() || m_psmOrApsd)
  {
    Tim timElement (m_dtimCount, m_dtimPeriod, isMulticastBuffered);
    return timElement;
  }

  // There are PS stations
  uint8_t fullAidBitmap [251] = {0};    // All entries set to zero
  uint16_t minAidInBuffer = 2007;      // This is the minimum AID of an STA with packets in m_psUnicastBuffer
  uint16_t maxAidInBuffer = 0;      // This is the maximum AID of an STA with packets in m_psUnicastBuffer
  uint8_t tempMask = 0;
  for (const auto& sta : m_staList)
  {
    if (m_psUnicastBuffer->GetNPacketsByAddress (sta.second) > 0)
    {
      // There are buffered packets for this STA
      // Throw assert if this STA is not is PS
      NS_ASSERT (m_stationManager->GetPSM (sta.second) == true);
      if (sta.first <= minAidInBuffer)
      {
        minAidInBuffer = sta.first;
      }
      if (sta.first >= maxAidInBuffer)
      {
        maxAidInBuffer = sta.first;
      }
    
    // Setting the correct bit in fullAidBitmap
    tempMask = 1 << sta.first % 8;
    fullAidBitmap[int(sta.first/8)] |= tempMask;

    // NS_LOG_DEBUG ("Unicase PS Buffered packets found for STA with AID: "<<sta.first);
    // NS_LOG_DEBUG ("Full AID map is set at byte number: "<< unsigned(sta.first/8) <<" with bit mask: "<<std::bitset<8>(tempMask) );
    // NS_LOG_DEBUG ("Full AID map at byte number: "<<sta.first/8 <<" is now: "<<std::bitset<8>( fullAidBitmap[int(sta.first/8)] ));



    }
    
  }
  // NS_LOG_DEBUG ("minAidInBuffer is "<<minAidInBuffer);
  // NS_LOG_DEBUG ("maxAidInBuffer is "<<maxAidInBuffer);
 

  Tim timElement (m_dtimCount, m_dtimPeriod, isMulticastBuffered, fullAidBitmap, minAidInBuffer, maxAidInBuffer);
  return timElement;

}

CapabilityInformation
ApWifiMac::GetCapabilities (void) const
{
  NS_LOG_FUNCTION (this);
  CapabilityInformation capabilities;
  capabilities.SetShortPreamble (m_shortPreambleEnabled);
  capabilities.SetShortSlotTime (m_shortSlotTimeEnabled);
  capabilities.SetEss ();
  return capabilities;
}

ErpInformation
ApWifiMac::GetErpInformation (void) const
{
  NS_LOG_FUNCTION (this);
  ErpInformation information;
  information.SetErpSupported (1);
  if (GetErpSupported ())
    {
      information.SetNonErpPresent (m_numNonErpStations > 0);
      information.SetUseProtection (GetUseNonErpProtection ());
      if (m_shortPreambleEnabled)
        {
          information.SetBarkerPreambleMode (0);
        }
      else
        {
          information.SetBarkerPreambleMode (1);
        }
    }
  return information;
}

EdcaParameterSet
ApWifiMac::GetEdcaParameterSet (void) const
{
  NS_LOG_FUNCTION (this);
  EdcaParameterSet edcaParameters;
  if (GetQosSupported ())
    {
      edcaParameters.SetQosSupported (1);
      Ptr<QosTxop> edca;
      Time txopLimit;

      edca = m_edca.find (AC_BE)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetBeAci (0);
      edcaParameters.SetBeCWmin (edca->GetMinCw ());
      edcaParameters.SetBeCWmax (edca->GetMaxCw ());
      edcaParameters.SetBeAifsn (edca->GetAifsn ());
      edcaParameters.SetBeTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_BK)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetBkAci (1);
      edcaParameters.SetBkCWmin (edca->GetMinCw ());
      edcaParameters.SetBkCWmax (edca->GetMaxCw ());
      edcaParameters.SetBkAifsn (edca->GetAifsn ());
      edcaParameters.SetBkTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_VI)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetViAci (2);
      edcaParameters.SetViCWmin (edca->GetMinCw ());
      edcaParameters.SetViCWmax (edca->GetMaxCw ());
      edcaParameters.SetViAifsn (edca->GetAifsn ());
      edcaParameters.SetViTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_VO)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetVoAci (3);
      edcaParameters.SetVoCWmin (edca->GetMinCw ());
      edcaParameters.SetVoCWmax (edca->GetMaxCw ());
      edcaParameters.SetVoAifsn (edca->GetAifsn ());
      edcaParameters.SetVoTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edcaParameters.SetQosInfo (0);
    }
  return edcaParameters;
}

MuEdcaParameterSet
ApWifiMac::GetMuEdcaParameterSet (void) const
{
  NS_LOG_FUNCTION (this);
  MuEdcaParameterSet muEdcaParameters;
  if (GetHeSupported ())
    {
      Ptr<HeConfiguration> heConfiguration = GetHeConfiguration ();
      NS_ASSERT (heConfiguration != 0);

      muEdcaParameters.SetQosInfo (0);

      UintegerValue uintegerValue;
      TimeValue timeValue;

      heConfiguration->GetAttribute ("MuBeAifsn", uintegerValue);
      muEdcaParameters.SetMuAifsn (AC_BE, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuBeCwMin", uintegerValue);
      muEdcaParameters.SetMuCwMin (AC_BE, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuBeCwMax", uintegerValue);
      muEdcaParameters.SetMuCwMax (AC_BE, uintegerValue.Get ());
      heConfiguration->GetAttribute ("BeMuEdcaTimer", timeValue);
      muEdcaParameters.SetMuEdcaTimer (AC_BE, timeValue.Get ());

      heConfiguration->GetAttribute ("MuBkAifsn", uintegerValue);
      muEdcaParameters.SetMuAifsn (AC_BK, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuBkCwMin", uintegerValue);
      muEdcaParameters.SetMuCwMin (AC_BK, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuBkCwMax", uintegerValue);
      muEdcaParameters.SetMuCwMax (AC_BK, uintegerValue.Get ());
      heConfiguration->GetAttribute ("BkMuEdcaTimer", timeValue);
      muEdcaParameters.SetMuEdcaTimer (AC_BK, timeValue.Get ());

      heConfiguration->GetAttribute ("MuViAifsn", uintegerValue);
      muEdcaParameters.SetMuAifsn (AC_VI, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuViCwMin", uintegerValue);
      muEdcaParameters.SetMuCwMin (AC_VI, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuViCwMax", uintegerValue);
      muEdcaParameters.SetMuCwMax (AC_VI, uintegerValue.Get ());
      heConfiguration->GetAttribute ("ViMuEdcaTimer", timeValue);
      muEdcaParameters.SetMuEdcaTimer (AC_VI, timeValue.Get ());

      heConfiguration->GetAttribute ("MuVoAifsn", uintegerValue);
      muEdcaParameters.SetMuAifsn (AC_VO, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuVoCwMin", uintegerValue);
      muEdcaParameters.SetMuCwMin (AC_VO, uintegerValue.Get ());
      heConfiguration->GetAttribute ("MuVoCwMax", uintegerValue);
      muEdcaParameters.SetMuCwMax (AC_VO, uintegerValue.Get ());
      heConfiguration->GetAttribute ("VoMuEdcaTimer", timeValue);
      muEdcaParameters.SetMuEdcaTimer (AC_VO, timeValue.Get ());
    }
  return muEdcaParameters;
}

HtOperation
ApWifiMac::GetHtOperation (void) const
{
  NS_LOG_FUNCTION (this);
  HtOperation operation;
  if (GetHtSupported ())
    {
      operation.SetHtSupported (1);
      operation.SetPrimaryChannel (m_phy->GetChannelNumber ());
      operation.SetRifsMode (false);
      operation.SetNonGfHtStasPresent (true);
      if (m_phy->GetChannelWidth () > 20)
        {
          operation.SetSecondaryChannelOffset (1);
          operation.SetStaChannelWidth (1);
        }
      if (m_numNonHtStations == 0)
        {
          operation.SetHtProtection (NO_PROTECTION);
        }
      else
        {
          operation.SetHtProtection (MIXED_MODE_PROTECTION);
        }
      uint64_t maxSupportedRate = 0; //in bit/s
      for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HT))
        {
          uint8_t nss = (mcs.GetMcsValue () / 8) + 1;
          NS_ASSERT (nss > 0 && nss < 5);
          uint64_t dataRate = mcs.GetDataRate (m_phy->GetChannelWidth (), GetHtConfiguration ()->GetShortGuardIntervalSupported () ? 400 : 800, nss);
          if (dataRate > maxSupportedRate)
            {
              maxSupportedRate = dataRate;
              NS_LOG_DEBUG ("Updating maxSupportedRate to " << maxSupportedRate);
            }
        }
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedTxSpatialStreams ();
      auto mcsList = m_phy->GetMcsList (WIFI_MOD_CLASS_HT);
      uint8_t nMcs = mcsList.size ();
      for (const auto& sta : m_staList)
        {
          if (m_stationManager->GetHtSupported (sta.second))
            {
              uint64_t maxSupportedRateByHtSta = 0; //in bit/s
              auto itMcs = mcsList.begin ();
              for (uint8_t j = 0; j < (std::min (nMcs, m_stationManager->GetNMcsSupported (sta.second))); j++)
                {
                  WifiMode mcs = *itMcs++;
                  uint8_t nss = (mcs.GetMcsValue () / 8) + 1;
                  NS_ASSERT (nss > 0 && nss < 5);
                  uint64_t dataRate = mcs.GetDataRate (m_stationManager->GetChannelWidthSupported (sta.second),
                                                       m_stationManager->GetShortGuardIntervalSupported (sta.second) ? 400 : 800, nss);
                  if (dataRate > maxSupportedRateByHtSta)
                    {
                      maxSupportedRateByHtSta = dataRate;
                    }
                }
              if (maxSupportedRateByHtSta < maxSupportedRate)
                {
                  maxSupportedRate = maxSupportedRateByHtSta;
                }
              if (m_stationManager->GetNMcsSupported (sta.second) < nMcs)
                {
                  nMcs = m_stationManager->GetNMcsSupported (sta.second);
                }
              if (m_stationManager->GetNumberOfSupportedStreams (sta.second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (sta.second);
                }
            }
        }
      operation.SetRxHighestSupportedDataRate (static_cast<uint16_t> (maxSupportedRate / 1e6)); //in Mbit/s
      operation.SetTxMcsSetDefined (nMcs > 0);
      operation.SetTxMaxNSpatialStreams (maxSpatialStream);
      //To be filled in once supported
      operation.SetObssNonHtStasPresent (0);
      operation.SetDualBeacon (0);
      operation.SetDualCtsProtection (0);
      operation.SetStbcBeacon (0);
      operation.SetLSigTxopProtectionFullSupport (0);
      operation.SetPcoActive (0);
      operation.SetPhase (0);
      operation.SetRxMcsBitmask (0);
      operation.SetTxRxMcsSetUnequal (0);
      operation.SetTxUnequalModulation (0);
    }
  return operation;
}

VhtOperation
ApWifiMac::GetVhtOperation (void) const
{
  NS_LOG_FUNCTION (this);
  VhtOperation operation;
  if (GetVhtSupported ())
    {
      operation.SetVhtSupported (1);
      uint16_t channelWidth = GetVhtOperationalChannelWidth ();
      if (channelWidth == 160)
        {
          operation.SetChannelWidth (2);
        }
      else if (channelWidth == 80)
        {
          operation.SetChannelWidth (1);
        }
      else
        {
          operation.SetChannelWidth (0);
        }
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedRxSpatialStreams ();
      for (const auto& sta : m_staList)
        {
          if (m_stationManager->GetVhtSupported (sta.second))
            {
              if (m_stationManager->GetNumberOfSupportedStreams (sta.second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (sta.second);
                }
            }
        }
      for (uint8_t nss = 1; nss <= maxSpatialStream; nss++)
        {
          uint8_t maxMcs = 9; //TBD: hardcode to 9 for now since we assume all MCS values are supported
          operation.SetMaxVhtMcsPerNss (nss, maxMcs);
        }
    }
  return operation;
}

HeOperation
ApWifiMac::GetHeOperation (void) const
{
  NS_LOG_FUNCTION (this);
  HeOperation operation;
  if (GetHeSupported ())
    {
      operation.SetHeSupported (1);
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedRxSpatialStreams ();
      for (const auto& sta : m_staList)
        {
          if (m_stationManager->GetHeSupported (sta.second))
            {
              if (m_stationManager->GetNumberOfSupportedStreams (sta.second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (sta.second);
                }
            }
        }
      for (uint8_t nss = 1; nss <= maxSpatialStream; nss++)
        {
          operation.SetMaxHeMcsPerNss (nss, 11); //TBD: hardcode to 11 for now since we assume all MCS values are supported
        }
      operation.SetBssColor (GetHeConfiguration ()->GetBssColor ());
    }
  return operation;
}

void
ApWifiMac::SendProbeResp (Mac48Address to)
{
  NS_LOG_FUNCTION (this << to);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_PROBE_RESPONSE);
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtProbeResponseHeader probe;
  probe.SetSsid (GetSsid ());
  probe.SetSupportedRates (GetSupportedRates ());
  probe.SetBeaconIntervalUs (GetBeaconInterval ().GetMicroSeconds ());
  probe.SetCapabilities (GetCapabilities ());
  m_stationManager->SetShortPreambleEnabled (m_shortPreambleEnabled);
  m_stationManager->SetShortSlotTimeEnabled (m_shortSlotTimeEnabled);
  if (GetDsssSupported ())
    {
      probe.SetDsssParameterSet (GetDsssParameterSet ());
    }
  if (GetErpSupported ())
    {
      probe.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      probe.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      probe.SetExtendedCapabilities (GetExtendedCapabilities ());
      probe.SetHtCapabilities (GetHtCapabilities ());
      probe.SetHtOperation (GetHtOperation ());
    }
  if (GetVhtSupported ())
    {
      probe.SetVhtCapabilities (GetVhtCapabilities ());
      probe.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      probe.SetHeCapabilities (GetHeCapabilities ());
      probe.SetHeOperation (GetHeOperation ());
      probe.SetMuEdcaParameterSet (GetMuEdcaParameterSet ());
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
  else if (!m_stationManager->GetQosSupported (to))
    {
      GetBEQueue ()->Queue (packet, hdr);
    }
  else
    {
      GetVOQueue ()->Queue (packet, hdr);
    }
}

void
ApWifiMac::SendAssocResp (Mac48Address to, bool success, bool isReassoc)
{
  NS_LOG_FUNCTION (this << to << success << isReassoc);
  WifiMacHeader hdr;
  hdr.SetType (isReassoc ? WIFI_MAC_MGT_REASSOCIATION_RESPONSE : WIFI_MAC_MGT_ASSOCIATION_RESPONSE);
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtAssocResponseHeader assoc;
  StatusCode code;
  if (success)
    {
      code.SetSuccess ();
      uint16_t aid = 0;
      bool found = false;
      if (isReassoc)
        {
          for (const auto& sta : m_staList)
            {
              if (sta.second == to)
                {
                  aid = sta.first;
                  found = true;
                  break;
                }
            }
        }
      if (!found)
        {
          aid = GetNextAssociationId ();
          m_staList.insert (std::make_pair (aid, to));
          m_assocLogger (aid, to);
          m_stationManager->SetAssociationId (to, aid);
          if (m_stationManager->GetDsssSupported (to) && !m_stationManager->GetErpOfdmSupported (to))
            {
              m_numNonErpStations++;
            }
          if (!m_stationManager->GetHtSupported (to))
            {
              m_numNonHtStations++;
            }
          UpdateShortSlotTimeEnabled ();
          UpdateShortPreambleEnabled ();
        }
      assoc.SetAssociationId (aid);
    }
  else
    {
      code.SetFailure ();
    }
  assoc.SetSupportedRates (GetSupportedRates ());
  assoc.SetStatusCode (code);
  assoc.SetCapabilities (GetCapabilities ());
  if (GetErpSupported ())
    {
      assoc.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      assoc.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      assoc.SetExtendedCapabilities (GetExtendedCapabilities ());
      assoc.SetHtCapabilities (GetHtCapabilities ());
      assoc.SetHtOperation (GetHtOperation ());
    }
  if (GetVhtSupported ())
    {
      assoc.SetVhtCapabilities (GetVhtCapabilities ());
      assoc.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      assoc.SetHeCapabilities (GetHeCapabilities ());
      assoc.SetHeOperation (GetHeOperation ());
      assoc.SetMuEdcaParameterSet (GetMuEdcaParameterSet ());
    }
  packet->AddHeader (assoc);

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
  else if (!m_stationManager->GetQosSupported (to))
    {
      GetBEQueue ()->Queue (packet, hdr);
    }
  else
    {
      GetVOQueue ()->Queue (packet, hdr);
    }
}

void
ApWifiMac::SendOneBeacon (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("m_dtimPeriod is "<<unsigned (m_dtimPeriod));
  NS_LOG_DEBUG ("Number of stations in PS is "<<m_stationManager->GetNStaInPs());
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_BEACON);
  hdr.SetAddr1 (Mac48Address::GetBroadcast ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtBeaconHeader beacon;
  beacon.SetSsid (GetSsid ());
  beacon.SetSupportedRates (GetSupportedRates ());
  beacon.SetBeaconIntervalUs (GetBeaconInterval ().GetMicroSeconds ());
  beacon.SetCapabilities (GetCapabilities ());
  beacon.SetTim (GetTim ());
  m_stationManager->SetShortPreambleEnabled (m_shortPreambleEnabled);
  m_stationManager->SetShortSlotTimeEnabled (m_shortSlotTimeEnabled);
  
  if (GetDsssSupported ())
    {
      beacon.SetDsssParameterSet (GetDsssParameterSet ());
    }
  if (GetErpSupported ())
    {
      beacon.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      beacon.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      beacon.SetExtendedCapabilities (GetExtendedCapabilities ());
      beacon.SetHtCapabilities (GetHtCapabilities ());
      beacon.SetHtOperation (GetHtOperation ());
    }
  if (GetVhtSupported ())
    {
      beacon.SetVhtCapabilities (GetVhtCapabilities ());
      beacon.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      beacon.SetHeCapabilities (GetHeCapabilities ());
      beacon.SetHeOperation (GetHeOperation ());
      beacon.SetMuEdcaParameterSet (GetMuEdcaParameterSet ());
    }
  packet->AddHeader (beacon);

  //The beacon has it's own special queue, so we load it in there
  m_beaconTxop->Queue (packet, hdr);
  m_beaconEvent = Simulator::Schedule (GetBeaconInterval (), &ApWifiMac::SendOneBeacon, this);
  // NS_LOG_DEBUG ("Timestamp for beacon is "<<beacon.GetTimestamp());
  // NS_LOG_DEBUG ("m_beaconEvent is scheduled for tstamp = "<<m_beaconEvent.GetTs());

  // If this is a DTIM beacon and if there are PS buffered multicast packets, send them all
  if (m_dtimCount == 0 && m_psMulticastBuffer->GetNPackets() > 0)
  {
    NS_LOG_DEBUG ("That was a DTIM beacon and AP has PS buffered multicast packets. Sending all packets after a delay. #bug - shyam - need to send this automatically after knowing that the beacon is sent.");
    Simulator::Schedule (MicroSeconds (100), &ApWifiMac::SendAllMulticastPsPackets, this);
    
  }

  uint8_t newDtimCount = (m_dtimCount + 1)%m_dtimPeriod;
  NS_LOG_DEBUG ("Next beacon scheduled. DTIM count is updated from "<< unsigned(m_dtimCount)<< " to " << unsigned(newDtimCount));
  m_dtimCount = newDtimCount;
  //If a STA that does not support Short Slot Time associates,
  //the AP shall use long slot time beginning at the first Beacon
  //subsequent to the association of the long slot time STA.
  if (GetErpSupported ())
    {
      if (m_shortSlotTimeEnabled)
        {
          //Enable short slot time
          m_phy->SetSlot (MicroSeconds (9));
        }
      else
        {
          //Disable short slot time
          m_phy->SetSlot (MicroSeconds (20));
        }
    }
}

void
ApWifiMac::TxOk (Ptr<const WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  const WifiMacHeader& hdr = mpdu->GetHeader ();
  if ((hdr.IsAssocResp () || hdr.IsReassocResp ())
      && m_stationManager->IsWaitAssocTxOk (hdr.GetAddr1 ()))
    {
      NS_LOG_DEBUG ("associated with sta=" << hdr.GetAddr1 ());
      m_stationManager->RecordGotAssocTxOk (hdr.GetAddr1 ());
    }
}

void
ApWifiMac::TxFailed (WifiMacDropReason timeoutReason, Ptr<const WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << +timeoutReason << *mpdu);
  const WifiMacHeader& hdr = mpdu->GetHeader ();

  if ((hdr.IsAssocResp () || hdr.IsReassocResp ())
      && m_stationManager->IsWaitAssocTxOk (hdr.GetAddr1 ()))
    {
      NS_LOG_DEBUG ("association failed with sta=" << hdr.GetAddr1 ());
      m_stationManager->RecordGotAssocTxFailed (hdr.GetAddr1 ());
    }
}

void 
ApWifiMac::SendAllUnicastPsPacketsByAddress (Mac48Address address)
{
  NS_LOG_FUNCTION(this << address);

   


  // Check if there are buffered packets
  uint32_t numPacketsForSta;
  numPacketsForSta = m_psUnicastBuffer->GetNPacketsByAddress (address);

  if (numPacketsForSta == 0)
  {
    NS_LOG_DEBUG ("No PS buffered packets to forward to STA with MAC "<<address);
    if (m_stationManager->GetPSM (address) && m_psmOrApsd)
    {
      NS_LOG_DEBUG ("This STA has APSD and has no buffered packets - sending Null Qos Frame with EoSP = 1");
      NS_ASSERT (GetQosSupported());
    //   uint8_t tid = 0;
      // WifiMacHeader hdr;
      Ptr<Packet> packet = Create<Packet> (10);

      // hdr.SetType (WIFI_MAC_QOSDATA);
    //   hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
    //   // hdr.SetQosAckPolicy (WifiMacHeader::BLOCK_ACK);
    //   // hdr.SetQosNoAmsdu ();
    //   // hdr.SetQosTxopLimit (0);
      // hdr.SetQosTid (0);    // Sending with TID = 0
      // hdr.SetAddr1 (address);
      // hdr.SetAddr2 (GetAddress ());
      // hdr.SetAddr3 (GetAddress ());
      // Ptr<WifiMacQueueItem> queueItem = Create<WifiMacQueueItem> (packet, hdr);
      // Ptr<Packet> packet2 = queueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet

      ForwardDown (packet, GetAddress (), address);
    //   hdr.SetDsNotFrom ();
    //   // hdr.SetDsFrom ();
    //   hdr.SetDsNotTo ();
    //   hdr.SetNoMoreData();  

    //   hdr.SetQosEosp();

    //   m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
    }
    return;
  }
  Ptr<WifiMacQueueItem> PsQueueItem;
  for (uint32_t i = 0; i < numPacketsForSta; i++)
  {
    NS_LOG_DEBUG ("Sending Packet #"<<(i+1)<<" of #"<<numPacketsForSta <<" for STA with MAC "<<address);
    PsQueueItem = m_psUnicastBuffer->DequeueByAddress (address);
    Ptr<Packet> packet = PsQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet

    ForwardDown (packet, PsQueueItem->GetHeader().GetAddr3(), PsQueueItem->GetHeader().GetAddr1());
  }
  return;

}

bool 
ApWifiMac::IsEospQosFrameQueuedForSta (Mac48Address address)
{
  NS_LOG_FUNCTION (this<<address);
  ns3::Ptr<const ns3::WifiMacQueueItem> nextMpdu =  GetBEQueue ()->PeekNextMpdu (0, address);
  bool foundEospQosFrame = false;
  while (nextMpdu != nullptr)
  {
    if (nextMpdu->GetHeader().IsQosData())
    {
      NS_LOG_DEBUG ("Found Qos data frame for STA "<<address);
      if (nextMpdu->GetHeader().IsQosEosp())
      {
        NS_LOG_DEBUG ("Found EOSP = 1 Qos data frame for STA "<<address);
        foundEospQosFrame = true;
        break;
      }
    }
    nextMpdu =  GetBEQueue ()->PeekNextMpdu (0, address, nextMpdu);

  }
  return foundEospQosFrame;
}


void 
ApWifiMac::SendQoSEospNullByAddress (Mac48Address address)
{
  NS_LOG_FUNCTION(this << address);
  NS_ASSERT (GetQosSupported());
  uint8_t tid = 0;
  WifiMacHeader hdr;
  Ptr<Packet> packet = Create<Packet> ();

  hdr.SetType (WIFI_MAC_QOSDATA_NULL);
  // hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
  hdr.SetQosNoAmsdu ();
  hdr.SetQosTxopLimit (0);
  hdr.SetQosTid (0);    // Sending with TID = 0
  hdr.SetAddr1 (address);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  // hdr.SetDsNotFrom ();
  hdr.SetDsFrom ();
  hdr.SetDsNotTo ();
  hdr.SetNoMoreData();  

  hdr.SetQosEosp();

  m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);

}


void 
ApWifiMac::SendOneUnicastPsPacketByAddressNow (Mac48Address address)
{
  NS_LOG_FUNCTION(this << address);
  // Check if there are buffered packets
  uint32_t numPacketsForSta;
  uint8_t tid = 0;
  numPacketsForSta = m_psUnicastBuffer->GetNPacketsByAddress (address);
  NS_LOG_DEBUG ("numPacketsForSta:"<<numPacketsForSta);
  WifiMacHeader hdr;
  Ptr<Packet> packet = Create<Packet> ();
  if (numPacketsForSta == 0)
  {
    NS_LOG_DEBUG ("No PS buffered unicast packets to forward to STA with MAC "<<address << "Sending null frame with More data = 0 ");
    
    if (GetQosSupported ())
    {
      hdr.SetType (WIFI_MAC_QOSDATA_NULL);
      hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
      hdr.SetQosNoEosp ();
      hdr.SetQosNoAmsdu ();
      hdr.SetQosTxopLimit (0);
      hdr.SetQosTid (0);    // Sending with TID = 0
    }
    else
    {
      hdr.SetType (WIFI_MAC_DATA_NULL);
    }
    hdr.SetAddr1 (address);
    hdr.SetAddr2 (GetAddress ());
    hdr.SetAddr3 (GetAddress ());
    hdr.SetDsNotFrom ();
    hdr.SetDsNotTo ();
    hdr.SetNoMoreData();

    if (!GetQosSupported ())
    {
      m_txop->QueueFront (packet, hdr);
      m_feManager->StartTransmission (m_txop);
    }
    else
    {
      // m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
      m_edca[QosUtilsMapTidToAc (tid)]->QueueFront (packet, hdr);
      m_feManager->StartTransmission (GetBEQueue ());
    }

    return;
  }
  else
  {
    NS_LOG_DEBUG ("There is at least one unicast buffered PS packet for STA with MAC "<<address);
    Ptr<WifiMacQueueItem> PsQueueItem;
    

    PsQueueItem = m_psUnicastBuffer->DequeueByAddress (address);
    Ptr<Packet> packet = PsQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet
    
    bool moreData = false;
    if (numPacketsForSta > 1)
    {
      moreData = true;
    }
  
  
  bool pushFront = true;
  ForwardDown (packet, PsQueueItem->GetHeader().GetAddr3(), PsQueueItem->GetHeader().GetAddr1(), tid, moreData, pushFront);
  m_feManager->StartTransmission (GetBEQueue ());
  
  return;
}

}

void 
ApWifiMac::SendAllMulticastPsPackets ()
{
  NS_LOG_FUNCTION(this);
  // Check if there are buffered packets
  uint32_t numPackets;
  numPackets = m_psMulticastBuffer->GetNPackets();

  if (numPackets == 0)
  {
    NS_LOG_DEBUG ("No PS buffered multicast packets to send");
    return;
  }
  Ptr<WifiMacQueueItem> PsQueueItem;
  for (uint32_t i = 0; i < numPackets; i++)
  {
    NS_LOG_DEBUG ("Sending PS buffered multicast Packet #"<<(i+1)<<" of #"<<numPackets);
    PsQueueItem = m_psMulticastBuffer->Dequeue();
    Ptr<Packet> packet = PsQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet
    bool moreData = ( i < numPackets - 1 );
    
    ForwardDown (packet, PsQueueItem->GetHeader().GetAddr3(), PsQueueItem->GetHeader().GetAddr1(), moreData);
    
    
  }
  return;

}

void
ApWifiMac::Receive (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  const WifiMacHeader* hdr = &mpdu->GetHeader ();
  Ptr<const Packet> packet = mpdu->GetPacket ();
  Mac48Address from = hdr->GetAddr2 ();
  NS_LOG_DEBUG ("From address: "<<from);



  // Update remote station state with Power management information from the header of every received data and mgmt frame
  // if (m_stationManager->GetPSM (from) == true && (hdr->IsData() || hdr->IsMgt())&&  hdr->IsPowerMgt() == false && !(m_stationManager->GetTwtAgreement (from) ) )
  if (m_stationManager->GetPSM (from) == true && hdr->IsData()&&  hdr->IsPowerMgt() == false && !(m_stationManager->GetTwtAgreement (from) ) )
  {
    NS_LOG_DEBUG ("STA "<<from << " switched from PS 1 to 0 and does not have TWT agreement. Sending all unicast. ");
    // Send all unicast PS buffered packets
    SendAllUnicastPsPacketsByAddress (from);
  }

  // If QoS data frame is received from an APSD STA, send all queued frames
  if (m_psmOrApsd && m_stationManager->GetPSM (from) == true && hdr->IsQosData() && !(m_stationManager->GetTwtAgreement (from) ) )
  {
    NS_LOG_DEBUG ("STA "<<from << " with APSD enabled sent a QoS UL frame. ");
    
    if (IsEospQosFrameQueuedForSta (from))
    {
      NS_LOG_DEBUG ("At least one Qos Packet has been queued for this APSD STA with EOSP = 1. Do nothing.");
      
    }
    else
    {
      NS_LOG_DEBUG ("EoSP Qos frame not queued yet. Sending all Unicast Packets.");
      SendAllUnicastPsPacketsByAddress (from);
    }

    
  }

  // IF STA with TWT agreement sends a packet with PS = 1 within TWT SP
  // if (m_stationManager->GetTwtAgreement (from) && hdr->IsPowerMgt() && m_stationManager->GetTwtSpNow (from) )
  //   {
  //     m_stationManager->SetTwtAwakeForSp (from, false);
  //     NS_LOG_DEBUG ("SetTwtAwakeForSp for "<<from<<" changed to false");
  //   }
  
  // IF STA with TWT trigger based agreement sends a packet with more data = 0 within twt SP, set m_expectingBasicTf = 0
  // if (m_stationManager->GetTwtAgreement (from) && m_stationManager->GetTwtSpNow (from) && hdr->IsMoreData() && hdr->IsData())
  //   {
  //     m_stationManager->SetExpectingBasicTf (from, false);
  //     NS_LOG_DEBUG ("SetExpectingBasicTf for "<<from<<" changed to false since a data frame with MoreData = 0 was received within TWT SP");
  //     NS_LOG_DEBUG ("m_nStaExpectingBasicTf is now "<< m_stationManager->GetNStaExpectingTf());

  //   }
  
  // NS_LOG_DEBUG ("m_stationManager->GetTwtAgreement (from) : "<<m_stationManager->GetTwtAgreement (from));
  // NS_LOG_DEBUG ("m_stationManager->GetTwtAnnounced (from) : "<< m_stationManager->GetTwtAnnounced (from));
  NS_LOG_DEBUG ("hdr->IsPowerMgt() : "<< hdr->IsPowerMgt());

  if (m_stationManager->GetTwtAgreement (from) && m_stationManager->GetTwtAnnounced (from) && !(hdr->IsPowerMgt()) )
    {
      // TWT Announced STA sent frame with PM = 0 --> set m_twtAwakeForSp to true
      m_stationManager->SetTwtAwakeForSp (from, true);
      NS_LOG_DEBUG ("Received frame from Announced TWT STA with PM = 0; m_twtAwakeForSp is now : "<< m_stationManager->GetTwtAwakeForSp(from));
    }
  

  if (hdr->IsData()  && !(m_stationManager->GetTwtAgreement (from) ) )      // shyam - check
  {
    if(m_stationManager->GetPSM(from) == false && hdr->IsPowerMgt())
    {
      NS_LOG_DEBUG ("An STA that was not in PSM is switching to PSM. Checking that AP's MAC queue does not have any MPDUs to this STA");
      if (GetQosSupported ())
      {
        
        // NS_ASSERT_MSG (m_edca[AC_BE]->PeekNextMpdu(0, from) == 0, "MAC AC_BE queue at AP was not empty for STA switching from CAM to PSM");
        NS_LOG_DEBUG ("STA switched from CAM to PSM. Moving all AC_BE queued packets at AP for this STA into PS buffer");
        while (m_edca[AC_BE]->PeekNextMpdu(0, from) != 0)
        {
          // m_edca[AC_BE]->GetWifiMacQueue()->DequeueByAddress(from);
          // m_psUnicastBuffer->Enqueue (PsQueueItem);
          // NS_LOG_DEBUG ("Discarding one packet");
          Ptr<WifiMacQueueItem> PsQueueItem = m_edca[AC_BE]->GetWifiMacQueue()->DequeueByAddress(from);
          m_psUnicastBuffer->Enqueue (PsQueueItem);
          NS_LOG_DEBUG ("Moving one packet");
        }
        
      }
      else 
      {
        NS_ABORT_MSG ("Non QoS STA not supported for PSM");
      }
      NS_ASSERT_MSG (m_edca[AC_BE]->PeekNextMpdu(0, from) == 0, "MAC AC_BE queue at AP was not empty for STA switching from CAM to PSM");
    }
    // PSM state at AP is changed based on UL packets only if there is no TWT agreement for that STA
    m_stationManager->SetPSM (hdr->GetAddr2 (), hdr->IsPowerMgt());
  }


  // If received packet is PS-POLL and if there is no TWT agreement, call SendOneUnicastPsPacketByAddressNow
  NS_LOG_DEBUG ("Is this packet PS-POLL? "<<hdr->IsPsPoll());
  if (hdr->IsPsPoll() && !(m_stationManager->GetTwtAgreement (from) ))
  {
    NS_LOG_DEBUG ("Received PS-POLL from "<<from<<" and it does not have TWT agreement.");
    Simulator::Schedule(m_phy->GetSifs(), &ApWifiMac::SendOneUnicastPsPacketByAddressNow, this, from);
    return;
  }
  
  if (hdr->IsPsPoll() && m_stationManager->GetTwtAgreement (from) && m_stationManager->GetTwtAnnounced (from) )
  {
    NS_LOG_DEBUG ("Received PS-POLL from "<<from<<" and it has an announced TWT agreement. Changing m_twtAwakeForSp to true ");
    m_stationManager->SetTwtAwakeForSp (from, true);
    return;
  }



  // If received frame is mgt Action S1G TWT Info frame
  if (hdr->IsMgt () && hdr->IsAction () 
          && hdr->GetAddr1 () == GetAddress ()
          && m_stationManager->IsAssociated (from))
  {
    WifiActionHeader actionHdr;
    Ptr<Packet> packetCopy = packet->Copy();
    packetCopy->RemoveHeader (actionHdr);
    if (actionHdr.GetCategory() == WifiActionHeader::UNPROTECTED_S1G)
    {
      NS_LOG_DEBUG ("UNPROTECTED_S1G received at AP from:"<<from);
      MgtUnprotectedTwtInfoHeader s1gHdr;
      packetCopy->RemoveHeader(s1gHdr);
      u_int64_t nextTwt = s1gHdr.GetNextTwt();
      u_int8_t twtId = s1gHdr.GetTwtFlowId();
      NS_LOG_DEBUG ("nextTwt is "<<nextTwt);
      NS_LOG_DEBUG ("twtId is "<<twtId);
      m_stationManager->UpdateFromTwtInfoFrame (from, twtId, nextTwt);
      return;
    }
  }

  if (hdr->IsData ())
    {
      Mac48Address bssid = hdr->GetAddr1 ();
      if (!hdr->IsFromDs ()
          && hdr->IsToDs ()
          && bssid == GetAddress ()
          && m_stationManager->IsAssociated (from))
        {
          Mac48Address to = hdr->GetAddr3 ();
          NS_LOG_DEBUG ("Is this packet group addressed? "<<to.IsGroup());
          NS_LOG_DEBUG ("m_stationManager->GetNStaInPs(): "<<m_stationManager->GetNStaInPs());
          if (to == GetAddress ())
            {
              NS_LOG_DEBUG ("frame for me from=" << from);
              if (hdr->IsQosData ())
                {
                  if (hdr->IsQosAmsdu ())
                    {
                      NS_LOG_DEBUG ("Received A-MSDU from=" << from << ", size=" << packet->GetSize ());
                      DeaggregateAmsduAndForward (mpdu);
                      packet = 0;
                    }
                  else
                    {
                      ForwardUp (packet, from, bssid);
                    }
                }
              else if (hdr->HasData ())
                {
                  ForwardUp (packet, from, bssid);
                }
            }
          // If it is a unicast packet to a PS STA
          else if (m_stationManager->IsAssociated (to) && m_stationManager->GetPSM (to))
          {
            NS_LOG_DEBUG ("AP received a packet to FORWARD to an STA in PSM. MAC of STA: "<< to);
            // Create a wifi mac queue item from this packet
            WifiMacHeader header;
            header.SetType (WIFI_MAC_QOSDATA);
            header.SetQosTid (0);
            header.SetAddr1 (to);
            header.SetAddr2 (GetAddress ());
            header.SetAddr3 (from);

            Ptr<WifiMacQueueItem> PsQueueItem = Create<WifiMacQueueItem> (packet, header);
            m_psUnicastBuffer->Enqueue (PsQueueItem);

            // Ptr<WifiMacQueueItem> PsQueueItem2 = Create<WifiMacQueueItem> (packet, header);   // Duplicate to test
            // m_psUnicastBuffer->Enqueue (PsQueueItem2);
            // Ptr<WifiMacQueueItem> PsQueueItem3 = Create<WifiMacQueueItem> (packet, header);    // Duplicate to test
            // m_psUnicastBuffer->Enqueue (PsQueueItem3);

            NS_LOG_DEBUG ("Packet received for PS STA added to m_psUnicastBuffer. nPackets of m_psUnicastBuffer: "<<m_psUnicastBuffer->GetNPackets ());
            NS_LOG_DEBUG ("Returning without sending packet. Packet is safe in the m_psUnicastBuffer.");

            ForwardUp (packet, from, to);     // shyam - is this correct?
          }
          // If it is a group addressed packet and there are PS STAs
          else if (to.IsGroup ()  && ( m_stationManager->GetNStaInPs() > 0 ))
          {
            // Received a group addressed packet and there is at least 1 STA in PS
            NS_LOG_DEBUG ("AP received a group addressed packet and there is at least 1 STA in PS. To address of Packet: "<< to);
            // Create a wifi mac queue item from this packet
            WifiMacHeader header;
            header.SetType (WIFI_MAC_QOSDATA);
            header.SetQosTid (0);
            header.SetAddr1 (to);
            header.SetAddr2 (GetAddress ());
            header.SetAddr3 (from);

            Ptr<WifiMacQueueItem> PsQueueItem = Create<WifiMacQueueItem> (packet, header);
            m_psMulticastBuffer->Enqueue (PsQueueItem);

            // Ptr<WifiMacQueueItem> PsQueueItem_temp = Create<WifiMacQueueItem> (packet, header);
            // m_psMulticastBuffer->Enqueue (PsQueueItem_temp);     // Remove - shyam - for testing - duplicate multicast
            // Ptr<WifiMacQueueItem> PsQueueItem_temp2 = Create<WifiMacQueueItem> (packet, header);
            // m_psMulticastBuffer->Enqueue (PsQueueItem_temp2);     // Remove - shyam - for testing - duplicate multicast
            
            NS_LOG_DEBUG ("Group addressed packet received while there are PS STAs and added to PsBuffer. nPackets of m_psMulticastBuffer: "<<m_psMulticastBuffer->GetNPackets ());
            NS_LOG_DEBUG ("Returning without sending packet. Packet is safe in the m_psMulticastBuffer.");

            ForwardUp (packet, from, to);        // shyam - is this correct?
          }
          else if (to.IsGroup ()
                   || m_stationManager->IsAssociated (to))
            {
              NS_LOG_DEBUG ("forwarding frame from=" << from << ", to=" << to);
              Ptr<Packet> copy = packet->Copy ();

              //If the frame we are forwarding is of type QoS Data,
              //then we need to preserve the UP in the QoS control
              //header...
              if (hdr->IsQosData ())
                {
                  ForwardDown (copy, from, to, hdr->GetQosTid ());
                }
              else
                {
                  ForwardDown (copy, from, to);
                }
              ForwardUp (packet, from, to);
            }
          else
            {
              ForwardUp (packet, from, to);
            }
        }
      else if (hdr->IsFromDs ()
               && hdr->IsToDs ())
        {
          //this is an AP-to-AP frame
          //we ignore for now.
          NotifyRxDrop (packet);
        }
      else
        {
          //we can ignore these frames since
          //they are not targeted at the AP
          NotifyRxDrop (packet);
        }
      return;
    }
  else if (hdr->IsMgt ())
    {
      if (hdr->IsProbeReq ())
        {
          NS_ASSERT (hdr->GetAddr1 ().IsBroadcast ());
          MgtProbeRequestHeader probeRequestHeader;
          packet->PeekHeader (probeRequestHeader);
          Ssid ssid = probeRequestHeader.GetSsid ();
          if (ssid == GetSsid () || ssid.IsBroadcast ())
            {
              NS_LOG_DEBUG ("Probe request received from " << from << ": send probe response");
              SendProbeResp (from);
            }
          return;
        }
      else if (hdr->GetAddr1 () == GetAddress ())
        {
          if (hdr->IsAssocReq ())
            {
              NS_LOG_DEBUG ("Association request received from " << from);
              //first, verify that the the station's supported
              //rate set is compatible with our Basic Rate set
              MgtAssocRequestHeader assocReq;
              packet->PeekHeader (assocReq);
              CapabilityInformation capabilities = assocReq.GetCapabilities ();
              m_stationManager->AddSupportedPhyPreamble (from, capabilities.IsShortPreamble ());
              SupportedRates rates = assocReq.GetSupportedRates ();
              bool problem = false;
              if (rates.GetNRates () == 0)
                {
                  problem = true;
                }
              if (GetHtSupported ())
                {
                  //check whether the HT STA supports all MCSs in Basic MCS Set
                  HtCapabilities htcapabilities = assocReq.GetHtCapabilities ();
                  if (htcapabilities.IsSupportedMcs (0))
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!htcapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetVhtSupported ())
                {
                  //check whether the VHT STA supports all MCSs in Basic MCS Set
                  VhtCapabilities vhtcapabilities = assocReq.GetVhtCapabilities ();
                  if (vhtcapabilities.GetVhtCapabilitiesInfo () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!vhtcapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetHeSupported ())
                {
                  //check whether the HE STA supports all MCSs in Basic MCS Set
                  HeCapabilities hecapabilities = assocReq.GetHeCapabilities ();
                  if (hecapabilities.GetSupportedMcsAndNss () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!hecapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (problem)
                {
                  NS_LOG_DEBUG ("One of the Basic Rate set mode is not supported by the station: send association response with an error status");
                  SendAssocResp (hdr->GetAddr2 (), false, false);
                }
              else
                {
                  NS_LOG_DEBUG ("The Basic Rate set modes are supported by the station");
                  //record all its supported modes in its associated WifiRemoteStation
                  for (const auto & mode : m_phy->GetModeList ())
                    {
                      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                        {
                          m_stationManager->AddSupportedMode (from, mode);
                        }
                    }
                  if (GetErpSupported () && m_stationManager->GetErpOfdmSupported (from) && capabilities.IsShortSlotTime ())
                    {
                      m_stationManager->AddSupportedErpSlotTime (from, true);
                    }
                  if (GetHtSupported ())
                    {
                      HtCapabilities htCapabilities = assocReq.GetHtCapabilities ();
                      if (htCapabilities.IsSupportedMcs (0))
                        {
                          m_stationManager->AddStationHtCapabilities (from, htCapabilities);
                        }
                    }
                  if (GetVhtSupported ())
                    {
                      VhtCapabilities vhtCapabilities = assocReq.GetVhtCapabilities ();
                      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
                      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
                        {
                          m_stationManager->AddStationVhtCapabilities (from, vhtCapabilities);
                          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_VHT))
                            {
                              if (vhtCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  if (GetHtSupported ())
                    {
                      ExtendedCapabilities extendedCapabilities = assocReq.GetExtendedCapabilities ();
                      //TODO: to be completed
                    }
                  if (GetHeSupported ())
                    {
                      HeCapabilities heCapabilities = assocReq.GetHeCapabilities ();
                      if (heCapabilities.GetSupportedMcsAndNss () != 0)
                        {
                          m_stationManager->AddStationHeCapabilities (from, heCapabilities);
                          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HE))
                            {
                              if (heCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  m_stationManager->RecordWaitAssocTxOk (from);
                  NS_LOG_DEBUG ("Send association response with success status");
                  SendAssocResp (hdr->GetAddr2 (), true, false);
                }
              return;
            }
          else if (hdr->IsReassocReq ())
            {
              NS_LOG_DEBUG ("Reassociation request received from " << from);
              //first, verify that the the station's supported
              //rate set is compatible with our Basic Rate set
              MgtReassocRequestHeader reassocReq;
              packet->PeekHeader (reassocReq);
              CapabilityInformation capabilities = reassocReq.GetCapabilities ();
              m_stationManager->AddSupportedPhyPreamble (from, capabilities.IsShortPreamble ());
              SupportedRates rates = reassocReq.GetSupportedRates ();
              bool problem = false;
              if (rates.GetNRates () == 0)
                {
                  problem = true;
                }
              if (GetHtSupported ())
                {
                  //check whether the HT STA supports all MCSs in Basic MCS Set
                  HtCapabilities htcapabilities = reassocReq.GetHtCapabilities ();
                  if (htcapabilities.IsSupportedMcs (0))
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!htcapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetVhtSupported ())
                {
                  //check whether the VHT STA supports all MCSs in Basic MCS Set
                  VhtCapabilities vhtcapabilities = reassocReq.GetVhtCapabilities ();
                  if (vhtcapabilities.GetVhtCapabilitiesInfo () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!vhtcapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetHeSupported ())
                {
                  //check whether the HE STA supports all MCSs in Basic MCS Set
                  HeCapabilities hecapabilities = reassocReq.GetHeCapabilities ();
                  if (hecapabilities.GetSupportedMcsAndNss () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!hecapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (problem)
                {
                  NS_LOG_DEBUG ("One of the Basic Rate set mode is not supported by the station: send reassociation response with an error status");
                  SendAssocResp (hdr->GetAddr2 (), false, true);
                }
              else
                {
                  NS_LOG_DEBUG ("The Basic Rate set modes are supported by the station");
                  //update all its supported modes in its associated WifiRemoteStation
                  for (const auto & mode : m_phy->GetModeList ())
                    {
                      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                        {
                          m_stationManager->AddSupportedMode (from, mode);
                        }
                    }
                  if (GetErpSupported () && m_stationManager->GetErpOfdmSupported (from) && capabilities.IsShortSlotTime ())
                    {
                      m_stationManager->AddSupportedErpSlotTime (from, true);
                    }
                  if (GetHtSupported ())
                    {
                      HtCapabilities htCapabilities = reassocReq.GetHtCapabilities ();
                      if (htCapabilities.IsSupportedMcs (0))
                        {
                          m_stationManager->AddStationHtCapabilities (from, htCapabilities);
                        }
                    }
                  if (GetVhtSupported ())
                    {
                      VhtCapabilities vhtCapabilities = reassocReq.GetVhtCapabilities ();
                      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
                      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
                        {
                          m_stationManager->AddStationVhtCapabilities (from, vhtCapabilities);
                          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_VHT))
                            {
                              if (vhtCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  if (GetHtSupported ())
                    {
                      ExtendedCapabilities extendedCapabilities = reassocReq.GetExtendedCapabilities ();
                      //TODO: to be completed
                    }
                  if (GetHeSupported ())
                    {
                      HeCapabilities heCapabilities = reassocReq.GetHeCapabilities ();
                      if (heCapabilities.GetSupportedMcsAndNss () != 0)
                        {
                          m_stationManager->AddStationHeCapabilities (from, heCapabilities);
                          for (const auto & mcs : m_phy->GetMcsList (WIFI_MOD_CLASS_HE))
                            {
                              if (heCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  m_stationManager->RecordWaitAssocTxOk (from);
                  NS_LOG_DEBUG ("Send reassociation response with success status");
                  SendAssocResp (hdr->GetAddr2 (), true, true);
                }
              return;
            }
          else if (hdr->IsDisassociation ())
            {
              NS_LOG_DEBUG ("Disassociation received from " << from);
              m_stationManager->RecordDisassociated (from);
              for (auto it = m_staList.begin (); it != m_staList.end (); ++it)
                {
                  if (it->second == from)
                    {
                      m_staList.erase (it);
                      m_deAssocLogger (it->first, it->second);
                      if (m_stationManager->GetDsssSupported (from) && !m_stationManager->GetErpOfdmSupported (from))
                        {
                          m_numNonErpStations--;
                        }
                      if (!m_stationManager->GetHtSupported (from))
                        {
                          m_numNonHtStations--;
                        }
                      UpdateShortSlotTimeEnabled ();
                      UpdateShortPreambleEnabled ();
                      break;
                    }
                }
              return;
            }
        }
    }

  //Invoke the receive handler of our parent class to deal with any
  //other frames. Specifically, this will handle Block Ack-related
  //Management Action frames.
  RegularWifiMac::Receive (Create<WifiMacQueueItem> (packet, *hdr));
}

void
ApWifiMac::DeaggregateAmsduAndForward (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  for (auto& i : *PeekPointer (mpdu))
    {
      if (i.second.GetDestinationAddr () == GetAddress ())
        {
          ForwardUp (i.first, i.second.GetSourceAddr (),
                     i.second.GetDestinationAddr ());
        }
      else
        {
          Mac48Address from = i.second.GetSourceAddr ();
          Mac48Address to = i.second.GetDestinationAddr ();
          NS_LOG_DEBUG ("forwarding QoS frame from=" << from << ", to=" << to);
          ForwardDown (i.first->Copy (), from, to, mpdu->GetHeader ().GetQosTid ());
        }
    }
}

void
ApWifiMac::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop->Initialize ();
  m_beaconEvent.Cancel ();
  if (m_enableBeaconGeneration)
    {
      if (m_enableBeaconJitter)
        {
          Time jitter = MicroSeconds (static_cast<int64_t> (m_beaconJitter->GetValue (0, 1) * (GetBeaconInterval ().GetMicroSeconds ())));
          NS_LOG_DEBUG ("Scheduling initial beacon for access point " << GetAddress () << " at time " << jitter);
          m_beaconEvent = Simulator::Schedule (jitter, &ApWifiMac::SendOneBeacon, this);
        }
      else
        {
          NS_LOG_DEBUG ("Scheduling initial beacon for access point " << GetAddress () << " at time 0");
          m_beaconEvent = Simulator::ScheduleNow (&ApWifiMac::SendOneBeacon, this);
        }
    }
  NS_ABORT_IF (!TraceConnectWithoutContext ("AckedMpdu", MakeCallback (&ApWifiMac::TxOk, this)));
  NS_ABORT_IF (!TraceConnectWithoutContext ("DroppedMpdu", MakeCallback (&ApWifiMac::TxFailed, this)));
  RegularWifiMac::DoInitialize ();
  UpdateShortSlotTimeEnabled ();
  UpdateShortPreambleEnabled ();
}

bool
ApWifiMac::GetUseNonErpProtection (void) const
{
  bool useProtection = (m_numNonErpStations > 0) && m_enableNonErpProtection;
  m_stationManager->SetUseNonErpProtection (useProtection);
  return useProtection;
}

uint16_t
ApWifiMac::GetNextAssociationId (void)
{
  //Return the first free AID value between 1 and 2007
  for (uint16_t nextAid = 1; nextAid <= 2007; nextAid++)
    {
      if (m_staList.find (nextAid) == m_staList.end ())
        {
          return nextAid;
        }
    }
  NS_FATAL_ERROR ("No free association ID available!");
  return 0;
}

const std::map<uint16_t, Mac48Address>&
ApWifiMac::GetStaList (void) const
{
  return m_staList;
}

uint16_t
ApWifiMac::GetAssociationId (Mac48Address addr) const
{
  return m_stationManager->GetAssociationId (addr);
}

uint8_t
ApWifiMac::GetBufferStatus (uint8_t tid, Mac48Address address) const
{
  auto it = m_bufferStatus.find (WifiAddressTidPair (address, tid));
  if (it == m_bufferStatus.end ()
      || it->second.timestamp + m_bsrLifetime < Simulator::Now ())
    {
      return 255;
    }
  return it->second.value;
}

void
ApWifiMac::SetBufferStatus (uint8_t tid, Mac48Address address, uint8_t size)
{
  if (size == 255)
    {
      // no point in storing an unspecified size
      m_bufferStatus.erase (WifiAddressTidPair (address, tid));
    }
  else
    {
      m_bufferStatus[WifiAddressTidPair (address, tid)] = {size, Simulator::Now ()};
      // shyam - buffer status updated - request channel access 
      // if (GetQosSupported ())
      // {
      //   GetBEQueue()->RequestAccess();
      //   // m_channelAccessManager->RequestAccess(GetBEQueue());
      
      // }
    }
    //shyam - everytime Buffer Status is updated in any way, set expectingBasicTf accordingly for AWAKE TWT STAs
    if (m_stationManager->GetTwtAgreement (address) == true && m_stationManager->GetTwtAwakeForSp (address))
    {
      uint8_t maxSize = GetMaxBufferStatus (address);
      // if (maxSize > 0 && maxSize < 255)
      if (maxSize > 0)
      {
        m_stationManager->SetExpectingBasicTf (address, true);
      }
      else
      {
        m_stationManager->SetExpectingBasicTf (address, false);
        // m_stationManager->SetTwtAwakeForSp (address, false);
      }

    }

    

}

void
ApWifiMac::RequestBEChannelAccess ()
{
  NS_LOG_FUNCTION (this);
  if (GetQosSupported())
  {
    GetBEQueue()->RequestAccess();
  }
}
uint8_t
ApWifiMac::GetMaxBufferStatus (Mac48Address address) const
{
  uint8_t maxSize = 0;
  bool found = false;

  for (uint8_t tid = 0; tid < 8; tid++)
    {
      uint8_t size = GetBufferStatus (tid, address);
      if (size != 255)
        {
          maxSize = std::max (maxSize, size);
          found = true;
        }
    }

  if (found)
    {
      return maxSize;
    }
  return 255;
}

void 
ApWifiMac::SetTwtSchedule (uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt)

{
  NS_LOG_FUNCTION (this << flowId << peerMacAddress << isRequestingNode << isImplicitAgreement << flowType << isTriggerBasedAgreement << isIndividualAgreement << twtChannel << wakeInterval << nominalWakeDuration << nextTwt);
  // Fetching time left till next beacon
  NS_ABORT_MSG_IF (wakeInterval<nominalWakeDuration, "wakeInterval should be >= nominalWakeDuration");
  Time timeLeftTillNextBeacon =  Simulator::GetDelayLeft (m_beaconEvent);
  NS_LOG_DEBUG ("At time of adding TWT schedule at AP, timeLeftTillNextBeacon (us) = "<<timeLeftTillNextBeacon.GetMicroSeconds());
  // TWT agreement should contain nextTwt but AP should schedule starting from (nextTwt + timeLeftTillNextBeacon) at 2 places: CheckTwtBufferForThisSta and m_stationManager->CreateTwtAgreement
  m_stationManager->CreateTwtAgreement (flowId, peerMacAddress, isRequestingNode, isImplicitAgreement, flowType, isTriggerBasedAgreement, isIndividualAgreement, twtChannel, wakeInterval, nominalWakeDuration, nextTwt, timeLeftTillNextBeacon);
  NS_LOG_DEBUG ("CheckTwtBufferForThisSta is scheduled for time (ms) "<<(nextTwt + timeLeftTillNextBeacon).GetMilliSeconds());
  Simulator::Schedule (nextTwt + timeLeftTillNextBeacon, &ApWifiMac::CheckTwtBufferForThisSta, this, peerMacAddress, wakeInterval);
  // Add TWT agreement to m_pendingTwtAgreements in HeFeMan
  NS_ABORT_MSG_IF (GetHeConfiguration () == 0, "SetTwtSchedule only supported on HE APs");

  Ptr<HeFrameExchangeManager> m_heFem = DynamicCast<HeFrameExchangeManager> (GetFrameExchangeManager ());
  WifiTwtAgreement *agreement = new WifiTwtAgreement (flowId, peerMacAddress, isRequestingNode, isImplicitAgreement, flowType, isTriggerBasedAgreement, isIndividualAgreement, twtChannel, wakeInterval, nominalWakeDuration, nextTwt);  

  NS_LOG_DEBUG ("TWT Accept frame queued at AP for STA: "<<peerMacAddress<<"; Moving all packets for this STA to PS buffer");
  MoveAllPacketsToPsBufferByAddress(peerMacAddress);

  m_heFem->AddToPendingTwtAgreement (*agreement);
  // start BE channel access
  RequestBEChannelAccess ();

  // m_stationManager->SetTwtParameters(staMacAddress, twtPeriod, minTwtWakeDuration, twtAnnounced, twtTriggerBased);
  // m_stationManager->SetTwtParameters(staMacAddress, twtFlowId, targetWakeTime, twtPeriod, minTwtWakeDuration, twtAnnounced, twtTriggerBased);
  // m_stationManager->SetPSM (peerMacAddress, true);
  

  NS_LOG_DEBUG ("TWT Schedule set for STA with MAC "<<peerMacAddress);
  
  
  // NS_LOG_DEBUG ("Checking that AP's MAC queue does not have any MPDUs to this STA");
  // if (GetQosSupported ())
  // {
    
  //   // NS_ASSERT_MSG (m_edca[AC_BE]->PeekNextMpdu(0, from) == 0, "MAC AC_BE queue at AP was not empty for STA switching from CAM to PSM");
  //   NS_LOG_DEBUG ("Moving all AC_BE queued packets at AP for this STA into PS buffer");
  //   while (m_edca[AC_BE]->PeekNextMpdu(0, staMacAddress) != 0)
  //   {
  //     Ptr<WifiMacQueueItem> PsQueueItem = m_edca[AC_BE]->GetWifiMacQueue()->DequeueByAddress(staMacAddress);
  //     m_psUnicastBuffer->Enqueue (PsQueueItem);
  //     NS_LOG_DEBUG ("Moving one packet");
  //     // m_edca[AC_BE]->GetWifiMacQueue()->DequeueByAddress(staMacAddress);
      
  //     // NS_LOG_DEBUG ("Discarded one packet");
  //   }
    
  // }
  // else 
  // {
  //   NS_ABORT_MSG ("Non QoS STA not supported for TWT");
  // }

  // Simulator::Schedule (twtPeriod + NanoSeconds(1), &ApWifiMac::CheckTwtBufferForThisSta, this, staMacAddress, twtPeriod);
  return;
}


uint32_t 
ApWifiMac::GetPendingTwtAgreementCount ()
{
  NS_LOG_FUNCTION (this);
  // verify that this is an HE AP
  NS_ABORT_MSG_IF (GetHeConfiguration () == 0, "GetNumPendingTwtAgreements only supported on HE APs");
  Ptr<HeFrameExchangeManager> m_heFem = DynamicCast<HeFrameExchangeManager> (GetFrameExchangeManager ());
  return m_heFem->GetPendingTwtAgreementsCount ();
}

void 
ApWifiMac::CheckTwtBufferForThisSta (Mac48Address staMacAddress, Time twtPeriod)
{
  NS_LOG_FUNCTION (this << staMacAddress<< twtPeriod );
  if (m_stationManager->GetTwtAgreement (staMacAddress) == true)
    {
      Simulator::Schedule (twtPeriod, &ApWifiMac::CheckTwtBufferForThisSta, this, staMacAddress, twtPeriod);
      // Simulator::Schedule (twtPeriod + NanoSeconds(1), &ApWifiMac::CheckTwtBufferForThisSta, this, staMacAddress, twtPeriod);
    }
  // Clearing BSR for all tid of this TWT STA - in case STA sent a BSR before the SP and it has a TB TWT
  // This is done so that AP will always send a BSRP at the beginning of a TWT SP
  for (uint8_t tid = 0; tid < 8; tid++)
  {
    NS_LOG_DEBUG ("At AP, Clearing BSR for TID "<<(int)tid<<" of STA with MAC "<<staMacAddress);
    SetBufferStatus (tid, staMacAddress, 255);
  }
  

  NS_LOG_FUNCTION ("m_stationManager->GetTwtAwakeForSp(staMacAddress) is now: "<< m_stationManager->GetTwtAwakeForSp(staMacAddress));

  if (m_stationManager->GetTwtAwakeForSp(staMacAddress))
    {
      uint32_t numPacketsForSta;
      numPacketsForSta = m_psUnicastBuffer->GetNPacketsByAddress (staMacAddress);
      WifiMacHeader hdr;

      if (numPacketsForSta == 0  && m_stationManager->GetTwtTriggerBased (staMacAddress))
      {
        
        // NS_LOG_DEBUG ("Trigger based TWT STA has no packets queued. But Trigger must be sent.");
        NS_LOG_DEBUG ("Trigger based TWT STA has no packets queued. But Trigger must be sent. Requesting channel access.");
        Simulator::Schedule (NanoSeconds (1), &ApWifiMac::RequestBEChannelAccess, this);
        // GetBEQueue()->RequestAccess();
        // m_channelAccessManager->RequestAccess (GetBEQueue() );
        

      }
      
      // Below snippet is for non-trigger based-----------------------------
      if (numPacketsForSta == 0)
      {
        NS_LOG_DEBUG ("No buffered unicast packets to forward to TWT STA with MAC "<<staMacAddress);
        
        
        return;
      }
      
      while (numPacketsForSta > 0 && m_stationManager->GetTwtAwakeForSp(staMacAddress))
        {
          Ptr<WifiMacQueueItem> PsQueueItem;
          PsQueueItem = m_psUnicastBuffer->DequeueByAddress (staMacAddress);
          Ptr<Packet> packet = PsQueueItem->GetPacket()->Copy ();   // Need this because wifi-mac-queue-item returns a const packet
          bool moreData = false;
          numPacketsForSta = m_psUnicastBuffer->GetNPacketsByAddress (staMacAddress);
          if (numPacketsForSta > 0)
          {
            moreData = true;
          }
          ForwardDown (packet, PsQueueItem->GetHeader().GetAddr3(), PsQueueItem->GetHeader().GetAddr1(), moreData);

        
        }
    }
  return;
}


} //namespace ns3
