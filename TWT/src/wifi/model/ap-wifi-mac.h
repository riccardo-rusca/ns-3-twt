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

#ifndef AP_WIFI_MAC_H
#define AP_WIFI_MAC_H

#include "regular-wifi-mac.h"
#include "ns3/wifi-mac-queue.h"
#include <unordered_map>

namespace ns3 {

class SupportedRates;
class CapabilityInformation;
class DsssParameterSet;
class Tim;
class ErpInformation;
class EdcaParameterSet;
class MuEdcaParameterSet;
class HtOperation;
class VhtOperation;
class HeOperation;
class CfParameterSet;

/**
 * \brief Wi-Fi AP state machine
 * \ingroup wifi
 *
 * Handle association, dis-association and authentication,
 * of STAs within an infrastructure BSS.
 */
class ApWifiMac : public RegularWifiMac
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  ApWifiMac ();
  virtual ~ApWifiMac ();

  void SetLinkUpCallback (Callback<void> linkUp) override;
  void Enqueue (Ptr<Packet> packet, Mac48Address to) override;
  void Enqueue (Ptr<Packet> packet, Mac48Address to, Mac48Address from) override;
  bool SupportsSendFrom (void) const override;
  void SetAddress (Mac48Address address) override;
  Ptr<WifiMacQueue> GetTxopQueue (AcIndex ac) const override;

  /**
   * \param interval the interval between two beacon transmissions.
   */
  void SetBeaconInterval (Time interval);
  /**
   * \return the interval between two beacon transmissions.
   */
  Time GetBeaconInterval (void) const;
  /**
   * 
   * Added by Shyam
   * Requests BE edca channel access - only for QoS AP
   */
  void RequestBEChannelAccess ();
  /**
   * Determine the VHT operational channel width (in MHz).
   *
   * \returns the VHT operational channel width (in MHz).
   */
  uint16_t GetVhtOperationalChannelWidth (void) const;

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   *
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  /**
   * Get a const reference to the map of associated stations. Each station is
   * specified by an (association ID, MAC address) pair. Make sure not to use
   * the returned reference after that this object has been deallocated.
   *
   * \return a const reference to the map of associated stations
   */
  const std::map<uint16_t, Mac48Address>& GetStaList (void) const;
  /**
   * \param addr the address of the associated station
   * \return the Association ID allocated by the AP to the station, SU_STA_ID if unallocated
   */
  uint16_t GetAssociationId (Mac48Address addr) const;

  /**
   * Return the value of the Queue Size subfield of the last QoS Data or QoS Null
   * frame received from the station with the given MAC address and belonging to
   * the given TID.
   *
   * The Queue Size value is the total size, rounded up to the nearest multiple
   * of 256 octets and expressed in units of 256 octets, of all  MSDUs and A-MSDUs
   * buffered at the STA (excluding the MSDU or A-MSDU of the present QoS Data frame).
   * A queue size value of 254 is used for all sizes greater than 64 768 octets.
   * A queue size value of 255 is used to indicate an unspecified or unknown size.
   * See Section 9.2.4.5.6 of 802.11-2016
   *
   * \param tid the given TID
   * \param address the given MAC address
   * \return the value of the Queue Size subfield
   */
  uint8_t GetBufferStatus (uint8_t tid, Mac48Address address) const;
  /**
   * Store the value of the Queue Size subfield of the last QoS Data or QoS Null
   * frame received from the station with the given MAC address and belonging to
   * the given TID.
   *
   * \param tid the given TID
   * \param address the given MAC address
   * \param size the value of the Queue Size subfield
   */
  void SetBufferStatus (uint8_t tid, Mac48Address address, uint8_t size);
  /**
   * Return the maximum among the values of the Queue Size subfield of the last
   * QoS Data or QoS Null frames received from the station with the given MAC address
   * and belonging to any TID.
   *
   * \param address the given MAC address
   * \return the maximum among the values of the Queue Size subfields
   */
  uint8_t GetMaxBufferStatus (Mac48Address address) const;
  /**
   * Create TWT agreement object and initiate the schedule. nextTwt is defined as an offset after the subsequent beacon Tx
    * \param flowId the flow ID
    * \param peerMacAddress the peer MAC address
    * \param isRequestingNode true if this node is the requesting node, false otherwise
    * \param isImplicitAgreement true for implicit agreement, false for explicit agreement
    * \param flowType true for unannounced agreement, false for announced agreement
    * \param isTriggerBasedAgreement true for trigger based, false for non-trigger based
    * \param isIndividualAgreement true for individual agreement, false for broadcast agreement
    * \param twtChannel TWT channel - set as 0 for now
    * \param wakeInterval agreed upon TWT wake interval
    * \param nominalWakeDuration nominal TWT wake duration
    * \param nextTwt next TWT time; nextTwt is defined as an offset after the subsequent beacon Tx
   * 
   */
  void SetTwtSchedule (uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt);
  /**
   * This function reschedules itself every twtPeriod and checks if there are packets in the buffer for this STA
   * If STA is awake (m_twtAwakeForSp), unicast packets are sent. Else, do nothing and reschedule
   *
   * \param staMacAddress MAC address of STA
   * \param twtPeriod Time interval between start of two subsequent TWT SPs
   */
  void CheckTwtBufferForThisSta (Mac48Address staMacAddress, Time twtPeriod);
  /**
   * Queue QoS EOSP NULL frame with tid = 0 to given address
   *
   * \param address of the STA for which packet will be sent
   */
  void SendQoSEospNullByAddress (Mac48Address address);  
  /**
   * Return the number of pending TWT agreements from He FeMan
   * 
   * 
   */
  uint32_t GetPendingTwtAgreementCount ();
  /**
   * Move all packets in MAC queues to PS buffer for given STA MAC address. Return total number of packets moved
   * 
   * \param address of the STA for which packets will be moved
   * \return total number of packets moved
  */
  uint32_t MoveAllPacketsToPsBufferByAddress (Mac48Address address);
   
  

private:
  void Receive (Ptr<WifiMacQueueItem> mpdu)  override;
  /**
   * The packet we sent was successfully received by the receiver
   * (i.e. we received an Ack from the receiver).  If the packet
   * was an association response to the receiver, we record that
   * the receiver is now associated with us.
   *
   * \param mpdu the MPDU that we successfully sent
   */
  void TxOk (Ptr<const WifiMacQueueItem> mpdu);
  /**
   * The packet we sent was successfully received by the receiver
   * (i.e. we did not receive an Ack from the receiver).  If the packet
   * was an association response to the receiver, we record that
   * the receiver is not associated with us yet.
   *
   * \param timeoutReason the reason why the TX timer was started (\see WifiTxTimer::Reason)
   * \param mpdu the MPDU that we failed to sent
   */
  void TxFailed (WifiMacDropReason timeoutReason, Ptr<const WifiMacQueueItem> mpdu);

  

  /**
   * This method is called to de-aggregate an A-MSDU and forward the
   * constituent packets up the stack. We override the WifiMac version
   * here because, as an AP, we also need to think about redistributing
   * to other associated STAs.
   *
   * \param mpdu the MPDU containing the A-MSDU.
   */
  void DeaggregateAmsduAndForward (Ptr<WifiMacQueueItem> mpdu) override;
  /**
   * Forward the packet down to DCF/EDCAF (enqueue the packet). This method
   * is a wrapper for ForwardDown with traffic id.
   *
   * \param packet the packet we are forwarding to DCF/EDCAF
   * \param from the address to be used for Address 3 field in the header
   * \param to the address to be used for Address 1 field in the header
   */
  void ForwardDown (Ptr<Packet> packet, Mac48Address from, Mac48Address to);
  /**
   * Forward the packet down to DCF/EDCAF (enqueue the packet).
   *
   * \param packet the packet we are forwarding to DCF/EDCAF
   * \param from the address to be used for Address 3 field in the header
   * \param to the address to be used for Address 1 field in the header
   * \param tid the traffic id for the packet
   */
  void ForwardDown (Ptr<Packet> packet, Mac48Address from, Mac48Address to, uint8_t tid);
  /**
   * Forward the packet down to DCF/EDCAF (enqueue the packet) with given moreData bool. This method
   * is a wrapper for ForwardDown with traffic id.
   *
   * \param packet the packet we are forwarding to DCF/EDCAF
   * \param from the address to be used for Address 3 field in the header
   * \param to the address to be used for Address 1 field in the header
   * \param moreData bool specifying whether the more data field be set in the packet
   * \param pushFront If true, packet is pushed to front of queue
   */
  void ForwardDown (Ptr<Packet> packet, Mac48Address from, Mac48Address to, bool moreData, bool pushFront = false);
  /**
   * Forward the packet down to DCF/EDCAF (enqueue the packet) with More Data field set if required.
   *
   * \param packet the packet we are forwarding to DCF/EDCAF
   * \param from the address to be used for Address 3 field in the header
   * \param to the address to be used for Address 1 field in the header
   * \param tid the traffic id for the packet
   * \param moreData bool specifying whether the more data field be set in the packet
   * \param pushFront If true, packet is pushed to front of queue
   */
  void ForwardDown (Ptr<Packet> packet, Mac48Address from, Mac48Address to, uint8_t tid, bool moreData, bool pushFront = false);
  /**
   * Forward a probe response packet to the DCF. The standard is not clear on the correct
   * queue for management frames if QoS is supported. We always use the DCF.
   *
   * \param to the address of the STA we are sending a probe response to
   */
  void SendProbeResp (Mac48Address to);
  /**
   * Forward an association or a reassociation response packet to the DCF.
   * The standard is not clear on the correct queue for management frames if QoS is supported.
   * We always use the DCF.
   *
   * \param to the address of the STA we are sending an association response to
   * \param success indicates whether the association was successful or not
   * \param isReassoc indicates whether it is a reassociation response
   */
  void SendAssocResp (Mac48Address to, bool success, bool isReassoc);
  /**
   * Forward a beacon packet to the beacon special DCF.
   */
  void SendOneBeacon (void);

  /**
   * Return the Capability information of the current AP.
   *
   * \return the Capability information that we support
   */
  CapabilityInformation GetCapabilities (void) const;
  /**
   * Return the ERP information of the current AP.
   *
   * \return the ERP information that we support
   */
  ErpInformation GetErpInformation (void) const;
  /**
   * Return the EDCA Parameter Set of the current AP.
   *
   * \return the EDCA Parameter Set that we support
   */
  EdcaParameterSet GetEdcaParameterSet (void) const;
  /**
   * Return the MU EDCA Parameter Set of the current AP.
   *
   * \return the MU EDCA Parameter Set that we support
   */
  MuEdcaParameterSet GetMuEdcaParameterSet (void) const;
  /**
   * Return the HT operation of the current AP.
   *
   * \return the HT operation that we support
   */
  HtOperation GetHtOperation (void) const;
  /**
   * Return the VHT operation of the current AP.
   *
   * \return the VHT operation that we support
   */
  VhtOperation GetVhtOperation (void) const;
  /**
   * Return the HE operation of the current AP.
   *
   * \return the HE operation that we support
   */
  HeOperation GetHeOperation (void) const;
  /**
   * Return an instance of SupportedRates that contains all rates that we support
   * including HT rates.
   *
   * \return SupportedRates all rates that we support
   */
  SupportedRates GetSupportedRates (void) const;
  /**
   * Return the DSSS Parameter Set that we support.
   *
   * \return the DSSS Parameter Set that we support
   */
  DsssParameterSet GetDsssParameterSet (void) const;
  /**
   * Return the TIM information element. Includes DTIM count and period.
   * Information about buffered frames for STAS in PSM are accounted for here
   *
   * \return the TIM information element
   */
  Tim GetTim (void) const;
  /**
   * Enable or disable beacon generation of the AP.
   *
   * \param enable enable or disable beacon generation
   */
  void SetBeaconGeneration (bool enable);

  /**
   * Update whether short slot time should be enabled or not in the BSS.
   * Typically, short slot time is enabled only when there is no non-ERP station
   * associated  to the AP, and that short slot time is supported by the AP and by all
   * other ERP stations that are associated to the AP. Otherwise, it is disabled.
   */
  void UpdateShortSlotTimeEnabled (void);
  /**
   * Update whether short preamble should be enabled or not in the BSS.
   * Typically, short preamble is enabled only when the AP and all associated
   * stations support short PHY preamble. Otherwise, it is disabled.
   */
  void UpdateShortPreambleEnabled (void);

  /**
   * Return whether protection for non-ERP stations is used in the BSS.
   *
   * \return true if protection for non-ERP stations is used in the BSS,
   *         false otherwise
   */
  bool GetUseNonErpProtection (void) const;
  /**
   * Send all PS buffered unicast packets for a specific STA
   *
   * \param address of the STA for which packets will be sent
   */
  void SendAllUnicastPsPacketsByAddress (Mac48Address address);
  /**
   * Send one PS buffered unicast packet for a specific STA.
   * If more than 1 packet is present, 'More data' field is set to 1 except for last packet.
   * If there is no packet buffered, send null data frame with 'More data' = 0
   *
   * \param address of the STA for which packets will be sent
   */
  void SendOneUnicastPsPacketByAddressNow (Mac48Address address);

  /**
   * Send all PS buffered group addressed packets
   *
   */
  void SendAllMulticastPsPackets ();
  /**
   * If there is at least one Qos data frame queued for this STA with EOSP = 1 in BE traffic class, return true. Else false
   *
   * \param address of the STA 
   */
  bool IsEospQosFrameQueuedForSta (Mac48Address address);

  void DoDispose (void) override;
  void DoInitialize (void) override;

  /**
   * \return the next Association ID to be allocated by the AP
   */
  uint16_t GetNextAssociationId (void);

  Ptr<Txop> m_beaconTxop;                    //!< Dedicated Txop for beacons
  bool m_enableBeaconGeneration;             //!< Flag whether beacons are being generated
  Time m_beaconInterval;                     //!< Beacon interval
  EventId m_beaconEvent;                     //!< Event to generate one beacon
  Ptr<UniformRandomVariable> m_beaconJitter; //!< UniformRandomVariable used to randomize the time of the first beacon
  bool m_enableBeaconJitter;                 //!< Flag whether the first beacon should be generated at random time
  std::map<uint16_t, Mac48Address> m_staList; //!< Map of all stations currently associated to the AP with their association ID
  uint16_t m_numNonErpStations;              //!< Number of non-ERP stations currently associated to the AP
  uint16_t m_numNonHtStations;               //!< Number of non-HT stations currently associated to the AP
  bool m_shortSlotTimeEnabled;               //!< Flag whether short slot time is enabled within the BSS
  bool m_shortPreambleEnabled;               //!< Flag whether short preamble is enabled in the BSS
  bool m_enableNonErpProtection;             //!< Flag whether protection mechanism is used or not when non-ERP STAs are present within the BSS
  Time m_bsrLifetime;                        //!< Lifetime of Buffer Status Reports
  uint8_t m_dtimPeriod;                     //!< DTIM period configured on the AP
  uint8_t m_dtimCount;                     //!< DTIM count - counts 0 to (m_dtimPeriod - 1) every beacon and cycles
  QueueSize m_psBufferSize;                 //!< A single queue of size m_psQueueSize is created to buffer packets for STAs that enter PSM. Packets are buffered in this queue will retrieved or dropped. Drop policy is DROP_OLDEST. Default size is 10 packets "10p"
  Time m_psBufferDropDelay;               //!< Any packet that is in the PS buffer for more than m_psBufferDropDelay is dropped. Default is 500 ms.
  Ptr<WifiMacQueue>  m_psUnicastBuffer;          //!< Unicast Buffer pointer for PS stations
  Ptr<WifiMacQueue>  m_psMulticastBuffer;          //!< Multicast Buffer pointer for PS stations
  bool m_psmOrApsd;                       //!< If false, PSM is used for STAs sending PM = 1; If true, U-APSD for BE traffic is assumed - #shyam makeshift solution for U-APSD
  


  /// store value and timestamp for each Buffer Status Report
  typedef struct
  {
    uint8_t value;  //!< value of BSR
    Time timestamp; //!< timestamp of BSR
  } bsrType;
  /// Per (MAC address, TID) buffer status reports
  std::unordered_map<WifiAddressTidPair, bsrType, WifiAddressTidHash> m_bufferStatus;

  /**
   * TracedCallback signature for association/deassociation events.
   *
   * \param aid the AID of the station
   * \param address the MAC address of the station
   */
  typedef void (* AssociationCallback)(uint16_t aid, Mac48Address address);

  TracedCallback<uint16_t /* AID */, Mac48Address> m_assocLogger;   ///< association logger
  TracedCallback<uint16_t /* AID */, Mac48Address> m_deAssocLogger; ///< deassociation logger
};

} //namespace ns3

#endif /* AP_WIFI_MAC_H */
