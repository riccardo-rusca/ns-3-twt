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

#ifndef STA_WIFI_MAC_H
#define STA_WIFI_MAC_H

#include "regular-wifi-mac.h"
#include "mgt-headers.h"
#include "ns3/random-variable-stream.h"
#include "wifi-twt-agreement.h"

class TwoLevelAggregationTest;
class AmpduAggregationTest;
class HeAggregationTest;

namespace ns3  {

class SupportedRates;
class CapabilityInformation;

/**
 * \ingroup wifi
 *
 * Struct to hold information regarding observed AP through
 * active/passive scanning
 */
struct ApInfo
{
  Mac48Address m_bssid;               ///< BSSID
  Mac48Address m_apAddr;              ///< AP MAC address
  double m_snr;                       ///< SNR in linear scale
  bool m_activeProbing;               ///< Flag whether active probing is used or not
  MgtBeaconHeader m_beacon;           ///< Beacon header
  MgtProbeResponseHeader m_probeResp; ///< Probe Response header
};

/**
 * \ingroup wifi
 *
 * The Wifi MAC high model for a non-AP STA in a BSS. The state
 * machine is as follows:
 *
   \verbatim
   ---------       --------------                                         -----------
   | Start |       | Associated | <-------------------------        ----> | Refused |
   ---------       --------------                           |      /      -----------
      |              |   /------------------------------\   |     /
      \              v   v                              |   v    /
       \    ----------------     ---------------     -----------------------------
        \-> | Unassociated | --> | Wait Beacon | --> | Wait Association Response |
            ----------------     ---------------     -----------------------------
                  \                  ^     ^ |              ^    ^ |
                   \                 |     | |              |    | |
                    \                v      -               /     -
                     \    -----------------------          /
                      \-> | Wait Probe Response | --------/
                          -----------------------
                                  ^ |
                                  | |
                                   -
   \endverbatim
 *
 * Notes:
 * 1. The state 'Start' is not included in #MacState and only used
 *    for illustration purpose.
 * 2. The Unassociated state is a transient state before STA starts the
 *    scanning procedure which moves it into either Wait Beacon or Wait
 *    Probe Response, based on whether passive or active scanning is
 *    selected.
 * 3. In Wait Beacon and Wait Probe Response, STA is gathering beacon or
 *    probe response packets from APs, resulted in a list of candidate AP.
 *    After the respective timeout, it then tries to associate to the best
 *    AP (i.e., best SNR). STA will switch between the two states and
 *    restart the scanning procedure if SetActiveProbing() called.
 * 4. In the case when AP responded to STA's association request with a
 *    refusal, STA will try to associate to the next best AP until the list
 *    of candidate AP is exhausted which sends STA to Refused state.
 *    - Note that this behavior is not currently tested since ns-3 does not
  *     implement association refusal at present.
 * 5. The transition from Wait Association Response to Unassociated
 *    occurs if an association request fails without explicit
 *    refusal (i.e., the AP fails to respond).
 * 6. The transition from Associated to Wait Association Response
 *    occurs when STA's PHY capabilities changed. In this state, STA
 *    tries to reassociate with the previously associated AP.
 * 7. The transition from Associated to Unassociated occurs if the number
 *    of missed beacons exceeds the threshold.
 */
class StaWifiMac : public RegularWifiMac
{
public:
  /// Allow test cases to access private members
  friend class ::TwoLevelAggregationTest;
  /// Allow test cases to access private members
  friend class ::AmpduAggregationTest;
  /// Allow test cases to access private members
  friend class ::HeAggregationTest;
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  StaWifiMac ();
  virtual ~StaWifiMac ();

  /**
   * \param packet the packet to send.
   * \param to the address to which the packet should be sent.
   *
   * The packet should be enqueued in a TX queue, and should be
   * dequeued as soon as the channel access function determines that
   * access is granted to this MAC.
   */
  void Enqueue (Ptr<Packet> packet, Mac48Address to) override;

  /**
   * \param phy the physical layer attached to this MAC.
   */
  void SetWifiPhy (const Ptr<WifiPhy> phy) override;

  /**
   * Return whether we are associated with an AP.
   *
   * \return true if we are associated with an AP, false otherwise
   */
  bool IsAssociated (void) const;

  /**
   * Return whether all the Qos Queues are empty
   *
   * \return true if all Qos queues are empty, false otherwise
   */
  bool IsEveryEdcaQueueEmpty () const;

  /**
   * Return whether the non-Qos (DCF) Queue is empty
   *
   * \return true if non-Qos queue is empty, false otherwise
   */
  bool IsDcfQueueEmpty () const;

  /**
   * Return the association ID.
   *
   * \return the association ID
   */
  uint16_t GetAssociationId (void) const;

  /**
   * This function is called if FeMan receives a unicast frame not addressed to this STA and is not a broadcast frame. If this is a TWT STA outside of its SP, it will be put to sleep
   * 
  */
  void ReceivedUnicastNotForMe (void);
  /**
   * Enable or disable 802.11 PSM.
   *
   * \param enable enable or disable 802.11 PSM
   */
  void SetPowerSaveMode (bool enable);
  /**
   * Return whether 802.11 PSM is enabled.
   *
   * \return true if 802.11 PSM is enabled, false otherwise
   */
  bool GetPowerSaveMode (void) const;
  /**
   * Return whether a Trigger Based TWT agreement is created
   *
   * \return true if a Trigger Based TWT agreement is created
   */
  bool GetTwtTriggerBased (void) const;
  /**
   * Return whether a STA currently has a TWT SP active
   *
   * \return true if STA currently has a TWT SP active
   */
  bool GetTwtSpNow (void) const;
  /**
   * Return the remaining time if STA is in TWT SP. If not in TWT SP, Time::min is returned
   *
   * \return remaining time if STA is in TWT SP
   */
  Time GetRemainingTimeInTwtSp(void) const;
  /**
   * Get expected remaining time till next beacon
   * 
   * \return expected remaining time till next beacon
  */
  Time GetExpectedRemainingTimeTillNextBeacon(void) const;
  /**
   * Placeholder function scheduled at next expected beacon generation time at AP - used to calculate TWT schedules
   * 
  */
  void NextExpectedBeaconGeneration ();
  /**
   * If STA is in Power Save, go to sleep if both QoS and non Qos queues are empty
   * If queues are not empty, this function reschedules itself after SIFS.
   *
   * 
   */
  void SleepPsIfQueuesEmpty ();
  /**
   * * @brief Create a Twt Agreement object
   * 
   * @param flowId  Flow ID 0 to 7
   * @param peerMacAddress  Peer MAC address
   * @param isRequestingNode  true if requesting node. Set to 0 for AP/responder node
   * @param isImplicitAgreement   true if implicit agreement. false otherwise
   * @param flowType  true for unannounced agreement, false for announced agreement
   * @param isTriggerBasedAgreement   true for trigger based agreement, false for non-trigger based agreement
   * @param isIndividualAgreement   true for individual agreement, false for broadcast agreement
   * @param twtChannel  TWT channel - set to 0 for now
   * @param wakeInterval  TWT wake interval
   * @param nominalWakeDuration   TWT nominal wake duration
   * @param nextTwt   TWT next TWT SP start time   */
  void CreateTwtAgreement (uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt);
  /**
   * Start TWT schedule for given TWT agreement
   * @param agreement WifiTwtAgreement object
   * 
   */
  void StartTwtSchedule (WifiTwtAgreement agreement);
  
  /**
   * Deprecated------
   * TODO: Check if STA support TWT first - shyam
   * TODO: Create function bool isTwtActive() that returns whether TWT is active - shyam
   * Set up parameters for TWT and activate it. TWT SP will be resumed only if TWT is active (bool isTwtActive())
   * \param minTwtWakeDuration Twt agreement parameter Minimum TWT Wake duration
   * \param twtPeriod Time interval between start of two subsequent TWT SPs
   * \param twtUplinkOutsideSp If true, STA can wake up outside TWT SP for uplink traffic - to be used only for very sparse uplink
   * \param twtWakeForBeacons If true, STA will wake up for every beacon. If false, STA does not wake up
   * \param twtAnnounced If true, TWT agreement is trigger based. UL within TWT SP will be sent only if triggered
   * \param twtTriggerBased If true, TWT agreement is trigger based. UL within TWT SP will be sent only if triggered

   */
  void SetUpTwt ( Time twtPeriod, Time minTwtWakeDuration, bool twtUplinkOutsideSp = false, bool twtWakeForBeacons = false, bool twtAnnounced=false, bool twtTriggerBased = true);
  /**
   * This function wakes up the STA for every TWT SP and reschedules itself after twtPeriod
   * This function also schedules SetPhyToSleep() after minTwtWakeDuration
   * This function is called first by SetUpTwt()
   * 
   * \param minTwtWakeDuration Twt agreement parameter Minimum TWT Wake duration
   * \param twtPeriod Time interval between start of two subsequent TWT SPs
   */
  void WakeUpForTwtSp ( Time twtPeriod, Time minTwtWakeDuration);
  /**
   * Return whether a TWT agreement has been made
   *
   * \return true if a TWT agreement has been setup - even if suspended
   */
  bool GetTwtAgreement (void) const;
  /**
   * Return whether the TWT PS buffer is empty. If TWT is not used, return true
   *
   * \return true if the TWT PS buffer is empty. If TWT is not used, return true
   */
  bool IsTwtPsQueueEmpty (void) const;
  /**
   * Set the PHy to SLEEP
   *
   */
  void SetPhyToSleep ();
  /**
   * Resume the PHy from SLEEP
   *
   */
  void ResumePhyFromSleep ();
  /**
   * Called exactly at end of TWT SP (after Min Wake DUration) regardless of awake status of STA
   * If STA is not in SLEEP, already, it is set to SLEEP
   * 
   *
   */
  void EndOfTwtSpRoutine ();
  /**
   * Ends TWT SP wake state for STA when called. Puts PHY to SLEEP
   *
   */
  void SleepWithinTwtSpNow ();
  /**
   * 
   * For PSM if MAC queues are empty, put STA to Sleep.
   * For Twt, If m_sleepAfterAck is true, put STA to SLEEP and set m_sleepAfterAck to false. Otherwise do nothing. 
   * This function is called from FeManager and QoS FeManager. It is also called in some DL operations from the STA.
   * This function is used only if PSM or TWT is used
   *
   */
  void SleepAfterAckIfRequired ();
  /**
   * 
   * Set m_sleepAfterAck to true
   */
  void SetSleepAfterAck ();
  

  /**
   * 
   * Temp solution for TB PPDU in TWT
   * Send one buffered PPDU immediately to AP. Set PM and More data fields appropriately.
   * If no buffered data, send QoS null frame with PM bit = 1 and more data = 0
   *
   */
  void SendOneTwtUlPpduNow ();
  /**
   * 
   * Add to the STA MAC queue a Management TWT Info frame (unprotected S1G Action frame)
   *
   */
  void EnqueueTwtInfoFrameToApNow (uint8_t m_twtFlowId, uint8_t m_responseRequested, uint8_t m_nextTwtRequest, uint8_t m_nextTwtSubfieldSize, uint8_t m_allTwt, uint64_t m_nextTwt = 0);
  /**
   * 
   * Temp solution for Resuming/Suspending TWT - will be called from FeMan upon receiving ACK for S1G action frame
   * If nextTwt is 0, suspend TWT if TWT active and suspendable
   * If nextTwt is >0, Resume TWT
   *
   */
  void ResumeSuspendTwtIfRequired (u_int8_t twtId, uint64_t nextTwt);
  /**
   * Returns m_PsWaitingForPolledFrame for PSs STA 
   * 
   */
  bool isStaPsWaitingForPolledFrame ();
    /**
   * Check if Polled frame was received. If not, send ps poll.
   */
  void CheckPsPolledFrameReceived (void);
  /**
   * Called after sending normal or Block ack successfully - Used by APSD STAs to enter sleep after receiving EOSP frame
   */
  void NormalOrBlockAckTransmitted (void);


private:
  /**
   * The current MAC state of the STA.
   */
  enum MacState
  {
    ASSOCIATED,
    WAIT_BEACON,
    WAIT_PROBE_RESP,
    WAIT_ASSOC_RESP,
    UNASSOCIATED,
    REFUSED
  };

  /**
   * Return whether AID of this STA was in the PVB of given TIM
   *
   * \return true if AID of this STA was in the PVB of given TIM, false otherwise
   */
  bool IsAidInTim (Tim tim) const;
  /**
   * Enable or disable active probing.
   *
   * \param enable enable or disable active probing
   */
  void SetActiveProbing (bool enable);
  /**
   * Return whether active probing is enabled.
   *
   * \return true if active probing is enabled, false otherwise
   */
  bool GetActiveProbing (void) const;

  /**
   * Handle a received packet.
   *
   * \param mpdu the received MPDU
   */
  void Receive (Ptr<WifiMacQueueItem> mpdu) override;
  /**
   * Update associated AP's information from beacon. If STA is not associated,
   * this information will used for the association process.
   *
   * \param beacon the beacon header
   * \param apAddr MAC address of the AP
   * \param bssid MAC address of BSSID
   */
  void UpdateApInfoFromBeacon (MgtBeaconHeader beacon, Mac48Address apAddr, Mac48Address bssid);
  /**
   * Update AP's information from probe response. This information is required
   * for the association process.
   *
   * \param probeResp the probe response header
   * \param apAddr MAC address of the AP
   * \param bssid MAC address of BSSID
   */
  void UpdateApInfoFromProbeResp (MgtProbeResponseHeader probeResp, Mac48Address apAddr, Mac48Address bssid);
  /**
   * Update AP's information from association response.
   *
   * \param assocResp the association response header
   * \param apAddr MAC address of the AP
   */
  void UpdateApInfoFromAssocResp (MgtAssocResponseHeader assocResp, Mac48Address apAddr);
  /**
   * Update list of candidate AP to associate. The list should contain ApInfo sorted from
   * best to worst SNR, with no duplicate.
   *
   * \param newApInfo the new ApInfo to be inserted
   */
  void UpdateCandidateApList (ApInfo newApInfo);

  /**
   * Forward a probe request packet to the DCF. The standard is not clear on the correct
   * queue for management frames if QoS is supported. We always use the DCF.
   */
  void SendProbeRequest (void);
  /**
   * Forward an association or reassociation request packet to the DCF.
   * The standard is not clear on the correct queue for management frames if QoS is supported.
   * We always use the DCF.
   *
   * \param isReassoc flag whether it is a reassociation request
   *
   */
  void SendAssociationRequest (bool isReassoc);
  /**
   * Send a Null frame with PS bit set enablePS
   */
  void SendNullFramePS (bool enablePS);
  /**
   * Send a PS-POLL frame to AP
   */
  void SendPsPoll (void);
  /**
   * Try to ensure that we are associated with an AP by taking an appropriate action
   * depending on the current association status.
   */
  void TryToEnsureAssociated (void);
  /**
   * This method is called after the association timeout occurred. We switch the state to
   * WAIT_ASSOC_RESP and re-send an association request.
   */
  void AssocRequestTimeout (void);
  /**
   * Start the scanning process which trigger active or passive scanning based on the
   * active probing flag.
   */
  void StartScanning (void);
  /**
   * This method is called after wait beacon timeout or wait probe request timeout has
   * occurred. This will trigger association process from beacons or probe responses
   * gathered while scanning.
   */
  void ScanningTimeout (void);
  /**
   * Return whether we are waiting for an association response from an AP.
   *
   * \return true if we are waiting for an association response from an AP, false otherwise
   */
  bool IsWaitAssocResp (void) const;
  /**
   * This method is called after we have not received a beacon from the AP
   */
  void MissedBeacons (void);
  /**
   * Restarts the beacon timer.
   *
   * \param delay the delay before the watchdog fires
   */
  void RestartBeaconWatchdog (Time delay);
  /**
   * Upon receiving a good beacon, this function is scheduled to wake the STA just for next beacon if STA is in PS. 
   * This is different from the beacon watchdog timer which takes into account m_maxMissedBeacons
   */
  void WakeForNextBeaconPS (void);
  /**
   * This function checks if there is already a beacon wake up event scheduled. If no, it schedules one based on beacon period and beaconTimeOutPs, then calls SleepPsIfQueuesEmpty
   * 
   */
  void CheckBeaconTimeOutPS (Time beaconInterval, Time beaconTimeOutPs);
  /**
   * Return an instance of SupportedRates that contains all rates that we support
   * including HT rates.
   *
   * \return SupportedRates all rates that we support
   */
  SupportedRates GetSupportedRates (void) const;
  /**
   * Set the current MAC state.
   *
   * \param value the new state
   */
  void SetState (MacState value);

  /**
   * Set the EDCA parameters.
   *
   * \param ac the access class
   * \param cwMin the minimum contention window size
   * \param cwMax the maximum contention window size
   * \param aifsn the number of slots that make up an AIFS
   * \param txopLimit the TXOP limit
   */
  void SetEdcaParameters (AcIndex ac, uint32_t cwMin, uint32_t cwMax, uint8_t aifsn, Time txopLimit);
  /**
   * Set the MU EDCA parameters.
   *
   * \param ac the Access Category
   * \param cwMin the minimum contention window size
   * \param cwMax the maximum contention window size
   * \param aifsn the number of slots that make up an AIFS
   * \param muEdcaTimer the MU EDCA timer
   */
  void SetMuEdcaParameters (AcIndex ac, uint16_t cwMin, uint16_t cwMax, uint8_t aifsn, Time muEdcaTimer);
  /**
   * Return the Capability information of the current STA.
   *
   * \return the Capability information that we support
   */
  CapabilityInformation GetCapabilities (void) const;

  /**
   * Indicate that PHY capabilities have changed.
   */
  void PhyCapabilitiesChanged (void);

  void DoInitialize (void) override;

  MacState m_state;            ///< MAC state
  uint16_t m_aid;              ///< Association AID
  Time m_waitBeaconTimeout;    ///< wait beacon timeout
  Time m_probeRequestTimeout;  ///< probe request timeout
  Time m_assocRequestTimeout;  ///< association request timeout
  Time m_advanceWakeupPS;      ///< Advance wake up to Rx beacon while in PS
  EventId m_waitBeaconEvent;   ///< wait beacon event
  EventId m_probeRequestEvent; ///< probe request event
  EventId m_assocRequestEvent; ///< association request event
  EventId m_beaconWatchdog;    ///< beacon watchdog
  EventId m_nextExpectedBeaconGenerationEvent; ///< next expected beacon generation at AP - used for setting TWT schedules using next beacon as the reference
  
  EventId m_beaconWakeUpPsEvent;    ///< Event for beacon wake up in PSM
  EventId m_twtSpEndRoutineEvent;    ///< Event for end of twt sp routine - designed only for one TWT agreement per STA
  
  EventId m_apsdEospRxTimeoutEvent;    ///< Event for Eosp Frame Rx timeout after sending an UL frame in APSD

  Time m_beaconWatchdogEnd;    ///< beacon watchdog end
  uint32_t m_maxMissedBeacons; ///< maximum missed beacons
  bool m_activeProbing;        ///< active probing
  bool m_PSM;                 ///< 802.11 PSM
  bool m_waitingToSwitchToPSM;                 ///< If set to true, STA is waiting for ACK from AP to switch to PSM
  bool m_waitingToSwitchOutOfPSM;                 ///< If set to true, STA is waiting for ACK from AP to switch to PSM
  bool m_pendingPsPoll;       ///< When there is both unicast and multicast downlink buffered, set flag to retrieve unicast packets after multicast downlink is complete.
  bool m_PsNeedToBeAwake;       //< When STA is in PS, this flag is set if an operation requires the STA to be awake till it is complete. Must be set back to false by the same operation
  bool m_PsWaitingForPolledFrame;       //< When STA is in PS, this flag is set immediately after a PS-POLL frame is queued. As soon as a DL frame is received from AP (regardless of MoreData field), this flag is unset
  Time m_PsBeaconTimeOut;       //< When STA is in PS, and wakes up for a beacon, it wait for m_PsBeaconTimeOut before going back to sleep
  
  bool m_uApsd;             //< Set to true if U-APSD is enabled for BE traffic. If false, basic PSM is assumed for STA sending PM = 1
  Time m_apsdEospTimeOut;       //< For U-APSD, STA enters sleep after this duration even if EOSP frame is not received after sending a BE QoS data frame
  bool m_uApsdWaitingToSleepAfterAck;     //< STA has received EOSP frame - waiting to sleep after ACK - notmal or block
  bool m_uApsdWaitingForEospFrame;     //< STA is waiting for an EOSP QoS frame to enter sleep - do not enter sleep if this flag is true


  // TWT state maintenance
  typedef std::pair<uint8_t, ns3::Mac48Address> TwtFlowMacPair; // Flow ID and MAC address pair for TWT state maintenance
  std::map<TwtFlowMacPair, WifiTwtAgreement> m_twtAgreementMap;
  std::map<Mac48Address, uint8_t> m_twtAgreementCount;  //!< Number of active TWT agreements with given MAC address - should be <=8
  
  
  
  // TWT parameters - will be deprecated
  bool m_twtAgreement;      //< Set to true if a TWT schedule has been set using SetUpTwt(). STA should wake up for the SP only if m_twtActive is set to true
  bool m_twtActive;           //< Set to true if the existing TWT agreement is active (not suspended). Automatically set to true upon creating a TWT schedule using SetUpTwt()
  bool m_awakeForTwtSp;           //< Set to true if Sta is currently awake during a TWT SP. Value is invalid in absence of TWT agreement
  bool m_twtSpNow;           //< Set to true at beginning of every TWT SP. Set to false after min wake duration regardless of awake status or suspension
  bool m_wakeUpForNextTwt;           //< Set to true if STA should wake up for next TWT - used only for announced TWT - not for TWT susp/res
  Ptr<WifiMacQueue>  m_twtPsBuffer;          //!< Uplink Buffer pointer for PS stations with TWT
  // bool m_twtUplinkOutsideSp;      //< Set to true if Sta is allowed to wake up outside TWT SP to send uplink packets - use only for very sparse uplink (less than one packet per TWT period)
  bool m_twtWakeForBeacons;       //<  Set to true if all beacons should be received. If false, STA will not wake up for beacons
  bool m_twtTriggerBased;       //<  Set to true it is trigger based TWT- within TWT SP, UL from STA can be sent only if triggered
  bool m_twtAnnounced;       //<  Set to true if it is Announced TWT
  bool m_sleepAfterAck;       //< Used only for PSM and TWT. Set to true if STA should enter SLEEP after an ACK or BACK is received. Defaults to false
  Ptr<UniformRandomVariable> m_uniformRandomVariable;     //< Used for random Ps Poll transmission in PSM

  std::vector<ApInfo> m_candidateAps; ///< list of candidate APs to associate to
  // Note: std::multiset<ApInfo> might be a candidate container to implement
  // this sorted list, but we are using a std::vector because we want to sort
  // based on SNR but find duplicates based on BSSID, and in practice this
  // candidate vector should not be too large.

  TracedCallback<Mac48Address> m_assocLogger;   ///< association logger
  TracedCallback<Mac48Address> m_deAssocLogger; ///< disassociation logger
  TracedCallback<Time>         m_beaconArrival; ///< beacon arrival logger
};

} //namespace ns3

#endif /* STA_WIFI_MAC_H */
