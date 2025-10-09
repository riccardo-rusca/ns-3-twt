/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023
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
 * Authors: Shyam K Venkateswaran <vshyamkrishnan@gmail.com>
 */

#ifndef WIFI_TWT_AGREEMENT_H
#define WIFI_TWT_AGREEMENT_H

// refer IEEE 802.11-2020 9.4.2.199

#include "ns3/mac48-address.h"
#include "mgt-headers.h"    


namespace ns3 {
/**
 * \brief Maintains information for a twt agreement.
 * \ingroup wifi
 */
class WifiTwtAgreement
{
public:
    
    /**
    * Constructor for TwtAgreement with given TWT configuration
    *
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
    * \param nextTwt next TWT time
    */
    WifiTwtAgreement(uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt);    
    
    virtual ~WifiTwtAgreement ();

    // Getters and setters
    /**
     * Return the flow ID
     *
     * \return the flow ID
     */
    uint8_t GetFlowId() const;

    /**
     * Set the flow ID
     *
     * \param id the flow ID
     */
    void SetFlowId(uint8_t id);

    /** 
     * Return the peer MAC address
     *
     * \return the peer MAC address
     */
    Mac48Address GetPeerMacAddress() const;

    /** 
     * Set the peer MAC address
     *
     * \param address the peer MAC address
     */
    void SetPeerMacAddress(Mac48Address address);


    /** 
     * Return whether this node is the requesting node
     *
     * \return true if this node is the requesting node, false otherwise
     */
    bool IsRequestingNode() const;
    
    /**
     * Set whether this node is the requesting node
     *
     * \param value true if this node is the requesting node, false otherwise
     */
    void SetRequestingNode(bool value);

    /**
     * Return whether this is an implicit agreement
     *
     * \return true for implicit agreement, false for explicit agreement
     */
    bool IsImplicitAgreement() const;
    
    /**
     * Set whether this is an implicit agreement
     *
     * \param value true for implicit agreement, false for explicit agreement
     */
    void SetImplicitAgreement(bool value);

    /**
     * Return the flow type
     *
     * \return true for unannounced agreement, false for announced agreement
     */
    bool GetFlowType() const;

    /**
     * Set the flow type
     *
     * \param type true for unannounced agreement, false for announced agreement
     */
    void SetFlowType(bool type);


    /** 
     * Return whether this is a trigger based agreement
     *
     * \return true for trigger based, false for non-trigger based
     */
    bool IsTriggerBasedAgreement() const;

    /** 
     * Set whether this is a trigger based agreement
     *
     * \param value true for trigger based, false for non-trigger based
     */
    void SetTriggerBasedAgreement(bool value);

    /** 
     * Return whether this is an individual agreement
     *
     * \return true for individual agreement, false for broadcast agreement
     */
    bool IsIndividualAgreement() const;
    
    /** 
     * Set whether this is an individual agreement
     *
     * \param value true for individual agreement, false for broadcast agreement
     */
    void SetIndividualAgreement(bool value);

    /** 
     * Return the TWT channel
     *
     * \return the TWT channel
     */
    u_int16_t GetTwtChannel() const;

    /** 
     * Set the TWT channel
     *
     * \param channel the TWT channel
     */
    void SetTwtChannel(u_int16_t channel);

    /** 
     * Return the agreed upon TWT wake interval
     *
     * \return the agreed upon TWT wake interval
     */
    Time GetWakeInterval() const;
    
    /** 
     * Set the agreed upon TWT wake interval
     *
     * \param interval the agreed upon TWT wake interval
     */
    void SetWakeInterval(const Time& interval);

    /** 
     * Return the nominal TWT wake duration
     *
     * \return the nominal TWT wake duration
     */
    Time GetNominalWakeDuration() const;
    
    /** 
     * Set the nominal TWT wake duration
     *
     * \param duration the nominal TWT wake duration
     */
    void SetNominalWakeDuration(const Time& duration);

    /** 
     * Return the next TWT time
     *
     * \return the next TWT time
     */
    Time GetNextTwt() const;
    
    /** 
     * Set the next TWT time
     *
     * \param nextTwt the next TWT time
     */
    void SetNextTwt(const Time& nextTwt);

    // Friend declaration for the operator << overload
    friend std::ostream& operator<<(std::ostream& os, const WifiTwtAgreement& twt);

    // friend declaration of WifiRemoteStationManager
    friend class WifiRemoteStationManager;


protected:
    uint8_t m_flowId;    // Flow ID, between 0 and 7
    Mac48Address m_peerMacAddress; // Peer MAC address
    bool m_isRequestingNode;    // True if this node is the requesting node, false otherwise
    bool m_isImplicitAgreement; // True for implicit agreement, false for explicit agreement
    bool m_flowType; // True for unannounced agreement, false for announced agreement
    bool m_isTriggerBasedAgreement; // True for trigger based, false for non-trigger based
    bool m_isIndividualAgreement; // True for individual agreement, false for broadcast agreement
    u_int16_t m_twtChannel; // TWT channel - set as 0 for now
    Time m_wakeInterval;   // Agreed upon TWT wake interval
    Time m_nominalWakeDuration; // nominal TWT wake duration
    Time m_nextTwt;       // Next TWT time
    EventId m_nextServicePeriodStartEvent; //!< TODO next scheduled TWT SP start event
    EventId m_nextServicePeriodEndEvent; //!< TODO next scheduled TWT SP end event

};

}




#endif /* WIFI_TWT_AGREEMENT_H */