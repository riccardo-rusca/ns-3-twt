/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 INRIA
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
 * Author: Shyam K Venkateswaran <vshyamkrishnan@gmail.com>
 */

#ifndef TWT_H
#define TWT_H

#include "ns3/wifi-information-element.h"


namespace ns3 {

/**
 * \brief The Target Wake Time Information Element
 * \ingroup wifi
 *
 * The IEEE 802.11 TWT Information Element class. It knows how to serialize
 * TWT element to be added to Mgt Action frames. 
 *
 */
class Twt : public WifiInformationElement
{
public:
  /**
   * Create TWT element
   */
  Twt ();
  // TODO create constructor with parameters

  
  // Implementations of pure virtual methods of WifiInformationElement
  WifiInformationElementId ElementId () const override;
  uint8_t GetInformationFieldSize () const override;
  void SerializeInformationField (Buffer::Iterator start) const override;
  uint8_t DeserializeInformationField (Buffer::Iterator start,
                                       uint8_t length) override;

  /**
   * @brief Get the Control Octet object
   * 
   * @return Control octet consisting of NDP paging (1 bit), Responder PM (1 bit), Negotiation type = 2 bits, TWT Info Frame disabled (1 bit), Wake Duration unit = 1 bit, reserved 2 bits
   */
  uint8_t GetControlOctet (void) const;

  // Setters
    /**
     * @brief Set the Ndp Paging Indicator 0 or 1
     * 
     * @param ndpPagingIndicator boolean value
     */
    void SetNdpPagingIndicator (bool ndpPagingIndicator);
    /**
     * @brief Set the Responder Pm 0 or 1
     * 
     * @param responderPm boolean value
     */
    void SetResponderPm (bool responderPm);
    /**
     * @brief Set the Negotiation Type 0, 1, 2, or 3
     * 00 for setting up Ind. TWT 
     * 
     * @param negotiationType negotiation type
     */
    void SetNegotiationType (uint8_t negotiationType);
    /**
     * @brief Set the Twt Info Frame Disabled 0 or 1
     * 
     * @param twtInfoFrameDisabled boolean value
     */
    void SetTwtInfoFrameDisabled (bool twtInfoFrameDisabled);
    /**
     * @brief Set the Wake Duration Unit 0 or 1
     * 
     * @param wakeDurationUnit boolean value
     */
    void SetWakeDurationUnit (u_int8_t wakeDurationUnit);
    /**
     * @brief Set the Twt Request 0 or 1
     * 
     * @param twtRequest boolean value
     */
    void SetTwtRequest (bool twtRequest);
    /**
     * @brief Set the Twt Setup Command 0, 1, 2, 3, 4, 5, 6, or 7
     * 
     * @param twtSetupCommand twt setup command
     */
    void SetTwtSetupCommand (uint8_t twtSetupCommand);
    /**
     * @brief Set Trigger-based twt 0 or 1
     * 
     * @param trigger boolean value
     */
    void SetTrigger (bool trigger);
    /**
     * @brief Set Implicit twt 0 or 1
     * 
     * @param implicit boolean value
     */
    void SetImplicit (bool implicit);
    /**
     * @brief Set Flow Type 0 (unannounced) or 1
     * 
     * @param flowType boolean value
     */
    void SetFlowType (bool flowType);
    /**
     * @brief Set Flow ID 0, 1, 2, 3, 4, 5, 6, or 7
     * 
     * @param flowId flow id
     */
    void SetFlowId (uint8_t flowId);
    /**
     * @brief Set Twt Wake Interval Exponent - 5 bits
     * 
     * @param twtWakeIntervalExponent twt wake interval exponent
     */
    void SetTwtWakeIntervalExponent (uint8_t twtWakeIntervalExponent);
    /**
     * @brief Set Twt Protection 0 or 1
     * 
     * @param twtProtection boolean value
     */
    void SetTwtProtection (bool twtProtection);
    /**
     * @brief Set Target Wake Time
     * 
     * @param targetWakeTime target wake time
     */
    void SetTargetWakeTime (uint64_t targetWakeTime);
    /**
     * @brief Set Twt Group Assignment
     * 
     * @param twtGroupAssignment twt group assignment
     */
    void SetTwtGroupAssignment (uint64_t twtGroupAssignment);
    /**
     * @brief Set Nominal Wake Duration
     * 
     * @param nominalWakeDuration nominal wake duration
     */
    void SetNominalWakeDuration (uint8_t nominalWakeDuration);
    /**
     * @brief Set Twt Wake Interval Mantissa
     * 
     * @param twtWakeIntervalMantissa twt wake interval mantissa
     */
    void SetTwtWakeIntervalMantissa (uint16_t twtWakeIntervalMantissa);
    /**
     * @brief Set Twt Channel
     * 
     * @param twtChannel twt channel
     */
    void SetTwtChannel (uint8_t twtChannel);

    // Getters
    /**
     * @brief Get the Ndp Paging Indicator
     * 
     * @return boolean value
     */
    bool GetNdpPagingIndicator (void) const;
    /**
     * @brief Get the Responder Pm
     * 
     * @return boolean value
     */
    bool GetResponderPm (void) const;
    /**
     * @brief Get the Negotiation Type
     * 00 for setting up Ind. TWT 
     * 
     * @return negotiation type
     */
    uint8_t GetNegotiationType (void) const;
    /**
     * @brief Get the Twt Info Frame Disabled
     * 
     * @return boolean value
     */
    bool GetTwtInfoFrameDisabled (void) const;
    /**
     * @brief Get the Wake Duration Unit
     * 
     * @return boolean value
     */
    bool GetWakeDurationUnit (void) const;
    /**
     * @brief Get the Twt Request
     * 
     * @return boolean value
     */
    bool GetTwtRequest (void) const;
    /**
     * @brief Get the Twt Setup Command
     * 
     * @return twt setup command
     */
    uint8_t GetTwtSetupCommand (void) const;
    /**
     * @brief Get whether Trigger-based TWT
     * 
     * @return boolean value
     */
    bool IsTriggerBased (void) const;
    /**
     * @brief Get whether Implicit TWT
     * 
     * @return boolean value
     */
    bool IsImplicit (void) const;
    /**
     * @brief Get the Flow Type
     * 
     * @return boolean value
     */
    bool GetFlowType (void) const;
    /**
     * @brief Get the Flow Id
     * 
     * @return flow id
     */
    uint8_t GetFlowId (void) const;
    /**
     * @brief Get the Twt Wake Interval Exponent
     * 
     * @return twt wake interval exponent
     */
    uint8_t GetTwtWakeIntervalExponent (void) const;
    /**
     * @brief Get the Twt Protection
     * 
     * @return boolean value
     */
    bool GetTwtProtection (void) const;
    /**
     * @brief Get the Target Wake Time
     * 
     * @return target wake time
     */
    uint64_t GetTargetWakeTime (void) const;
    /**
     * @brief Get the Twt Group Assignment
     * 
     * @return twt group assignment
     */
    uint64_t GetTwtGroupAssignment (void) const;
    /**
     * @brief Get the Nominal Wake Duration
     * 
     * @return nominal wake duration
     */
    uint8_t GetNominalWakeDuration (void) const;
    /**
     * @brief Get the Twt Wake Interval Mantissa
     * 
     * @return twt wake interval mantissa
     */
    uint16_t GetTwtWakeIntervalMantissa (void) const;
    /**
     * @brief Get the Twt Channel
     * 
     * @return twt channel
     */
    uint8_t GetTwtChannel (void) const;


  /**
   * @brief Set the Control Octet
   * 
   * @param controlOctet consisting of NDP paging (1 bit), Responder PM (1 bit), Negotiation type = 2 bits, TWT Info Frame disabled (1 bit), Wake Duration unit = 1 bit, reserved 2 bits
   */
  void SetControlOctet (uint8_t controlOctet);
  
  /**
   * @brief Get the Request Type object
   * 
   * @return 2 octets consisting of TWT request (1 bit), TWT setup command - 3 bits, Trigger = 1 for trigger based twt; Implicit = 1 for implicit; Flow type = 0 for announced; Flow ID = 3 bits, TWT wake interval exponent = 5 bits, TWT protection = 1 bit
   */
  uint16_t GetRequestType (void) const;

  /**
   * @brief Set the Request Type object
   * 
   * @param requestType consisting of TWT request (1 bit), TWT setup command - 3 bits, Trigger = 1 for trigger based twt; Implicit = 1 for implicit; Flow type = 0 for announced; Flow ID = 3 bits, TWT wake interval exponent = 5 bits, TWT protection = 1 bit
   */
  void SetRequestType (u_int16_t requestType);                                       

private:

// NDP paging (1 bit), Responder PM (1 bit), Negotiation type = 2 bits, TWT Info Frame disabled (1 bit), Wake Duration unit = 1 bit, reserved 2 bits
    // Request type subfields - totally 16 bits:
      // TWT request (1 bit) = 0 for AP, 1 for STA
      // TWT setup command - 3 bits = 4 for Accept TWT
      // Trigger = 1 for trigger based twt, 0 otherwise
      // Implicit = 1 for implicit, 0 explicit
      // Flow type = 0 for announced, 1 for unannounced
      // Flow ID = 3 bits
      // TWT wake interval exponent = 5 bits = used with wake interval mantissa
        // TWT wake interval = TWT Wake Interval Mantissa x 2^(TWT Wake Interval Exponent) in micro seconds
      // TWT protection = 1 bit, 1 if protected
    // Target wake time = 0 or 8 octets = positive integer in micro seconds
    // TWT group assignment = 0, 3, or 9 octets — set to 0 for now
    // Nominal wake duration - 1 octet - Wake duration in units provided in “Wake Duration unit”
    // TWT Wake Interval Mantissa - 2 octets - in microseconds, base 2
    // TWT channel = 1 octets = leave as 0 for now

    uint8_t m_length;   //!< Length of the TWT info element
    uint8_t m_ndpPagingIndicator; // 1 bit
    uint8_t m_responderPm; // 1 bit
    uint8_t m_negotiationType; // 2 bits - 00 for setting up Ind. TWT 
    uint8_t m_twtInfoFrameDisabled; // 1 bit
    uint8_t m_wakeDurationUnit; // 1 bit
    // Request type subfields - totally 16 bits:
    uint8_t m_twtRequest; // 1 bit
    uint8_t m_twtSetupCommand; // 3 bits
    uint8_t m_trigger; // 1 bit
    uint8_t m_implicit; // 1 bit
    uint8_t m_flowType; // 1 bit
    uint8_t m_flowId; // 3 bits
    uint8_t m_twtWakeIntervalExponent; // 5 bits
    uint8_t m_twtProtection; // 1 bit
    // End of request type subfields
    uint64_t m_targetWakeTime; // 0 or 8 octets
    uint64_t m_twtGroupAssignment; // 0, 3, or 9 octets
    uint8_t m_nominalWakeDuration; // 1 octet
    uint16_t m_twtWakeIntervalMantissa; // 2 octets
    uint8_t m_twtChannel; // 1 octet
};

/**
 * Serialize TWT to the given ostream
 *
 * \param os the output stream
 * \param twt the TWT
 *
 * \return std::ostream
 */
std::ostream &operator << (std::ostream &os, const Twt &twt);

ATTRIBUTE_HELPER_HEADER (Twt);

} //namespace ns3

#endif /* TWT_H */
