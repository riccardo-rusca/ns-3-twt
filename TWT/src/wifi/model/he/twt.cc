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

#include "twt.h"

namespace ns3 {

Twt::Twt ()
{
  m_length = 15;
  m_ndpPagingIndicator = 0;
  m_responderPm = 0;
  m_negotiationType = 0; // 2 bits
  m_twtInfoFrameDisabled = 1; // 1 bit
  m_wakeDurationUnit = 0; // 1 bit -  // 0 for 256 us, 1 for 1024 us
  // Request type subfields - totally 16 bits:
  m_twtRequest = 0; // 1 bit
  m_twtSetupCommand = 4; // 3 bits - 4 for accept
  m_trigger = 1; // 1 bit
  m_implicit = 1; // 1 bit
  m_flowType = 1; // 1 bit
  m_flowId = 0; // 3 bits
  m_twtWakeIntervalExponent = 0; // 5 bits
  m_twtProtection = 0; // 1 bit
  // End of request type subfields
  m_targetWakeTime = 0; // 8 octets
  m_twtGroupAssignment = 0; // 0 octets - field is not used
  m_nominalWakeDuration = 0; // 1 octet
  m_twtWakeIntervalMantissa = 0; // 2 octets
  m_twtChannel = 0; // 1 octet

}



WifiInformationElementId
Twt::ElementId () const
{
  return IE_TWT;
}

uint8_t
Twt::GetInformationFieldSize () const
{
  return m_length;
}

void
Twt::SerializeInformationField (Buffer::Iterator start) const
{
  NS_ASSERT (m_length == 15); 
  // start.Write (m_timElement, m_length);
  Buffer::Iterator i = start;
  i.WriteU8 (GetControlOctet());
  i.WriteHtolsbU16 (GetRequestType ());
  i.WriteHtolsbU64 (m_targetWakeTime);  //TODO - supports only 8 octets now
  // No group assignment for now
  i.WriteU8 (m_nominalWakeDuration);
  i.WriteHtolsbU16 (m_twtWakeIntervalMantissa);
  i.WriteU8 (m_twtChannel);
}

uint8_t
Twt::DeserializeInformationField (Buffer::Iterator start,
                                   uint8_t length)
{
  m_length = length;
  NS_ASSERT (m_length == 15);
  Buffer::Iterator i = start;
  SetControlOctet (i.ReadU8 ());
  SetRequestType (i.ReadLsbtohU16 ());
  m_targetWakeTime = i.ReadLsbtohU64 ();
  // No group assignment for now
  m_nominalWakeDuration = i.ReadU8 ();
  m_twtWakeIntervalMantissa = i.ReadLsbtohU16 ();
  m_twtChannel = i.ReadU8 ();
  return length;
}




uint8_t 
Twt::GetControlOctet (void) const
{
  uint8_t res = 0;
  res |= m_ndpPagingIndicator;
  res |= m_responderPm << 1;
  res |= m_negotiationType << 2;
  res |= m_twtInfoFrameDisabled << 4;
  res |= m_wakeDurationUnit << 5;
  // Last 2 bits are reserved
  return res;
}

void 
Twt::SetControlOctet (uint8_t control)
{
  m_ndpPagingIndicator = control & 0x01;
  m_responderPm = (control >> 1) & 0x01;
  m_negotiationType = (control >> 2) & 0x03;
  m_twtInfoFrameDisabled = (control >> 4) & 0x01;
  m_wakeDurationUnit = (control >> 5) & 0x01;
}

uint16_t 
Twt::GetRequestType (void) const
{
  uint16_t res = 0;
  res |= m_twtRequest;
  res |= m_twtSetupCommand << 1;
  res |= m_trigger << 4;
  res |= m_implicit << 5;
  res |= m_flowType << 6;
  res |= m_flowId << 7;
  res |= m_twtWakeIntervalExponent << 10;
  res |= m_twtProtection << 15;
  return res;
}

void
Twt::SetRequestType (uint16_t request)
{
  m_twtRequest = request & 0x01;
  m_twtSetupCommand = (request >> 1) & 0x07;
  m_trigger = (request >> 4) & 0x01;
  m_implicit = (request >> 5) & 0x01;
  m_flowType = (request >> 6) & 0x01;
  m_flowId = (request >> 7) & 0x07;
  m_twtWakeIntervalExponent = (request >> 10) & 0x3f;
  m_twtProtection = (request >> 15) & 0x01;
}

void 
Twt::SetNdpPagingIndicator (bool ndpPagingIndicator)
{
  m_ndpPagingIndicator = ndpPagingIndicator;
}

void 
Twt::SetResponderPm (bool responderPm)
{
  m_responderPm = responderPm;
}

void
Twt::SetNegotiationType (uint8_t negotiationType)
{
  // should be 0 or 1
  NS_ASSERT_MSG (negotiationType == 0 || negotiationType == 1 , "Negotiation Type should be 0 or 1");
  m_negotiationType = negotiationType;
}

void
Twt::SetTwtInfoFrameDisabled (bool twtInfoFrameDisabled)
{
  m_twtInfoFrameDisabled = twtInfoFrameDisabled;
}

void
Twt::SetWakeDurationUnit (u_int8_t wakeDurationUnit)
{
  // should be 0 or 1
  NS_ASSERT_MSG (wakeDurationUnit == 0 || wakeDurationUnit == 1 , "Wake Duration Unit should be 0 or 1");
  m_wakeDurationUnit = wakeDurationUnit;
}

void
Twt::SetTwtRequest (bool twtRequest)
{
  m_twtRequest = twtRequest;
}

void
Twt::SetTwtSetupCommand (uint8_t twtSetupCommand)
{
  NS_ASSERT_MSG (twtSetupCommand >= 0 && twtSetupCommand <= 7, "TWT Setup Command should be >=0 and <=7");
  m_twtSetupCommand = twtSetupCommand;
}

void
Twt::SetTrigger (bool trigger)
{
  m_trigger = trigger;
}

void
Twt::SetImplicit (bool implicit)
{
  m_implicit = implicit;
}

void
Twt::SetFlowType (bool flowType)
{
  m_flowType = flowType;
}

void
Twt::SetFlowId (uint8_t flowId)
{
  NS_ASSERT_MSG (flowId >= 0 && flowId <= 7, "Flow ID should be >=0 and <=7");
  m_flowId = flowId;
}

void
Twt::SetTwtWakeIntervalExponent (uint8_t twtWakeIntervalExponent)
{
  NS_ASSERT_MSG (twtWakeIntervalExponent >= 0 && twtWakeIntervalExponent <= 31, "TWT Wake Interval Exponent should be >=0 and <=63");
  m_twtWakeIntervalExponent = twtWakeIntervalExponent;
}

void
Twt::SetTwtProtection (bool twtProtection)
{
  m_twtProtection = twtProtection;
}

void
Twt::SetTargetWakeTime (uint64_t targetWakeTime)
{
  m_targetWakeTime = targetWakeTime;
}

void
Twt::SetTwtGroupAssignment (uint64_t twtGroupAssignment)
{
  NS_ASSERT_MSG (twtGroupAssignment >= 0 && twtGroupAssignment <= 7, "TWT Group Assignment should be >=0 and <=7");
  m_twtGroupAssignment = twtGroupAssignment;
}

void
Twt::SetNominalWakeDuration (uint8_t nominalWakeDuration)
{
  NS_ASSERT_MSG (nominalWakeDuration >= 0 && nominalWakeDuration <= 255, "Nominal Wake Duration should be >=0 and <=255");
  m_nominalWakeDuration = nominalWakeDuration;
}

void
Twt::SetTwtWakeIntervalMantissa (uint16_t twtWakeIntervalMantissa)
{
  m_twtWakeIntervalMantissa = twtWakeIntervalMantissa;
}

void
Twt::SetTwtChannel (uint8_t twtChannel)
{
  m_twtChannel = twtChannel;
}

// Getters

bool
Twt::GetNdpPagingIndicator (void) const
{
  return m_ndpPagingIndicator == 1;
}

bool
Twt::GetResponderPm (void) const
{
  return m_responderPm == 1;
}

uint8_t
Twt::GetNegotiationType (void) const
{
  return m_negotiationType;
}

bool
Twt::GetTwtInfoFrameDisabled (void) const
{
  return m_twtInfoFrameDisabled == 1;
}

bool
Twt::GetWakeDurationUnit (void) const
{
  return m_wakeDurationUnit == 1;
}

bool
Twt::GetTwtRequest (void) const
{
  return m_twtRequest == 1;
}

uint8_t
Twt::GetTwtSetupCommand (void) const
{
  return m_twtSetupCommand;
}

bool
Twt::IsTriggerBased (void) const
{
  return m_trigger == 1;
}

bool
Twt::IsImplicit (void) const
{
  return m_implicit == 1;
}

bool
Twt::GetFlowType (void) const
{
  return m_flowType == 1;
}

uint8_t
Twt::GetFlowId (void) const
{
  return m_flowId;
}


uint8_t
Twt::GetTwtWakeIntervalExponent (void) const
{
  return m_twtWakeIntervalExponent;
}

bool
Twt::GetTwtProtection (void) const
{
  return m_twtProtection == 1;
}

uint64_t
Twt::GetTargetWakeTime (void) const
{
  return m_targetWakeTime;
}

uint64_t
Twt::GetTwtGroupAssignment (void) const
{
  return m_twtGroupAssignment;
}

uint8_t
Twt::GetNominalWakeDuration (void) const
{
  return m_nominalWakeDuration;
}

uint16_t
Twt::GetTwtWakeIntervalMantissa (void) const
{
  return m_twtWakeIntervalMantissa;
}

uint8_t
Twt::GetTwtChannel (void) const
{
  return m_twtChannel;
}


std::ostream &
operator << (std::ostream &os, const Twt &twt)
{
  os << "NDP Paging Indicator: " << twt.GetNdpPagingIndicator () << std::endl;
  os << "Responder PM: " << twt.GetResponderPm () << std::endl;
  os << "Negotiation Type: " << +twt.GetNegotiationType () << std::endl;
  os << "TWT Info Frame Disabled: " << twt.GetTwtInfoFrameDisabled () << std::endl;
  os << "Wake Duration Unit: " << twt.GetWakeDurationUnit () << std::endl;
  os << "TWT Request: " << twt.GetTwtRequest () << std::endl;
  os << "TWT Setup Command: " << +twt.GetTwtSetupCommand () << std::endl;
  os << "Trigger: " << twt.IsTriggerBased () << std::endl;
  os << "Implicit: " << twt.IsImplicit () << std::endl;
  os << "Flow Type: " << +twt.GetFlowType () << std::endl;
  os << "Flow ID: " << +twt.GetFlowId () << std::endl;
  os << "TWT Wake Interval Exponent: " << +twt.GetTwtWakeIntervalExponent () << std::endl;
  os << "TWT Protection: " << twt.GetTwtProtection () << std::endl;
  os << "Target Wake Time: " << twt.GetTargetWakeTime () << std::endl;
  // os << "TWT Group Assignment: " << twt.GetTwtGroupAssignment () << std::endl;
  os << "Nominal Wake Duration: " << +twt.GetNominalWakeDuration () << std::endl;
  os << "TWT Wake Interval Mantissa: " << twt.GetTwtWakeIntervalMantissa () << std::endl;
  os << "TWT Channel: " << +twt.GetTwtChannel () << std::endl;
  return os;
}


} //namespace ns3
