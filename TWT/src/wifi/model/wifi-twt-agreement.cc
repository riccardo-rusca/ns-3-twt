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


#include "wifi-twt-agreement.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("WifiTwtAgreement");

WifiTwtAgreement::WifiTwtAgreement(uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt)
    : m_flowId(flowId),
      m_peerMacAddress(peerMacAddress),
      m_isRequestingNode(isRequestingNode),
      m_isImplicitAgreement(isImplicitAgreement),
      m_flowType(flowType),
      m_isTriggerBasedAgreement(isTriggerBasedAgreement),
      m_isIndividualAgreement(isIndividualAgreement),
      m_twtChannel(twtChannel),
      m_wakeInterval(wakeInterval),
      m_nominalWakeDuration(nominalWakeDuration),
      m_nextTwt(nextTwt),
      m_nextServicePeriodStartEvent (),
      m_nextServicePeriodEndEvent ()
      {
        NS_LOG_FUNCTION(this << "flowId:" << static_cast<int>(flowId) <<"; peerMacAddress:" << peerMacAddress << "; isRequestingNode:" << isRequestingNode << "; isImplicitAgreement:" << isImplicitAgreement << "; flowType:" << flowType << "; isTriggerBasedAgreement:" << isTriggerBasedAgreement << "; isIndividualAgreement:" << isIndividualAgreement << "; twtChannel:" << twtChannel << "; wakeInterval:" << wakeInterval << "; nominalWakeDuration:" << nominalWakeDuration << "; nextTwt:" << nextTwt);
      }

WifiTwtAgreement::~WifiTwtAgreement()
{
    NS_LOG_FUNCTION(this);
    m_nextServicePeriodStartEvent.Cancel();
    m_nextServicePeriodEndEvent.Cancel();
}

uint8_t 
WifiTwtAgreement::GetFlowId() const 
{
  NS_LOG_FUNCTION (this);
  return m_flowId;
}

void 
WifiTwtAgreement::SetFlowId(uint8_t id) 
{
  NS_LOG_FUNCTION (this << +id);  // Using +id to ensure it's logged as a number
  m_flowId = id;
}


Mac48Address
WifiTwtAgreement::GetPeerMacAddress() const 
{
  NS_LOG_FUNCTION (this);
  return m_peerMacAddress;
}

void
WifiTwtAgreement::SetPeerMacAddress(Mac48Address macAddress) 
{
  NS_LOG_FUNCTION (this << macAddress);
  m_peerMacAddress = macAddress;
}

bool 
WifiTwtAgreement::IsRequestingNode() const 
{
  NS_LOG_FUNCTION (this);
  return m_isRequestingNode;
}


void 
WifiTwtAgreement::SetRequestingNode(bool value) 
{
  NS_LOG_FUNCTION (this << value);
  m_isRequestingNode = value;
}

bool 
WifiTwtAgreement::IsImplicitAgreement() const 
{
  NS_LOG_FUNCTION (this);
  return m_isImplicitAgreement;
}

void 
WifiTwtAgreement::SetImplicitAgreement(bool value) 
{
  NS_LOG_FUNCTION (this << value);
  m_isImplicitAgreement = value;
}

bool 
WifiTwtAgreement::GetFlowType() const 
{
  NS_LOG_FUNCTION (this);
  return m_flowType;
}

void 
WifiTwtAgreement::SetFlowType(bool type) 
{
  NS_LOG_FUNCTION (this << type);
  m_flowType = type;
}

bool 
WifiTwtAgreement::IsTriggerBasedAgreement() const 
{
  NS_LOG_FUNCTION (this);
  return m_isTriggerBasedAgreement;
}

void 
WifiTwtAgreement::SetTriggerBasedAgreement(bool value) 
{
  NS_LOG_FUNCTION (this << value);
  m_isTriggerBasedAgreement = value;
}

bool 
WifiTwtAgreement::IsIndividualAgreement() const 
{
  NS_LOG_FUNCTION (this);
  return m_isIndividualAgreement;
}

void 
WifiTwtAgreement::SetIndividualAgreement(bool value) 
{
  NS_LOG_FUNCTION (this << value);
  m_isIndividualAgreement = value;
}

u_int16_t 
WifiTwtAgreement::GetTwtChannel() const 
{
  NS_LOG_FUNCTION (this);
  return m_twtChannel;
}

void 
WifiTwtAgreement::SetTwtChannel(u_int16_t channel) 
{
  NS_LOG_FUNCTION (this << channel);
  m_twtChannel = channel;
}

Time 
WifiTwtAgreement::GetWakeInterval() const 
{
  NS_LOG_FUNCTION (this);
  return m_wakeInterval;
}

void 
WifiTwtAgreement::SetWakeInterval(const Time& interval) 
{
  NS_LOG_FUNCTION (this << interval);
  m_wakeInterval = interval;
}

Time 
WifiTwtAgreement::GetNominalWakeDuration() const 
{
  NS_LOG_FUNCTION (this);
  return m_nominalWakeDuration;
}

void 
WifiTwtAgreement::SetNominalWakeDuration(const Time& duration) 
{
  NS_LOG_FUNCTION (this << duration);
  m_nominalWakeDuration = duration;
}

Time 
WifiTwtAgreement::GetNextTwt() const 
{
  NS_LOG_FUNCTION (this);
  return m_nextTwt;
}

void 
WifiTwtAgreement::SetNextTwt(const Time& nextTwt) 
{
  NS_LOG_FUNCTION (this << nextTwt);
  m_nextTwt = nextTwt;
}



std::ostream& operator<<(std::ostream& os, const WifiTwtAgreement& twt)
{
    os << "Flow ID: " << static_cast<int>(twt.m_flowId) << "\n";
    os << "Peer MAC Address: " << twt.m_peerMacAddress << "\n";
    os << "Is Requesting Node: " << (twt.m_isRequestingNode ? "True" : "False") << "\n";
    os << "Implicit Agreement: " << (twt.m_isImplicitAgreement ? "True" : "False") << "\n";
    os << "Flow Type (True for Unannounced Agreement): " << (twt.m_flowType ? "True" : "False") << "\n";
    os << "Trigger Based Agreement: " << (twt.m_isTriggerBasedAgreement ? "True" : "False") << "\n";
    os << "Individual Agreement: " << (twt.m_isIndividualAgreement ? "True" : "False") << "\n";
    os << "TWT Channel: " << twt.m_twtChannel << "\n";
    os << "Wake Interval: " << twt.m_wakeInterval.GetSeconds() << " seconds\n";
    os << "Nominal Wake Duration: " << twt.m_nominalWakeDuration.GetSeconds() << " seconds\n";
    os << "Next TWT: " << twt.m_nextTwt.GetSeconds() << " seconds";

    return os;
}

}