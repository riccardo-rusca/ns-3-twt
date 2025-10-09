/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mohamed Amine Ismail <amine.ismail@sophia.inria.fr>
 */
#include "udp-measurement-client-server-helper.h"

#include "ns3/string.h"
#include "ns3/udp-measurement-client.h"
#include "ns3/udp-measurement-server.h"
#include "ns3/uinteger.h"

namespace ns3
{

UdpMeasurementServerHelper::UdpMeasurementServerHelper()
{
    m_factory.SetTypeId(UdpMeasurementServer::GetTypeId());
}

UdpMeasurementServerHelper::UdpMeasurementServerHelper(uint16_t port)
{
    m_factory.SetTypeId(UdpMeasurementServer::GetTypeId());
    SetAttribute("Port", UintegerValue(port));
}

void
UdpMeasurementServerHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
UdpMeasurementServerHelper::Install(NodeContainer c)
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Node> node = *i;

        m_server = m_factory.Create<UdpMeasurementServer>();
        node->AddApplication(m_server);
        apps.Add(m_server);
    }
    return apps;
}

Ptr<UdpMeasurementServer>
UdpMeasurementServerHelper::GetServer()
{
    return m_server;
}

UdpMeasurementClientHelper::UdpMeasurementClientHelper()
{
    m_factory.SetTypeId(UdpMeasurementClient::GetTypeId());
}

UdpMeasurementClientHelper::UdpMeasurementClientHelper(Address address, uint16_t port)
{
    m_factory.SetTypeId(UdpMeasurementClient::GetTypeId());
    SetAttribute("RemoteAddress", AddressValue(address));
    SetAttribute("RemotePort", UintegerValue(port));
}

UdpMeasurementClientHelper::UdpMeasurementClientHelper(Address address)
{
    m_factory.SetTypeId(UdpMeasurementClient::GetTypeId());
    SetAttribute("RemoteAddress", AddressValue(address));
}

void
UdpMeasurementClientHelper::SetAttribute(std::string name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

ApplicationContainer
UdpMeasurementClientHelper::Install(NodeContainer c)
{
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        Ptr<Node> node = *i;
        Ptr<UdpMeasurementClient> client = m_factory.Create<UdpMeasurementClient>();
        node->AddApplication(client);
        apps.Add(client);
    }
    return apps;
}

} // namespace ns3
