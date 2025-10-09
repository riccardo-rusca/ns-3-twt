/*
 * Copyright (c) 2007,2008,2009 INRIA, UDCAST
 * Copyright (c) 2023 Politecnico di Torino
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
 * Author: Amine Ismail <amine.ismail@sophia.inria.fr>
 *                      <amine.ismail@udcast.com>
 *
 * Author: Francesco Raviglione <francescorav.es483@gmail.com>
 */
#include "udp-measurement-client.h"

#include "latency-comp-header.h"

#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UdpMeasurementClient");

NS_OBJECT_ENSURE_REGISTERED(UdpMeasurementClient);

TypeId
UdpMeasurementClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UdpMeasurementClient")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpMeasurementClient>()
            .AddAttribute(
                "MaxPackets",
                "The maximum number of packets the application will send (zero means infinite)",
                UintegerValue(100),
                MakeUintegerAccessor(&UdpMeasurementClient::m_count),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&UdpMeasurementClient::m_interval),
                          MakeTimeChecker())
            .AddAttribute("RemoteAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&UdpMeasurementClient::m_peerAddress),
                          MakeAddressChecker())
            .AddAttribute("RemotePort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&UdpMeasurementClient::m_peerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketSize",
                          "Size of packets generated. The minimum packet size is 12 bytes which is "
                          "the size of the header carrying the sequence number and the time stamp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&UdpMeasurementClient::m_size),
                          MakeUintegerChecker<uint32_t>(12, 65507))
            .AddAttribute("ReportManager",
                          "Assign a ReportManager for advanced metrics computation (latency, RSSI, SNR, packet loss, ...)",
                          PointerValue(nullptr),
                          MakePointerAccessor(&UdpMeasurementClient::m_reportman),
                          MakePointerChecker<ReportManager>())
            .AddAttribute("t0",
                          "t0 value for the client",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&UdpMeasurementClient::m_t0),
                          MakeDoubleChecker<double>())
            .AddAttribute("t1",
                          "t1 value for the client",
                          DoubleValue(0),
                          MakeDoubleAccessor(&UdpMeasurementClient::m_t1),
                          MakeDoubleChecker<double>())
            .AddAttribute("twtDuration",
                          "twtDuration value for the client",
                          DoubleValue(0),
                          MakeDoubleAccessor(&UdpMeasurementClient::m_twtDuration),
                          MakeDoubleChecker<double>())
            .AddTraceSource("Tx",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&UdpMeasurementClient::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("TxWithAddresses",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&UdpMeasurementClient::m_txTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

UdpMeasurementClient::UdpMeasurementClient()
{
    NS_LOG_FUNCTION(this);
    m_sent = 0;
    m_totalTx = 0;
    m_socket = nullptr;
    m_sendSingleEvent = EventId();
}

UdpMeasurementClient::~UdpMeasurementClient()
{
    NS_LOG_FUNCTION(this);
}

void
UdpMeasurementClient::SetRemote(Address ip, uint16_t port)
{
    NS_LOG_FUNCTION(this << ip << port);
    m_peerAddress = ip;
    m_peerPort = port;
}

void
UdpMeasurementClient::SetRemote(Address addr)
{
    NS_LOG_FUNCTION(this << addr);
    m_peerAddress = addr;
}

void
UdpMeasurementClient::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Application::DoDispose();
}

void
UdpMeasurementClient::StartApplication()
{
    NS_LOG_FUNCTION(this);

    if(m_t0>m_t1)
    {
      NS_FATAL_ERROR("Fatal error. Specified a t0 which is greater than t1, and this is not possible. Aborting simulation.");
    }

    if (!m_socket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket = Socket::CreateSocket(GetNode(), tid);
        if (Ipv4Address::IsMatchingType(m_peerAddress))
        {
            if (m_socket->Bind() == -1)
            {
                NS_FATAL_ERROR("Failed to bind socket");
            }
            m_socket->Connect(
                InetSocketAddress(Ipv4Address::ConvertFrom(m_peerAddress), m_peerPort));
        }
        else if (Ipv6Address::IsMatchingType(m_peerAddress))
        {
            if (m_socket->Bind6() == -1)
            {
                NS_FATAL_ERROR("Failed to bind socket");
            }
            m_socket->Connect(
                Inet6SocketAddress(Ipv6Address::ConvertFrom(m_peerAddress), m_peerPort));
        }
        else if (InetSocketAddress::IsMatchingType(m_peerAddress))
        {
            if (m_socket->Bind() == -1)
            {
                NS_FATAL_ERROR("Failed to bind socket");
            }
            m_socket->Connect(m_peerAddress);
        }
        else if (Inet6SocketAddress::IsMatchingType(m_peerAddress))
        {
            if (m_socket->Bind6() == -1)
            {
                NS_FATAL_ERROR("Failed to bind socket");
            }
            m_socket->Connect(m_peerAddress);
        }
        else
        {
            NS_ASSERT_MSG(false, "Incompatible address type: " << m_peerAddress);
        }
    }

#ifdef NS3_LOG_ENABLE
    std::stringstream peerAddressStringStream;
    if (Ipv4Address::IsMatchingType(m_peerAddress))
    {
        peerAddressStringStream << Ipv4Address::ConvertFrom(m_peerAddress);
    }
    else if (Ipv6Address::IsMatchingType(m_peerAddress))
    {
        peerAddressStringStream << Ipv6Address::ConvertFrom(m_peerAddress);
    }
    else if (InetSocketAddress::IsMatchingType(m_peerAddress))
    {
        peerAddressStringStream << InetSocketAddress::ConvertFrom(m_peerAddress).GetIpv4();
    }
    else if (Inet6SocketAddress::IsMatchingType(m_peerAddress))
    {
        peerAddressStringStream << Inet6SocketAddress::ConvertFrom(m_peerAddress).GetIpv6();
    }
    m_peerAddressString = peerAddressStringStream.str();
#endif // NS3_LOG_ENABLE

    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    m_socket->SetAllowBroadcast(true);

    m_sendSingleEvent = Simulator::Schedule(MilliSeconds(m_t0),&UdpMeasurementClient::SendSingle, this);
    // SendSingle();
}

void
UdpMeasurementClient::StopApplication()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendSingleEvent);
    m_socket->Close ();
}

void
UdpMeasurementClient::SendSingle()
{
  NS_LOG_FUNCTION(this);
  NS_ASSERT(m_sendSingleEvent.IsExpired());

    Address from;
    Address to;
    m_socket->GetSockName(from);
    m_socket->GetPeerName(to);

//    InetSocketAddress sdfrom=InetSocketAddress::ConvertFrom(from);
//    std::cout << "From: " << sdfrom.GetIpv4() << std::endl;
//
//    InetSocketAddress sdto=InetSocketAddress::ConvertFrom(to);
//    std::cout << "To: " << sdto.GetIpv4() << std::endl;

    LatencyCompHeader seqTs;
    seqTs.SetSeq(m_sent);
    NS_ABORT_IF(m_size < seqTs.GetSerializedSize());
    Ptr<Packet> single_packet = Create<Packet>(m_size - seqTs.GetSerializedSize());

    m_txTrace(single_packet);
    m_txTraceWithAddresses(single_packet, from, to);

    single_packet->AddHeader(seqTs);

    std::cout << "TX STA ID: " << m_peerPort-49000 << std::endl;

    int socket_success=m_socket->Send(single_packet);
    if (socket_success >= 0) {
        if (m_reportman != nullptr) {
            m_reportman->updateReportTx(single_packet->GetSize());
            m_reportman->set_last_t0(m_t0);
            m_reportman->set_last_t1(m_t1);
            double teff = static_cast<double>(Simulator::Now().GetNanoSeconds()) / 1000000.0; // Dividing by 1e6 as teff is measured in [ms]

            printf("Time packet sent to MAC: %.2lf ms.\n",Simulator::Now().GetNanoSeconds()/1000000.0);
            m_reportman->set_last_teff(teff);
            m_reportman->set_last_wait_time_slots(m_slot_wait_counter);
        }
        ++m_sent;
        m_totalTx += single_packet->GetSize();
        m_slot_wait_counter = 0; // Reset the counter couting how many slots we have been waiting before being able to transmit the packet
    }
}

uint64_t
UdpMeasurementClient::GetTotalTx() const
{
    return m_totalTx;
}

} // Namespace ns3
