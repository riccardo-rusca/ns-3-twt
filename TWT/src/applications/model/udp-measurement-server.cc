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

#include "udp-measurement-server.h"

#include "packet-loss-counter.h"
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
#include "ns3/rxsignalinfo-tag.h"
#include "ns3/mcs-tag.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UdpMeasurementServer");

NS_OBJECT_ENSURE_REGISTERED(UdpMeasurementServer);

TypeId
UdpMeasurementServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UdpMeasurementServer")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpMeasurementServer>()
            .AddAttribute("Port",
                          "Port on which we listen for incoming packets.",
                          UintegerValue(100),
                          MakeUintegerAccessor(&UdpMeasurementServer::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketWindowSize",
                          "The size of the window used to compute the packet loss. This value "
                          "should be a multiple of 8.",
                          UintegerValue(32),
                          MakeUintegerAccessor(&UdpMeasurementServer::GetPacketWindowSize,
                                               &UdpMeasurementServer::SetPacketWindowSize),
                          MakeUintegerChecker<uint16_t>(8, 256))
            .AddAttribute("ReportManager",
                          "Assign a ReportManager for advanced metrics computation (latency, RSSI, SNR, packet loss, ...)",
                          PointerValue(nullptr),
                          MakePointerAccessor(&UdpMeasurementServer::m_reportman),
                          MakePointerChecker<ReportManager>())
            .AddAttribute("STAClass",
                            "The STA class from which this server is receiving data (optional)",
                            UintegerValue(0),
                            MakeUintegerAccessor(&UdpMeasurementServer::m_class),
                            MakeUintegerChecker<unsigned int>(0, 255))
            .AddAttribute("RxTimeLimit",
                          "Set a limit time after which packets will be discarded by the server"
                          "[advanced option: use it only if you really know what you ae doing]",
                          DoubleValue(-1.0),
                          MakeDoubleAccessor(&UdpMeasurementServer::m_rx_limit),
                          MakeDoubleChecker<double>())
            .AddTraceSource("Rx",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpMeasurementServer::m_rxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("RxWithAddresses",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpMeasurementServer::m_rxTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

UdpMeasurementServer::UdpMeasurementServer()
    : m_lossCounter(0)
{
    NS_LOG_FUNCTION(this);
    m_received = 0;
}

UdpMeasurementServer::~UdpMeasurementServer()
{
    NS_LOG_FUNCTION(this);
}

uint16_t
UdpMeasurementServer::GetPacketWindowSize() const
{
    NS_LOG_FUNCTION(this);
    return m_lossCounter.GetBitMapSize();
}

void
UdpMeasurementServer::SetPacketWindowSize(uint16_t size)
{
    NS_LOG_FUNCTION(this << size);
    m_lossCounter.SetBitMapSize(size);
}

uint32_t
UdpMeasurementServer::GetLost() const
{
    NS_LOG_FUNCTION(this);
    return m_lossCounter.GetLost();
}

uint64_t
UdpMeasurementServer::GetReceived() const
{
    NS_LOG_FUNCTION(this);
    return m_received;
}

void
UdpMeasurementServer::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Application::DoDispose();
}

void
UdpMeasurementServer::StartApplication()
{
    NS_LOG_FUNCTION(this);

    if(m_class>0)
    {
      m_port=49000+m_class;
    }

    if (!m_socket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket = Socket::CreateSocket(GetNode(), tid);
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        if (m_socket->Bind(local) == -1)
        {
            NS_FATAL_ERROR("Failed to bind socket");
        }
    }

    m_socket->SetRecvCallback(MakeCallback(&UdpMeasurementServer::HandleRead, this));

    if (!m_socket6)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_socket6 = Socket::CreateSocket(GetNode(), tid);
        Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_port);
        if (m_socket6->Bind(local) == -1)
        {
            NS_FATAL_ERROR("Failed to bind socket");
        }
    }

    m_socket6->SetRecvCallback(MakeCallback(&UdpMeasurementServer::HandleRead, this));
}

void
UdpMeasurementServer::StopApplication()
{
    NS_LOG_FUNCTION(this);

    if (m_socket)
    {
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        m_socket->Close ();
    }
}

void
UdpMeasurementServer::HandleRead(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    Ptr<Packet> packet;
    Address from;
    Address localAddress;
    while ((packet = socket->RecvFrom(from)))
    {

        socket->GetSockName(localAddress);
        m_rxTrace(packet);
        m_rxTraceWithAddresses(packet, from, localAddress);

        if (packet->GetSize() > 0)
        {
//            if(m_metricssup!=nullptr)
//            {
//              m_metricssup->signalReceivedPacket(packet);
//            }

            uint32_t receivedSize = packet->GetSize();
            LatencyCompHeader seqTs;
            packet->RemoveHeader(seqTs);
            uint32_t currentSequenceNumber = seqTs.GetSeq();

            std::cout << "RX STA ID: " << m_port-49000 << std::endl;

            // Compute the current latency
            double curr_latency_ms = static_cast<double>(Simulator::Now ().GetNanoSeconds () - seqTs.GetNanosecondsTs())/1000000.0;

            printf("Rx time: %.2lf ms | Latency: %.2lf ms | TX time: %.2lf ms | TWT window start: %.2lf ms | Effective latency (latency-t1): %.2lf ms.\n",
                   Simulator::Now().GetNanoSeconds()/1000000.0,
                   curr_latency_ms,
                   (seqTs.GetNanosecondsTs()/1000000.0),
                   m_reportman->get_last_t1(),
                   Simulator::Now().GetNanoSeconds()/1000000.0-m_reportman->get_last_t1()-1024.0);

            // If RxTimeLimit has been specified and the packet arrives after the limit, discard it and save approximately how many Bytes could have been left for transmission
            double shifted_rx_limit=m_reportman->get_time_reference_shift_ms()+m_rx_limit;

            if(m_rx_limit>0.0) std::cout << "Shifted RxTimeLimit: " << shifted_rx_limit << " ms." << std::endl;

            if(m_rx_limit>0.0 && Simulator::Now ().GetNanoSeconds ()/1000000.0>shifted_rx_limit) {
                double trx=Simulator::Now ().GetNanoSeconds ()/1000000.0-m_reportman->get_time_reference_shift_ms(); // With respect to when the server started
                int bytes_left=std::floor(packet->GetSize()*(trx-m_rx_limit)/(trx-m_reportman->get_last_teff ()));
                m_reportman->set_time_limit_rem_data (m_class,bytes_left); // Estimated number of remaining bytes with a simple proportion: <total B>*(trx-m_rx_limit)/(trx-teff)
                std::cout << "Server reported that a packet from a client exceeded the limit of " << m_rx_limit  << " ms. Packet discarded with estimated " << bytes_left << " B left." << std::endl;
                return;
            }

            if(m_reportman!=nullptr) {
              // Get the SNR and RSSI for the packet that has been been just received
              double curr_snr=DBL_MAX;
              double curr_rssi=DBL_MAX;
              RxSignalInfoTag rxsignalinfotag;
              if(packet->PeekPacketTag(rxsignalinfotag)) {
                  curr_snr = rxsignalinfotag.GetSnrdB();
                  curr_rssi = rxsignalinfotag.GetRSSI();

//                  std::cout << "SNR: " << curr_snr << std::endl;
//                  std::cout << "RSSI: " << curr_rssi << std::endl;
              }

              McsTag mcstag;
              int mcs;
              if(packet->PeekPacketTag(mcstag)) {
                mcs=mcstag.GetMcs ();
              } else {
                mcs=-1;
              }

              // Update the report
              m_reportman->updateReportRx(curr_latency_ms,receivedSize,curr_snr,curr_rssi,mcs);
              m_reportman->set_last_trx(Simulator::Now ().GetNanoSeconds ()/1000000.0); // Dividing by 1e6 as t_rx is measured in [ms]
           }

            if (InetSocketAddress::IsMatchingType(from))
            {
                NS_LOG_INFO("TraceDelay: RX " << receivedSize << " bytes from "
                                              << InetSocketAddress::ConvertFrom(from).GetIpv4()
                                              << " Sequence Number: " << currentSequenceNumber
                                              << " Uid: " << packet->GetUid() << " TXtime: "
                                              << seqTs.GetTs() << " RXtime: " << Simulator::Now()
                                              << " Delay [ms]: " << curr_latency_ms);
            }
            else if (Inet6SocketAddress::IsMatchingType(from))
            {
                NS_LOG_INFO("TraceDelay: RX " << receivedSize << " bytes from "
                                              << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
                                              << " Sequence Number: " << currentSequenceNumber
                                              << " Uid: " << packet->GetUid() << " TXtime: "
                                              << seqTs.GetTs() << " RXtime: " << Simulator::Now()
                                              << " Delay [ms]: " << curr_latency_ms);
            }

            m_lossCounter.NotifyReceived(currentSequenceNumber);
            m_received++;
        }
    }
}

} // Namespace ns3
