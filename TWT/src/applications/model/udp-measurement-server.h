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

#ifndef UDP_MEASUREMENT_SERVER_H
#define UDP_MEASUREMENT_SERVER_H

#include "packet-loss-counter.h"

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"
#include "ns3/ReportManager.h"

#include <limits>
#include <cfloat>
#include <map>

namespace ns3
{
/**
 * \ingroup applications
 * \defgroup udpmeasurementclientserver UdpMeasurementClientServer
 */

/**
 * \ingroup udpmeasurementclientserver
 *
 * \brief A UDP server, receives UDP packets from a remote host.
 *
 * UDP packets carry a 32bits sequence number followed by a 64bits time
 * stamp in their payloads. The application uses the sequence number
 * to determine if a packet is lost, and the time stamp to compute the delay.
 */
class UdpMeasurementServer : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    UdpMeasurementServer();
    ~UdpMeasurementServer() override;
    /**
     * \brief Returns the number of lost packets
     * \return the number of lost packets
     */
    uint32_t GetLost() const;
    double getLostPercentage() {return static_cast<double>(GetLost())*100.0/static_cast<double>(GetLost()+m_received);}

    /**
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetReceived() const;

    /**
     * \brief Returns the size of the window used for checking loss.
     * \return the size of the window used for checking loss.
     */
    uint16_t GetPacketWindowSize() const;

    /**
     * \brief Set the size of the window used for checking loss. This value should
     *  be a multiple of 8
     * \param size the size of the window used for checking loss. This value should
     *  be a multiple of 8
     */
    void SetPacketWindowSize(uint16_t size);

    void setReportManager(Ptr<ReportManager> ptr_reportman) {m_reportman=ptr_reportman;}

    unsigned int getSTAClass(void) {return m_class;}

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * \brief Handle a packet reception.
     *
     * This function is called by lower layers.
     *
     * \param socket the socket the packet was received to.
     */
    void HandleRead(Ptr<Socket> socket);

    //    Ptr<MetricsSupervisor> m_metricssup=nullptr;
    Ptr<ReportManager> m_reportman=nullptr;

    uint16_t m_port;                 //!< Port on which we listen for incoming packets.
    Ptr<Socket> m_socket;            //!< IPv4 Socket
    Ptr<Socket> m_socket6;           //!< IPv6 Socket
    uint64_t m_received;             //!< Number of received packets
    PacketLossCounter m_lossCounter; //!< Lost packet counter

    /// Callbacks for tracing the packet Rx events
    TracedCallback<Ptr<const Packet>> m_rxTrace;

    /// Callbacks for tracing the packet Rx events, includes source and destination addresses
    TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_rxTraceWithAddresses;

    /// Current traffic and STA class from which the server is receiving its data
    // 0 means that no class is specified
    // If a class is specified, the port will be forced to be equal to 49000 + class number (m_class) and any "Port" attribute setting will have no effect
    unsigned int m_class=0;

    double m_rx_limit=-1.0; // [ms] If set to something more than 0, packets received after rx_limit ms SINCE THE SERVER STARTED (not absolute simulation time!) will be discarded
};

} // namespace ns3

#endif /* UDP_MEASUREMENT_SERVER_H */
