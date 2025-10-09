/*
 * Copyright (c) 2009 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Francesco Raviglione <francescorav.es483@gmail.com>
 */

#include "latency-comp-header.h"

#include "ns3/assert.h"
#include "ns3/header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LatencyCompHeader");

NS_OBJECT_ENSURE_REGISTERED(LatencyCompHeader);

LatencyCompHeader::LatencyCompHeader()
    : m_seq(0),
      m_ts(Simulator::Now ().GetNanoSeconds ())
{
    NS_LOG_FUNCTION(this);
}

void
LatencyCompHeader::SetSeq(uint32_t seq)
{
    NS_LOG_FUNCTION(this << seq);
    m_seq = seq;
}

uint32_t
LatencyCompHeader::GetSeq() const
{
    NS_LOG_FUNCTION(this);
    return m_seq;
}

Time
LatencyCompHeader::GetTs() const
{
    NS_LOG_FUNCTION(this);
    return TimeStep(m_ts);
}

uint64_t
LatencyCompHeader::GetNanosecondsTs () const
{
    NS_LOG_FUNCTION(this);
    return m_ts;
}

TypeId
LatencyCompHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LatencyCompHeader")
                            .SetParent<Header>()
                            .SetGroupName("Applications")
                            .AddConstructor<LatencyCompHeader>();
    return tid;
}

TypeId
LatencyCompHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
LatencyCompHeader::Print(std::ostream& os) const
{
    NS_LOG_FUNCTION(this << &os);
    os << "(seq=" << m_seq << " time=" << TimeStep(m_ts).As(Time::NS) << ")";
}

uint32_t
LatencyCompHeader::GetSerializedSize() const
{
    NS_LOG_FUNCTION(this);
    return 4 + 8;
}

void
LatencyCompHeader::Serialize(Buffer::Iterator start) const
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    i.WriteHtonU32(m_seq);
    i.WriteHtonU64(m_ts);
}

uint32_t
LatencyCompHeader::Deserialize(Buffer::Iterator start)
{
    NS_LOG_FUNCTION(this << &start);
    Buffer::Iterator i = start;
    m_seq = i.ReadNtohU32();
    m_ts = i.ReadNtohU64();
    return GetSerializedSize();
}

} // namespace ns3
