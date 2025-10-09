/*
 * Copyright (c) 2005,2006 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
 * Copyright (c) 2013 University of Surrey
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
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 *          Konstantinos Katsaros <dinos.katsaros@gmail.com>
 *          Francesco Raviglione <francescorav.es483@gmail.com>
 */

#include "mcs-tag.h"

#include "ns3/integer.h"

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(McsTag);

TypeId
McsTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::McsTag")
                            .SetParent<Tag>()
                            .SetGroupName("Wifi")
                            .AddConstructor<McsTag>()
                            .AddAttribute("Mcs",
                                          "The MCS gathered from the TxVector",
                                          IntegerValue(UNSET_MCS),
                                          MakeIntegerAccessor(&McsTag::GetMcs),
                                          MakeIntegerChecker<int>());
    return tid;
}

TypeId
McsTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

McsTag::McsTag()
    : m_mcs(0), m_mcs_available(false)
{
}

uint32_t
McsTag::GetSerializedSize() const
{
    return sizeof(uint8_t);
}

void
McsTag::Serialize(TagBuffer i) const
{
    i.WriteU8(m_mcs);
}

void
McsTag::Deserialize(TagBuffer i)
{
    m_mcs = i.ReadU8 ();
}

void
McsTag::Print(std::ostream& os) const
{
    os << "MCS=" << (int)(m_mcs);
}

void
McsTag::SetMcs(uint8_t mcs)
{
    m_mcs = mcs;
    m_mcs_available = true;
}

uint8_t
McsTag::GetMcs() const
{
    return m_mcs;
}

void
McsTag::markUnavailable ()
{
    m_mcs_available = false;
}

void
McsTag::markAvailable ()
{
    m_mcs_available = true;
}

} // namespace ns3
