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

#include "rxsignalinfo-tag.h"

#include "ns3/double.h"

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(RxSignalInfoTag);

TypeId
RxSignalInfoTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::RxSignalInfoTag")
                            .SetParent<Tag>()
                            .SetGroupName("Wifi")
                            .AddConstructor<RxSignalInfoTag>()
                            .AddAttribute("Snr",
                                          "The SNR of the last packet received (linear scale)",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&RxSignalInfoTag::GetSnr),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("RSSI",
                                          "The RSSI of the last packet received",
                                          DoubleValue(RSSI_INVALID),
                                          MakeDoubleAccessor(&RxSignalInfoTag::GetRSSI),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Snr_db",
                                          "The SNR of the last packet received (in dB)",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&RxSignalInfoTag::GetSnrdB),
                                          MakeDoubleChecker<double>());
    return tid;
}

TypeId
RxSignalInfoTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

RxSignalInfoTag::RxSignalInfoTag()
    : m_snr(0), m_rssi(RSSI_INVALID), m_snr_db(0)
{
}

uint32_t
RxSignalInfoTag::GetSerializedSize() const
{
    return sizeof(double)+sizeof(double)+sizeof(double);
}

void
RxSignalInfoTag::Serialize(TagBuffer i) const
{
    i.WriteDouble(m_snr);
    i.WriteDouble(m_rssi);
    i.WriteDouble(m_snr_db);
}

void
RxSignalInfoTag::Deserialize(TagBuffer i)
{
    m_snr = i.ReadDouble();
    m_rssi = i.ReadDouble ();
    m_snr_db = i.ReadDouble ();
}

void
RxSignalInfoTag::Print(std::ostream& os) const
{
    os << "Snr=" << m_snr << " - RSSI=" << m_rssi;
}

void
RxSignalInfoTag::SetSnr(double snr)
{
    m_snr = snr;
}

void
RxSignalInfoTag::SetSnrdB(double snr_db)
{
    m_snr_db = snr_db;
}


void
RxSignalInfoTag::SetRSSI(double rssi)
{
    m_rssi = rssi;
}

double
RxSignalInfoTag::GetSnr() const
{
    return m_snr;
}

double
RxSignalInfoTag::GetSnrdB() const
{
    return m_snr_db;
}

double
RxSignalInfoTag::GetRSSI() const
{
    return m_rssi;
}

} // namespace ns3
