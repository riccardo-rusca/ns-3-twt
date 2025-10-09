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

#ifndef RXSIGNALINFO_TAG_H
#define RXSIGNALINFO_TAG_H

#include "ns3/tag.h"

#define RSSI_INVALID -2000.0

namespace ns3
{

class Tag;

class RxSignalInfoTag : public Tag
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Create a RxSignalInfoTag with the default SNR 0 and RSSI -2000 (RSSI_INVALID)
     */
    RxSignalInfoTag();

    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    /**
     * Set the SNR to the given value.
     *
     * \param snr the value of the SNR to set in linear scale
     */
    void SetSnr(double snr);
    /**
     * Set the SNR to the given value.
     *
     * \param snr the value of the SNR to set in dB
     */
    void SetSnrdB(double snr_db);
    /**
     * Set the RSSI to the given value.
     *
     * \param snr the value of the SNR to set in linear scale
     */
    void SetRSSI(double rssi);
    /**
     * Return the SNR value.
     *
     * \return the SNR value in linear scale
     */
    double GetSnr() const;
    /**
     * Return the SNR value.
     *
     * \return the SNR value in dB
     */
    double GetSnrdB() const;
    /**
     * Return the RSSI value.
     *
     * \return the RSSI value in dBm
     */
    double GetRSSI() const;


  private:
    double m_snr; //!< SNR value in linear scale
    double m_rssi; //!< RSSI in dBm
    double m_snr_db; //!< SNR value in db
};

} // namespace ns3

#endif /* SNR_TAG_H */
