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

#ifndef MCS_TAG_H
#define MCS_TAG_H

#include "ns3/tag.h"

#define UNSET_MCS -20

namespace ns3
{

class Tag;

class McsTag : public Tag
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Create a McsTag with an initial MCS equal to 0 and m_mcs_valid equal to false (a valid MCS has not been set yet)
     */
    McsTag();

    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    /**
     * Set the MCS to a given value
     *
     * \param mcs the value of the MCS
     */
    void SetMcs(uint8_t mcs);
    /**
     * Return the MCS value.
     *
     * \return the MCS value
     */
    uint8_t GetMcs() const;
    /**
     * Return whether the MCS stored in this Tag is available/valid or not
     *
     * \param mcs the value of the MCS
     */
    bool isMcsAvailable() {return m_mcs_available;}
    /**
     * Mark the current MCS as unavailable
     */
    void markUnavailable();
    /**
     * Mark the current MCS as available
     */
    void markAvailable();

  private:
    uint8_t m_mcs; //!< MCS value
    bool m_mcs_available;
};

} // namespace ns3

#endif /* SNR_TAG_H */
