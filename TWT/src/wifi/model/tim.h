/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 INRIA
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
 * Author: Shyam Krishnan Venkateswaran <shyam1@gatech.edu>
 */

#ifndef TIM_H
#define TIM_H

#include "wifi-information-element.h"

namespace ns3 {

/**
 * \brief The Traffic Indication Map Information Element
 * \ingroup wifi
 *
 * The IEEE 802.11 TIM Information Element class. It knows how to serialize
 * TIM element to be added to beacons. 
 *
 */
class Tim : public WifiInformationElement
{
public:
  /**
   * Create TIM element
   */
  Tim ();
  /**
   * Create TIM element given DTIM count, DTIM period, and multicast buffer status.
   * This function is called only if there are no unicast buffered packets.
   * 
   * \param dtimCount DTIM count as uint8_t
   * \param dtimPeriod DTIM period as uint8_t
   * \param isMulticastBuffered Are there multicast packets buffered?
   */
  Tim (uint8_t dtimCount, uint8_t dtimPeriod, bool isMulticastBuffered);
    
  /**
   * Create TIM element given DTIM count, DTIM period, multicast buffer status, fullAidBitmap [251 bytes], minAidInBuffer, and maxAidInBuffer
   * 
   * \param dtimCount DTIM count as uint8_t
   * \param dtimPeriod DTIM period as uint8_t
   * \param isMulticastBuffered Are there multicast packets buffered?
   * \param fullAidBitmap is 251 bytes long - the full bitmap for AID 0 to 2007
   * \param minAidInBuffer is the lowest AID for which there is unicast buffered traffic
   * \param maxAidInBuffer is the highest AID for which there is unicast buffered traffic
   */
  Tim (uint8_t dtimCount, uint8_t dtimPeriod, bool isMulticastBuffered, uint8_t* fullAidBitmap, uint16_t minAidInBuffer, uint16_t maxAidInBuffer);


/**
 * Return DTIM period
 * \return DTIM period 
 */
  uint8_t GetDtimPeriod(void) const;
/**
 * Return DTIM count
 * \return DTIM count 
 */
  uint8_t GetDtimCount(void) const;
/**
 * Return Bitmap Control field
 * \return Bitmap Control field
 */
  uint8_t GetBitmapControl(void) const;
/**
 * Return Pointer to octets of Partial Virtual Bitmap. Length is (m_length - 3)
 * \return Pointer to octets of Partial Virtual Bitmap
 */
  uint8_t* GetPVB(void) const;
/**
 * Return whether Multicast buffer bit is set
 * \return isMulticastBuffered
 */
  bool GetMulticastBuffered(void) const;

  // Implementations of pure virtual methods of WifiInformationElement
  WifiInformationElementId ElementId () const override;
  uint8_t GetInformationFieldSize () const override;
  void SerializeInformationField (Buffer::Iterator start) const override;
  uint8_t DeserializeInformationField (Buffer::Iterator start,
                                       uint8_t length) override;

private:
  uint8_t m_dtimPeriod;   //!< dtim period of this beacon - if frame is a beacon
  uint8_t m_dtimCount;   //!< dtim count of beacon - if frame is a beacon
  uint8_t m_bitmapControl;    //!< Bitmap Control octet
  uint8_t* m_pvb;         //!< Pointer to octets of Partial Virtual Bitmap
  uint8_t m_length;   //!< Length of the TIM element
  uint8_t* m_timElement;      //!<  Pointer for complete TIM element to serialize
};

/**
 * Serialize TIM to the given ostream
 *
 * \param os the output stream
 * \param tim the TIM
 *
 * \return std::ostream
 */
std::ostream &operator << (std::ostream &os, const Tim &tim);

ATTRIBUTE_HELPER_HEADER (Tim);

} //namespace ns3

#endif /* TIM_H */
