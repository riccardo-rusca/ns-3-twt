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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "tim.h"

namespace ns3 {

Tim::Tim ()
{
  m_length = 4;     
  m_dtimPeriod = 1;
  m_dtimCount = 0;
  m_bitmapControl = 0;
}

Tim::Tim (uint8_t dtimCount, uint8_t dtimPeriod, bool isMulticastBuffered)
{
  m_length = 4;
  m_dtimPeriod = dtimPeriod;
  m_dtimCount = dtimCount;
  m_bitmapControl = 0;
  if ( dtimCount == 0 && isMulticastBuffered )
  {
    // it is  DTIM beacon and there is multicast buffered traffic
    m_bitmapControl = 1;
  }
  
  m_pvb = (uint8_t*)malloc (1);
  m_pvb[0] = 0;

  m_timElement = (uint8_t*)malloc (m_length);

  m_timElement[0] = m_dtimCount;
  m_timElement[1] = m_dtimPeriod;
  m_timElement[2] = m_bitmapControl;
  m_timElement[3] = m_pvb[0];
  


  // Copied from somewhere for reference
  //   ssize_t len =  (size_t) packet->GetSize ();
  // uint8_t *buffer = (uint8_t*)malloc (len);
  // packet->CopyData (buffer, len);
}

Tim::Tim (uint8_t dtimCount, uint8_t dtimPeriod, bool isMulticastBuffered, uint8_t* fullAidBitmap, uint16_t minAidInBuffer, uint16_t maxAidInBuffer)
{
  
  m_dtimPeriod = dtimPeriod;
  m_dtimCount = dtimCount;
  m_bitmapControl = 0;
  if ( dtimCount == 0 && isMulticastBuffered )
  {
    // it is  DTIM beacon and there is multicast buffered traffic
    m_bitmapControl = 1;
  }


  // From the standard 802.11 2020
  /*
   the Partial Virtual Bitmap field consists of octets numbered N1 to N2 of the traffic indication virtual bitmap, where N1 is the largest even number such that bits numbered 1 to (N1 * 8) – 1 in the traffic indication virtual bitmap are all 0 and N2 is the smallest number such that bits numbered (N2 + 1) * 8 to 2007 in the traffic indication virtual bitmap are all 0. In this case, the Bitmap Offset subfield value contains the number N1/2, and the Length field is set to (N2 – N1) + 4.
  */

  
  uint32_t N1 = minAidInBuffer/8;
  N1 = int(N1/2) * 2;        // Round down to even number if odd
  uint32_t N2 = maxAidInBuffer/8;

  uint8_t bitmapOffset7bits = (N1/2) << 1; 
  m_bitmapControl |= bitmapOffset7bits;
  m_length = 4 + (N2 - N1);

  m_timElement = (uint8_t*)malloc (m_length);

  m_timElement[0] = m_dtimCount;
  m_timElement[1] = m_dtimPeriod;
  m_timElement[2] = m_bitmapControl;

  for (u_int32_t i = 0; i <= (N2-N1); i++ )
  {
    m_timElement[3 + i] = fullAidBitmap[ N1 + i ];
  }

}

uint8_t 
Tim::GetDtimPeriod () const
{
  return m_dtimPeriod;
}

uint8_t 
Tim::GetDtimCount () const
{
  return m_dtimCount;
}

uint8_t 
Tim::GetBitmapControl () const
{
  return m_bitmapControl;
}

uint8_t* 
Tim::GetPVB () const
{
  return m_pvb;
}

bool 
Tim::GetMulticastBuffered(void) const
{
  bool multicastBit = ( ( m_bitmapControl & 1 ) == 1 );
  return multicastBit;
}

WifiInformationElementId
Tim::ElementId () const
{
  return IE_TIM;
}

uint8_t
Tim::GetInformationFieldSize () const
{
  return m_length;
}

void
Tim::SerializeInformationField (Buffer::Iterator start) const
{
  NS_ASSERT (m_length >= 4 && m_length <= 254); 
  start.Write (m_timElement, m_length);
}

uint8_t
Tim::DeserializeInformationField (Buffer::Iterator start,
                                   uint8_t length)
{
  m_length = length;
  NS_ASSERT (m_length >= 4 && m_length <= 254);   
  m_timElement = (uint8_t*)malloc (m_length);
  start.Read (m_timElement, m_length);

  m_dtimCount = m_timElement[0];
  m_dtimPeriod = m_timElement[1];
  m_bitmapControl = m_timElement[2];

  m_pvb = (uint8_t*)malloc (m_length - 3);
  for (int i = 0; i <= (m_length - 3); i++ )
  {
    m_pvb[i] = m_timElement[ 3 + i ];
  }
    
  // start.Read (dtimInfo, m_length);
  return length;
}

std::ostream &
operator << (std::ostream &os, const Tim &tim)
{
  os << "TIM ostream to be added - Shyam";

  return os;
}

} //namespace ns3
