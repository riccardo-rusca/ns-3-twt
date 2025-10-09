/*
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
 * Author: Francesco Raviglione <francescorav.es483@gmail.com>
 */

#ifndef TE_FLAG_H
#define TE_FLAG_H

#include "ns3/core-module.h"
#include <vector>
#include <functional>
#include <iostream>

namespace ns3
{
  typedef enum {
      UNLOCKED=0,
      LOCKED=1
    } TrafficExclusionFlagStatus_e;

  class TrafficExclusionFlag: public Object {
  public:
    static TypeId GetTypeId();
    TrafficExclusionFlag();
    virtual ~TrafficExclusionFlag();

    void unlockFlag() {m_flag=UNLOCKED;}
    void lockFlag() {m_flag=LOCKED; m_already_one_tx=true;}
    TrafficExclusionFlagStatus_e checkFlagStatus() {return m_flag;}
    bool checkAtLeastOneTransmitted() {return m_already_one_tx;}

    void resetFlag() {m_flag=UNLOCKED;m_already_one_tx=false;}

  private:
    TrafficExclusionFlagStatus_e m_flag=UNLOCKED;
    bool m_already_one_tx=false;
  };
}

#endif // TE_FLAG_H
