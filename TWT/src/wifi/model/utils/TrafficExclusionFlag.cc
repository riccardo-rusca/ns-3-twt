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

#include "TrafficExclusionFlag.h"
#include "ns3/core-module.h"

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE("TrafficExclusionFlag");

  TypeId
  TrafficExclusionFlag::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::TrafficExclusionFlag")
        .SetParent <Object>()
        .AddConstructor <TrafficExclusionFlag>();
    return tid;
  }

  TrafficExclusionFlag::TrafficExclusionFlag () {
    m_flag=UNLOCKED;
    m_already_one_tx=false;
  }

  TrafficExclusionFlag::~TrafficExclusionFlag () {
    NS_LOG_FUNCTION(this);
  }
}
