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
 * Facility to store a vector of timestamps (as doubles in milliseconds) in an ns-3 object
 * Author: Francesco Raviglione <francescorav.es483@gmail.com>
 */

#include "TimeVector.h"
#include "ns3/core-module.h"

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE("TimeVector");

  TypeId
  TimeVector::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::TimeVector")
        .SetParent <Object>()
        .AddConstructor <TimeVector>();
    return tid;
  }

  TimeVector::TimeVector () {
    m_timevector.clear();
  }

  TimeVector::~TimeVector () {
    NS_LOG_FUNCTION(this);
  }

  bool
  TimeVector::checkIfAnyTimeBetween(double t0,double t1) {
    auto start = std::lower_bound(m_timevector.begin(), m_timevector.end(), t0);
    auto end = std::upper_bound(m_timevector.begin(), m_timevector.end(), t1);

    return static_cast<bool>(start!=end);
  }

  void
  TimeVector::printVectorContent() {
    for(int i=0;i<(int)m_timevector.size ();i++) {
      std::cout << "[" << i << "]: " << m_timevector[i] << std::endl;
    }
  }
}
