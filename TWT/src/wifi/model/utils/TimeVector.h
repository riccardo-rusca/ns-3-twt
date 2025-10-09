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

#ifndef TIME_VECTOR_H
#define TIME_VECTOR_H

#include "ns3/core-module.h"
#include <vector>
#include <functional>
#include <iostream>

namespace ns3
{
  class TimeVector: public Object {
  public:
    static TypeId GetTypeId();
    TimeVector();
    virtual ~TimeVector();

    // This function adds a new element (i.e., timestamp) to the internal vector
    void addTime(double time_ms) {m_timevector.push_back (time_ms);}
    // This function returns true if the internal vector contains at least one element x such that t0 <= x <= t1, false otherwise
    bool checkIfAnyTimeBetween(double t0,double t1);
    // This function clears the internal vector
    void clearVector() {m_timevector.clear();}
    void printVectorContent();
  private:
    std::vector<double> m_timevector;
  };
}

#endif // TIME_VECTOR_H
