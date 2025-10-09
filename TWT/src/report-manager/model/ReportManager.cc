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

#include "ReportManager.h"
#include "ns3/core-module.h"
#include <sstream>
#include <cfloat>
#include <iomanip>

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE("ReportManager");

  TypeId
  ReportManager::GetTypeId ()
  {
    static TypeId tid = TypeId("ns3::ReportManager")
        .SetParent <Object>()
        .AddConstructor <ReportManager>();
    return tid;
  }

  ReportManager::ReportManager () {
//    for(double i=0;i<=0.9;i+=0.1) {
//      int lowerlim = std::floor(i * 10.0)/10.0;
//      int upperlim = i+0.1;
//      m_latency_distrib[std::make_pair(lowerlim,upperlim)]=0;
//    }

//    for(double i=1;i<=19;i++) {
//      int lowerlim = std::trunc(i);
//      int upperlim = i+1;
//      m_latency_distrib[std::make_pair(lowerlim,upperlim)]=0;
//    }

    int i=0;
    while(i<=190) {
      int lowerlim = i;
      if(i<10) {
        i++;
      } else {
        i+=10;
      }
      int upperlim = i;

      m_latency_distrib[std::make_pair(lowerlim,upperlim)]=0;
    }

    m_latency_distrib[std::make_pair(200,INT_MAX)]=0;
  }

  ReportManager::~ReportManager ()
  {
    NS_LOG_FUNCTION(this);
  }

  std::string
  ReportManager::latencyDistributionGetKey(latency_distr_iterator_t it) {
    std::ostringstream keyout;

    if(it->first.first<10) {
      keyout.precision(1);
    } else {
      keyout.precision(0);
    }

    keyout << std::fixed << (it->first.first)/10.0;
    keyout << "-";
    if(it->first.second<INT_MAX) {
      keyout << std::fixed << (it->first.second)/10.0;
    } else {
      keyout << std::fixed << "inf";
    }

    return std::move(keyout).str();
  }

  unsigned int
  ReportManager::latencyDistributionGetValue(latency_distr_iterator_t it) {
    return it->second;
  }

  std::vector<uint64_t>
  ReportManager::getMcsVector11()
  {
    std::vector<uint64_t> mcsvector(12);

    for(int i=0;i<=11;i++)
    {
      // Set the vector value corresponding to MCS=i to 0 if no packet with that MCS has been received
      // If no packet with that MCS has been received, no element will be stored in the map with key=i
      if(m_mcs_map.find(i)==m_mcs_map.end())
      {
        mcsvector[i]=0;
      }
      else
      {
        mcsvector[i]=m_mcs_map[i];
      }
    }

    return mcsvector;
  }

  void
  ReportManager::updateReportRx(double curr_latency_ms,unsigned int bytes_received,double curr_snr,double curr_rssi, int mcs) {
    // A new packet has been received
    m_rx_count++;
    m_bytes_rx_count+=bytes_received;

    // If the MCS info is available, update the internal MCS map
    if(mcs>=0) {
      m_mcs_map[mcs]++;
    } else {
      m_unavailable_mcs_count++;
    }

    // Compute the average, minimum and maximum SNR
    if(curr_snr<DBL_MAX) {
      m_snr_count++;

      m_metric_avg_snr+=(curr_snr-m_metric_avg_snr)/m_snr_count;

      if(curr_snr<m_metric_min_snr) {
          m_metric_min_snr=curr_snr;
      }

      if(curr_snr>m_metric_max_snr) {
          m_metric_max_snr=curr_snr;
      }
    }

    // Compute the average, minimum and maximum RSSI
    if(curr_rssi<DBL_MAX) {
      m_rssi_count++;

      m_metric_avg_rssi+=(curr_rssi-m_metric_avg_rssi)/m_rssi_count;

      if(curr_rssi<m_metric_min_rssi) {
          m_metric_min_rssi=curr_rssi;
      }

      if(curr_rssi>m_metric_max_rssi) {
          m_metric_max_rssi=curr_rssi;
      }
    }

    // Store the latency value into the latency distrbution map
//    int lowerlimit, upperlimit;
//    if(curr_latency_ms<1) {
//        lowerlimit=(std::floor(curr_latency_ms * 10.0)/10.0)*10;
//        upperlimit=(lowerlimit+0.1)*10;
//    } else if (curr_latency_ms>=1 && curr_latency_ms<19) {
//        lowerlimit=(std::trunc(curr_latency_ms))*10;
//        upperlimit=(lowerlimit+1)*10;
//    } else {
//        lowerlimit=200;
//        upperlimit=INT_MAX;
//    }
//    auto key=std::make_pair(lowerlimit,upperlimit);
//    m_latency_distrib[key]++;

    bool map_check=false;
    for (auto const& x : m_latency_distrib)
    {
        double curr_latency_ms_scaled=static_cast<double>(curr_latency_ms)*10.0; // Current latency measured in [0.1 ms] instead of [ms]
        if(curr_latency_ms_scaled>=x.first.first && curr_latency_ms_scaled<x.first.second) {
          m_latency_distrib[x.first]++;
          map_check=true;
          break;
        }
    }

    if(map_check==false) {
      NS_FATAL_ERROR("ReportManager fatal bug. Could not store current latency value " << curr_latency_ms << " due to wrong keys in m_latency_distrib. Please report this bug to the developers.");
    }

    // Store the previous value of average latency for the computation of the variance with Welford's online algorithm
    m_welfordAverageLatencyOld=m_metric_avg_latency_ms;

    // Average latency
    m_metric_avg_latency_ms+=(curr_latency_ms-m_metric_avg_latency_ms)/m_rx_count;

    // Minimum latency
    if(curr_latency_ms<m_metric_min_latency_ms) {
      m_metric_min_latency_ms=curr_latency_ms;
    }

    // Maximum latency
    if(curr_latency_ms>m_metric_max_latency_ms) {
      m_metric_max_latency_ms=curr_latency_ms;
    }

    // Compute the current variance (std dev squared) value using Welford's online algorithm
    m_welfordM2=m_welfordM2+(curr_latency_ms-m_welfordAverageLatencyOld)*(curr_latency_ms-m_metric_avg_latency_ms);
    if(m_rx_count>1) {
            m_variance=m_welfordM2/(m_rx_count-1);
    }
  }
}
