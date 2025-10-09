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

#ifndef REPORTMANAGER_H
#define REPORTMANAGER_H

#include <cfloat>
#include <cmath>
#include <limits>
#include <climits>
#include <list>
#include <unordered_map>
#include <string>
#include "ns3/core-module.h"
#include "ns3/packet.h"

namespace ns3 {
  class ReportManager : public Object {
    public:
      typedef std::map<std::pair<int,int>,unsigned int>::iterator latency_distr_iterator_t;

      static TypeId GetTypeId();
      ReportManager();
      virtual ~ReportManager();

      void updateReportRx(double curr_latency_ms,unsigned int bytes_received,double curr_snr=DBL_MAX,double curr_rssi=DBL_MAX,int mcs=-1);
      void updateReportTx(unsigned int bytes_transmitted) {m_bytes_tx_count+=bytes_transmitted; m_tx_count++;}

      double getPacketLossPercentage(void) {return (m_tx_count-m_rx_count)*100.0/m_tx_count;}
      unsigned long long int getTotalSentPackets(void) {return m_tx_count;}
      unsigned long long int getTotalReceivedPackets(void) {return m_rx_count;}

      uint64_t getTotalBytesReceived(void) {return m_bytes_rx_count;}
      uint64_t getTotalBytesTransmitted(void) {return m_bytes_tx_count;}
      double getPacketLossPercentageUsingTxRxBytes(void) {return (m_bytes_tx_count-m_bytes_rx_count)*100.0/m_bytes_tx_count;}

      // Methods (getters) to get the latency metrics at the end of the simulation
      double getAverageLatency(void) {return m_metric_avg_latency_ms;}
      double getMaximumLatency(void) {return m_metric_max_latency_ms;}
      double getMinimumLatency(void) {return m_metric_min_latency_ms;}
      double getVarianceLatency(void) {return m_variance;}
      double getStdDevLatency(void) {return sqrt(m_variance)/1000;}

      // Methods (getters) to get the PHY-layer metrics at the end of the simulation
      double getAverageSNR(void) {return m_metric_avg_snr;}
      double getMinimumSNR(void) {return m_metric_min_snr;}
      double getMaximumSNR(void) {return m_metric_max_snr;}
//      double getAverageSNR_log(void) {return 10*log10(m_metric_avg_snr);} // Returns the SNR in logaritmic scale
//      double getMinimumSNR_log(void) {return 10*log10(m_metric_min_snr);} // Returns the SNR in logaritmic scale
//      double getMaximumSNR_log(void) {return 10*log10(m_metric_max_snr);} // Returns the SNR in logaritmic scale
      double getAverageRSSI(void) {return m_metric_avg_rssi;}
      double getMinimumRSSI(void) {return m_metric_min_rssi;}
      double getMaximumRSSI(void) {return m_metric_max_rssi;}

      double getThroughputMbps(double simulationTime_sec) {
        return (m_bytes_rx_count * 8) / (simulationTime_sec * 1000000.0);
      }

      // Methods to get the MCS-related metrics
      uint8_t getMostUsedMcs(void) {
        auto maxel = std::max_element(
              m_mcs_map.begin(),
              m_mcs_map.end(),
              [] (const std::pair<uint8_t,uint64_t>& a, const std::pair<uint8_t,uint64_t>& b) -> bool
              {return a.second < b.second;});

        return maxel->first;
      }
      int getMostUsedMcsAsInt(void) {
        return static_cast<int>(getMostUsedMcs());
      }
      std::vector<uint64_t> getMcsVector11(void);
      unsigned int getUnavailableMcsCount(void) {return static_cast<unsigned int>(m_unavailable_mcs_count);}
      unsigned int getNBinsLatencyDistribution(void) {return m_latency_distrib.size();}
      std::string latencyDistributionGetKey(latency_distr_iterator_t it);
      unsigned int latencyDistributionGetValue(latency_distr_iterator_t it);
      latency_distr_iterator_t getInitialLatencyDistributionIterator(void) {return m_latency_distrib.begin();}
      latency_distr_iterator_t getFinalLatencyDistributionIterator(void) {return m_latency_distrib.end();}

      // Extra metrics (optionally filled in)
      double get_last_t0() {return m_last_t0;}
      double get_last_t1() {return m_last_t1;}
      double get_last_teff() {return m_last_teff;}
      double get_last_trx() {return m_last_trx;}
      double get_last_AOI() {
        if(m_last_trx==0) {
            return -1.0;
        } else {
          return m_last_trx-m_last_t0;
        }
      }

      double get_last_wait_time_slots() {return m_last_wait_time_slots;}
      int get_time_limit_exceeded_STA_id() {return m_time_limit_rem_data_STA_id;}
      int get_time_limit_rem_data_bytes() {return m_time_limit_rem_data;}
      double get_time_reference_shift_ms() {return m_reference_time_shift;}
      void set_last_t0(double t0) {m_last_t0=t0;}
      void set_last_t1(double t1) {m_last_t1=static_cast<double>(t1);}
      void set_last_teff(double teff) {m_last_teff=teff-m_reference_time_shift;}
      void set_last_trx(double trx) {m_last_trx=trx-m_reference_time_shift;}
      void set_last_wait_time_slots(int last_wait_slots) {m_last_wait_time_slots=static_cast<double>(last_wait_slots)*1.024;}
      void set_time_reference_shift(double milliseconds_shift) {m_reference_time_shift=milliseconds_shift;}
      void set_time_limit_rem_data(int sta_id, int rem_data_bytes) {m_time_limit_rem_data_STA_id=sta_id;m_time_limit_rem_data=rem_data_bytes;}
    private:
      // Latency metrics
      unsigned long long int m_tx_count=0;
      unsigned long long int m_rx_count=0;
      double m_metric_avg_latency_ms=0;
      double m_metric_max_latency_ms=-1;
      double m_metric_min_latency_ms=DBL_MAX;
      double m_metric_stddev_latency_ms=0;
      // Internal attributes for computing the std deviation
      double m_welfordM2=0;
      double m_welfordAverageLatencyOld=0;
      double m_variance=0;

      // PHY-layer metrics
      double m_metric_avg_snr=0; // [dB]
      double m_metric_min_snr=DBL_MAX; // [dB]
      double m_metric_max_snr=-DBL_MAX; // [dB]
      double m_metric_avg_rssi=0; // [dBm]
      double m_metric_min_rssi=DBL_MAX; // [dBm]
      double m_metric_max_rssi=-DBL_MAX; // [dBm]
      uint64_t m_snr_count=0; // Total number of received packets carrying SNR information
      uint64_t m_rssi_count=0; // Total number of received packets carrying RSSI information

      // MCS map
      std::map<uint8_t,uint64_t> m_mcs_map;
      uint64_t m_unavailable_mcs_count=0;

      // Latency distribution map, to store the number of latency values that fall within bins of 1 ms, up to 20 ms
      // Bins are reduced to a width of 0.1 ms in the interval [0,1[ ms, to cater for a more precise latency distrbution computation in relatively uncongested scenarios
      // The last bin is related the interval [20,+inf[ ms, where +inf is represented by DBL_MAX
      // The key is a pair of int describing the lower and upper limit values for that bin in [0.1 ms] (e.g., for the bin 0.1-0.19999999 ms, the key is <10,20> [0.1 ms], corresponding to the interval [0.1,0.2[)
      std::map<std::pair<int,int>,unsigned int> m_latency_distrib;

      // Throughput metrics
      uint64_t m_bytes_rx_count=0;
      uint64_t m_bytes_tx_count=0;

      // Extra metrics (optionally managed)
      double m_last_t0=0.0; // [ms]
      double m_last_t1=0.0; // [ms]
      double m_last_teff=0.0; // [ms]
      double m_last_trx=0.0; // [ms]
      double m_last_wait_time_slots=0.0; // [ms] Multiple of one time unit: 1.024 ms
      double m_reference_time_shift=0.0; // If you want to have t0, t1, teff, trx in a reference system shifted by x [ms] with respect to the global simulation time, this should be set with a value > 0 [ms]

      int m_time_limit_rem_data=0.0; // [B]
      int m_time_limit_rem_data_STA_id=0;
  };
}


#endif // REPORTMANAGER_H
