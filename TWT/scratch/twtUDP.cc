/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
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
 * Author: Sebastien Deronne <sebastien.deronne@gmail.com>
 * Modified by Shyam K Venkateswaran <vshyamkrishnan@gmail.com>
 */


#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source.h"
#include "ns3/netanim-module.h"
#include <unordered_map>
#include <iomanip>
//#include <execution>
#include <chrono>
#include <unistd.h>

#define FOLDER_PATH "scratch/"
#define CLASS_IDX_VECTOR(x) (x-1)
//#define SERVER_LISTEN_START_SECOND 0.9
//#define CLIENT_TRAFFIC_START_SECOND 1.024 // Multiple of the beacon time
//#define MAX_TIME_STA_TURN_ON_MS 100.0 // Default was 2500
//#define SCHEDULE_TWT_AGREEMENT_TIME_MS 900.0 // Default was 3000.0
#define SERVER_LISTEN_START_SECOND 2.972
#define CLIENT_TRAFFIC_START_SECOND 3.072 // Multiple of the beacon time
#define MAX_TIME_STA_TURN_ON_MS 300.0 // Default was 2500
#define SCHEDULE_TWT_AGREEMENT_TIME_MS 2900.0 // Default was 3000.0
#define DEFAULT_UDP_PAYLOAD_SIZE 700 // Default was 700 B

#define DEMO_POSITIONS false // Set to true to place the STAs in the positions for the demo; should be false for normal simulations

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("82011ax_simulations");

#define IF_CHANNELSTR_ERROR(channelStr) if(channelStr.find("error") != std::string::npos)

uint32_t loopIndex = 10001;
uint32_t randSeed = 7000;      // Seed for Random Number Generator

// Power Scheme ----------------------------------
std::string useCase = "twt";    // Use TWT
//std::string useCase = "noTwt";    // Use no TWT
// Power Scheme ----------------------------------


// Simulation configuration ----------------------
std::string currentLoopIndex_string;
uint32_t flowMonStartTime_s = 10;   // Start time for flow monitor in seconds
Time keepTrackOfEnergyFrom_S = Seconds (CLIENT_TRAFFIC_START_SECOND); // Start time for current logging in seconds
// std::string currentLoopIndex_string = "010001";
// Simulation configuration ----------------------



// Network configuration -------------------------
Time beaconInterval = MicroSeconds(102400);  // 102.4 ms as beacon interval
uint32_t bsrLife_ms = 10;                // in ms - BSR life time
uint32_t ampduLimitBytes = 20000;       //50,000 at 143 Mbps and 5/6 coding takes 3.4 ms
// Network configuration -------------------------


// Logging ---------------------------------------
bool enablePcap {false};            // Enable pcap traces if true
bool enableFlowMon = true;          // Enable flow monitor if true
bool recordApPhyState = false;     // If true, AP PHY states are also logged to the stateLog file (as StaId=0)
// Logging ---------------------------------------

bool twtTriggerBased = false; // Set it to false for contention-based TWT
uint64_t maxMuSta = 1;      // Maximum number of STAs the TWT scheduler will assign for a DATA MU exchange. For BSRP, max possible is used

std::unordered_map<uint32_t, int>staIdToSTAtype;

std::unordered_map<int,std::string> index_to_statype= {
        {0, "80211bgn"},
        {1, "80211bg"},
        {2, "80211ac"},
        {3, "80211ax"}
};

// TI device model
std::unordered_map<std::string, std::unordered_map<std::string, double>>TI_currentModel_mA = {
        {"80211bgn" , {{"IDLE", 50}, {"CCA_BUSY", 50}, {"RX", 66}, {"TX", 232}, {"SLEEP", 0.12}}}, //Texas Instruments SimpleLink CC3235SF
        {"80211bg" , {{"IDLE", 40}, {"CCA_BUSY", 40}, {"RX", 40}, {"TX", 140}, {"SLEEP", 0.004}}}, //RN-131G & RN-131C 802.11 b/g Wireless LAN Module
        {"80211ac" , {{"IDLE", 358}, {"CCA_BUSY", 358}, {"RX", 472}, {"TX", 573}, {"SLEEP", 12}}}, //QCA9882 Dual-Band 2x2 MIMO 802.11ac/abgn - HT20 2.4GHz
        {"80211ax" , {{"IDLE", 294}, {"CCA_BUSY", 294}, {"RX", 388.4}, {"TX", 555.29}, {"SLEEP", 11.63}}} //QCA1062 Dual Band 2 × 2 MIMO 802.11ax + Bluetooth 5.1 - 11ax_HE40_MCS11
};
double *timeElapsedForSta_ns_TI, *awakeTimeElapsedForSta_ns_TI, *current_mA_TimesTime_ns_ForSta_TI; // This is only after keepTrackOfEnergyFrom_S

struct ampereTimeState{
    double ampereTime;
    double elapsedTime;
};

std::map<int, std::unordered_map<std::string, ampereTimeState>>current_mA_TimesTime_ns_ForSta_ForState_TI;

// -************************************************************************

// To schedule starting of STA wifi PHYs - to avoid assoc req sent at same time
// Refer https://groups.google.com/g/ns-3-users/c/bWaK9QvB3OQ/m/uHPB3t9DBAAJ
void RandomWifiStart (Ptr<WifiPhy> phy) {
  phy->ResumeFromOff();
}

//-**************************************************************************
// Parse context strings of the form "/NodeList/x/DeviceList/x/..." to extract the NodeId integer
uint32_t ContextToNodeId (std::string context) {
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

//-*************************************************************************
//-**************************************************************************
// PHY state tracing 
//template <int node>
void PhyStateTrace_inPlace (std::string context, Time start, Time duration, WifiPhyState state) {
    std::ofstream statelog("outputTWT/log_" + std::to_string(ContextToNodeId (context)) + ".statelog",std::ios_base::app);

    if (statelog.is_open()) {
        statelog << "now=" << Simulator::Now ().GetNanoSeconds () << " state=" << state << " start=" << start.GetNanoSeconds() << " duration=" << duration.GetNanoSeconds() << std::endl;

        statelog.close();
    } else {
        NS_FATAL_ERROR("Cannot write statelog");
    }

  // Do not use spaces. Use '='  and ';'
  if (Simulator::Now ().GetMicroSeconds()>keepTrackOfEnergyFrom_S.GetMicroSeconds()) {
    std::ostringstream oss;
    oss << state;
    uint32_t nodeId = ContextToNodeId (context);
    // std::cout << "state=" <<state<< ";startTime_ns="<<start.GetNanoSeconds()<<";duration_ns=" << duration.GetNanoSeconds()<<";nextStateShouldStartAt_ns="<<(start + duration).GetNanoSeconds()<<";staId="<<ContextToNodeId (context)<< std::endl;
    timeElapsedForSta_ns_TI[nodeId] += duration.GetNanoSeconds();
    if (oss.str() != "SLEEP") {
      // Add to awake time for this STA
      awakeTimeElapsedForSta_ns_TI[nodeId] += duration.GetNanoSeconds();
    }
    std::unordered_map<std::string, double> TI_currentSTAModel_mA = TI_currentModel_mA[index_to_statype[staIdToSTAtype[nodeId]]];
    // std::cout<<"\nCurrent value (mA)= "<<TI_currentSTAModel_mA[oss.str()]<<"\n";
    current_mA_TimesTime_ns_ForSta_TI[nodeId] += TI_currentSTAModel_mA[oss.str()] * duration.GetNanoSeconds();

    current_mA_TimesTime_ns_ForSta_ForState_TI[nodeId][oss.str()].ampereTime += TI_currentSTAModel_mA[oss.str()] * duration.GetNanoSeconds();
    current_mA_TimesTime_ns_ForSta_ForState_TI[nodeId][oss.str()].elapsedTime += duration.GetNanoSeconds();
  }
}

//-*************************************************************************
//-**************************************************************************
// PHY Tx state tracing - check log file
void PhyTxStateTraceAll (std::string context, Time start, Time duration, WifiPhyState state) {
  std::stringstream ss;
  ss <<FOLDER_PATH<<"log"<<currentLoopIndex_string<<".txlog";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  // f << Simulator::Now ().GetSeconds () << "    state=" << state << " start=" << start << " duration=" << duration << std::endl;
  // Do not use spaces. Use '='  and ';'
  if (state == WifiPhyState::TX)
  {
    f<<"startTime_ns="<<start.GetNanoSeconds()<<";duration_ns=" << duration.GetNanoSeconds()<< std::endl;
  }
}


//-*************************************************************************
//-*************************************************************************
void initiateTwtAtAp (Ptr<ApWifiMac> apMac, Mac48Address staMacAddress, Time twtWakeInterval, Time nominalWakeDuration, Time nextTwt) {
  /**
   * @brief Set TWT schedule at the AP - TWT SP will be scheduled at nextTwt after next beacon generation
   * 
   */
  // uint8_t flowId, Mac48Address peerMacAddress, bool isRequestingNode, bool isImplicitAgreement, bool flowType, bool isTriggerBasedAgreement, bool isIndividualAgreement, u_int16_t twtChannel, Time wakeInterval, Time nominalWakeDuration, Time nextTwt
  apMac->SetTwtSchedule (0, staMacAddress, false, true, true, twtTriggerBased, true, 0, twtWakeInterval, nominalWakeDuration, nextTwt);
  return;
}


// Function to get the current time (real-time for timestamping the CSV file, not the simulated time!)
static uint64_t timeSinceEpochMilliseconds() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string getChannelStr(double frequency,int channelWidth,int mcs, StringValue *ctrlRate, StringValue *dataMode, int channelNumber=0) {
    std::string channelStr("{");

    if((mcs>=0 && (ctrlRate==nullptr || dataMode==nullptr)) || mcs>11) {
        return "error_mcs";
    }

    uint64_t nonHtRefRateMbps;
    if(mcs>=0) {
        nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs) / 1e6;
    }

    std::ostringstream ossDataMode;

    if(mcs>=0 && ctrlRate!=nullptr && dataMode!=nullptr) {
        ossDataMode << "HeMcs" << mcs;
    }

    if (frequency == 6) {
        if(ctrlRate!=nullptr && dataMode!=nullptr) {
            (*ctrlRate) = StringValue(ossDataMode.str());
        }
        channelStr += std::to_string(channelNumber);
        channelStr += ", ";
        channelStr += std::to_string(channelWidth);
        channelStr += ", BAND_6GHZ, 0}";
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                           DoubleValue(48));
    } else if (frequency == 5) {
        if(ctrlRate!=nullptr && dataMode!=nullptr) {
            std::ostringstream ossControlMode;
            ossControlMode << "OfdmRate" << nonHtRefRateMbps << "Mbps";
            (*ctrlRate) = StringValue(ossControlMode.str());
        }
        channelStr += std::to_string(channelNumber);
        channelStr += ", ";
        channelStr += std::to_string(channelWidth);
        channelStr += ", BAND_5GHZ, 0}";
    } else if (frequency == 2.4) {
        if(ctrlRate!=nullptr && dataMode!=nullptr) {
            std::ostringstream ossControlMode;
            ossControlMode << "ErpOfdmRate" << nonHtRefRateMbps << "Mbps";
            (*ctrlRate) = StringValue(ossControlMode.str());
        }
        channelStr += std::to_string(channelNumber);
        channelStr += ", ";
        channelStr += std::to_string(channelWidth);
        channelStr += ", BAND_2_4GHZ, 0}";
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                           DoubleValue(40));
    } else {
        return "error_frequency";
    }

    if(dataMode!=nullptr) {
        *dataMode = StringValue(ossDataMode.str());
    }

    return channelStr;
}

typedef struct simulationParameters {
    uint64_t simulation_id;
    bool useRts;
    bool useExtendedBlockAck;
    double simulationTime;
    double frequency;
    std::string dlAckSeqType;
    bool enableUlOfdma;
    bool enableBsrp;
    std::string mcs_str;
    Time accessReqInterval;
    std::string csv_outfile;
    int txPower_dBm;
    int nStations;
    int num_antennas_AP;
    int AP_channelWidth;
    int gi;
    std::string packet_periodicity_str;
    std::string num_antennas_str;
    std::string channelWidth_str;
    std::string payloadSize_str;
    std::string trafficType_str;
    std::string distance_str;
    std::string t0_str;
    std::string t1_str;
    std::string twtDuration_str;
    std::string STAtype_str;
    bool enableTwt;
    std::string deadlines_str;
    std::string current_policy;
    int globalSeed;

    std::string desired_duration_str;
} sim_params_t;

static void run_single_simulation(sim_params_t parameters,int *direct_ns3_mode_last_iter_rem_B_STA_ID_ptr,int *direct_ns3_mode_last_iter_rem_B_ptr) {
    uint64_t simulation_id = parameters.simulation_id;
    bool useRts = parameters.useRts;
    bool useExtendedBlockAck = parameters.useExtendedBlockAck;
    double simulationTime = parameters.simulationTime;
    double frequency = parameters.frequency;
    std::string dlAckSeqType = parameters.dlAckSeqType;
    bool enableUlOfdma = parameters.enableUlOfdma;
    std::string mcs_str = parameters.mcs_str;
    Time accessReqInterval = parameters.accessReqInterval;
    std::string csv_outfile = parameters.csv_outfile;
    int txPower_dBm = parameters.txPower_dBm;
    int nStations = parameters.nStations;
    int AP_channelWidth = parameters.AP_channelWidth;
    int gi = parameters.gi;
    std::string packet_periodicity_str = parameters.packet_periodicity_str;
    std::string num_antennas_str = parameters.num_antennas_str;
    int num_antennas_AP = parameters.num_antennas_AP;
    std::string channelWidth_str = parameters.channelWidth_str;
    std::string payloadSize_str = parameters.payloadSize_str;
    std::string trafficType_str = parameters.trafficType_str;
    std::string distance_str = parameters.distance_str;
    std::string t0_str = parameters.t0_str;
    std::string t1_str = parameters.t1_str;
    std::string twtDuration_str = parameters.twtDuration_str;
    std::string STAtype_str = parameters.STAtype_str;
    bool enableTwt = parameters.enableTwt;
    uint32_t staMaxMissedBeacon = 10000;                 // Set the max missed beacons for STA before attempt for reassociation
    Time AdvanceWakeupPS = MicroSeconds(10);
    std::string deadlines_str = parameters.deadlines_str;
    std::string desired_duration_str = parameters.desired_duration_str;

    if (simulation_id == 0) {
        NS_FATAL_ERROR("Error. A simulation ID must be specified with --simId.");
    }

    // Class parameters, i.e., per-class parameters that are specific to a given STA class, and may assume different values for different STA classes
    // We already set in the constructor the default value for the "numSTAClasses" STA classes specified by the user
    ns3::classParameter<double> packet_periodicity(5, nStations);
    ns3::classParameter<double> distance(1, nStations); // Default distance: 1 m
    ns3::classParameter<unsigned int> num_antennas(1, nStations);
    ns3::classParameter<int> channelWidth(20, nStations);
    ns3::classParameter<unsigned int> payloadSize(DEFAULT_UDP_PAYLOAD_SIZE, nStations);
    ns3::classParameter<unsigned int> trafficType(0, nStations); // Default traffic type: 0 = DL
    ns3::classParameter<double> t0(-1, nStations);
    ns3::classParameter<double> t1(-1, nStations);
    ns3::classParameter<double> twtDuration(1.024, nStations);
    ns3::classParameter<double> deadlines(0.0, nStations);
    ns3::classParameter<int> STAtype(0, nStations);
    ns3::classParameter<int> mcs(11.0,nStations);
    ns3::classParameter<double> desiredDuration(0,nStations);

    if (num_antennas_str != "") {
        num_antennas.fillFromStringSequential(num_antennas_str);
    }
    if (packet_periodicity_str != "") {
        packet_periodicity.fillFromStringSequential(packet_periodicity_str);
    }
    if (distance_str != "") {
        distance.fillFromStringSequential(distance_str);
    }
    if (channelWidth_str != "") {
        channelWidth.fillFromStringSequential(channelWidth_str);
    }
    if (payloadSize_str != "") {
        payloadSize.fillFromStringSequential(payloadSize_str);
    }
    if (trafficType_str != "") {
        trafficType.fillFromStringSequential(trafficType_str);
    }
    if (t0_str != "") {
        t0.fillFromStringSequential(t0_str);
    }
    if (t1_str != "") {
        t1.fillFromStringSequential(t1_str);
    }
    if (twtDuration_str != "") {
        twtDuration.fillFromStringSequential(twtDuration_str);
    }
    if (STAtype_str != "") {
        STAtype.fillFromStringSequential(STAtype_str);
    }
    if (deadlines_str != "") {
        deadlines.fillFromStringSequential(deadlines_str);
    }
    if(mcs_str != "") {
        mcs.fillFromStringSequential(mcs_str);
    }
    if(desired_duration_str != "") {
        desiredDuration.fillFromStringSequential(desired_duration_str);
    }

    // Create map of STA types
    for (int i = 1; i <= nStations; i++) {
        if (STAtype.getValue(i)>=0 && STAtype.getValue(i)<=3) {
            staIdToSTAtype[i-1]=STAtype.getValue(i);
        } else {
            NS_FATAL_ERROR("Error. Invalid STA type. Supported STA types are 80211bgn (0), 80211bg (1), 80211ac (2) and 80211ax (3). STA causing the error: " << i);
        }
    }

    // Check the consistency of the t0 and t1 parameters
    for (int i = 1; i <= nStations; i++) {
        if (t0.getValue(i) < 0) {
            NS_FATAL_ERROR(
                    "Error. t0 should be specified for all STAs in the simulation. STA with unset t0 generating the error: "
                            << i);
        }

        if (t1.getValue(i) < 0) {
            NS_FATAL_ERROR(
                    "Error. t1 should be specified for all STAs in the simulation. STA with unset t1 generating the error: "
                            << i);
        }

        std::cout << "t0 for STA " << i << " = " << t0.getValue(i) << std::endl;
    }

    // The total number of STAs in the scenario is stored inside "nStations"
    if (useRts) {
        // Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(65535));
        // May not work, double check
        Config::SetDefault("ns3::WifiDefaultProtectionManager::EnableMuRts", BooleanValue(true));
    }

    // To set QoS queue size in all QoS data frames by all STAs (GAtech)
    //  Config::SetDefault ("ns3::QosFrameExchangeManager::SetQueueSize",BooleanValue (true));

    if (twtTriggerBased) {
        dlAckSeqType = "AGGR-MU-BAR";   // Use this for MU TWT - only for trigger based
    } else {
        dlAckSeqType = "ACK-SU-FORMAT";
    }

    if (dlAckSeqType == "ACK-SU-FORMAT") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    } else if (dlAckSeqType == "MU-BAR") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
    } else if (dlAckSeqType == "AGGR-MU-BAR") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    } else {
        NS_FATAL_ERROR("Invalid dlAckSeqType. Maybe you need to enable TWT? Remember what this code is made for!");
    }

    // Set Arp request retry limit to 10
    Config::SetDefault("ns3::ArpCache::MaxRetries", UintegerValue(10));
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(
            2000)));    // Default is 500 ms - changed because frames timed out and TCP connection kept dropping.
    Config::SetDefault("ns3::WifiMacQueue::MaxSize", QueueSizeValue(QueueSize("1000p")));    // Default is 500p

    for (int i = 1; i <= nStations; i++) {
        if ((channelWidth.getValue(i) != 20 && channelWidth.getValue(i) != 40 && channelWidth.getValue(i) != 80 &&
             channelWidth.getValue(i) != 160) || (channelWidth.getValue(i) > (frequency == 2.4 ? 40 : 160))) {
            NS_FATAL_ERROR(
                    "Error. Invalid STA channel width. Valid channel widths are 20, 40, 80 and 160 MHz, and the maximum channel width for 2.4 GHz is 40 MHz. STA causing the error: "
                            << i);
        }
    }

    if ((AP_channelWidth != 20 && AP_channelWidth != 40 && AP_channelWidth != 80 && AP_channelWidth != 160) ||
        (AP_channelWidth > (frequency == 2.4 ? 40 : 160))) {
        NS_FATAL_ERROR(
                "Error. Invalid AP channel width. Valid channel widths are 20, 40, 80 and 160 MHz, and the maximum channel width for 2.4 GHz is 40 MHz.");
    }

    if (gi != 800 && gi != 1600 && gi != 3200) {
        NS_FATAL_ERROR("Error. Invalid guard interval value. Supported values are 800, 1600 and 3200 ns");
    }


    // Energy model init
    timeElapsedForSta_ns_TI = new double[nStations];
    awakeTimeElapsedForSta_ns_TI = new double[nStations];
    current_mA_TimesTime_ns_ForSta_TI = new double[nStations];
    for (int i = 0; i < nStations; i++) {
        timeElapsedForSta_ns_TI[i] = 0;
        awakeTimeElapsedForSta_ns_TI[i] = 0;
        current_mA_TimesTime_ns_ForSta_TI[i] = 0;
    }
    // Energy model end


    // Create the nodes (1 AP, nStations STAs)
    std::vector<NodeContainer> wifiStaNodes(nStations);
    for (int i = 1; i <= nStations; i++) {
        wifiStaNodes[CLASS_IDX_VECTOR(i)].Create(1); // One STA for each NodeContainer
    }

    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    NetDeviceContainer apDevice;
    // NetDeviceContainer staDevices;

    std::vector<NetDeviceContainer> staDevices(nStations);

    WifiMacHelper mac;
    WifiHelper wifi;

    if (frequency == 6) {
        wifi.SetStandard(WIFI_STANDARD_80211ax_6GHZ);
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(48));
        Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(2));
    } else if (frequency == 5) {
        wifi.SetStandard(WIFI_STANDARD_80211ax_5GHZ);
        Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(2));
    } else if (frequency == 2.4) {
        wifi.SetStandard(WIFI_STANDARD_80211ax_2_4GHZ);
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(40));
        Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(2));
    } else {
        NS_FATAL_ERROR("Wrong frequency value!");
    }

    // Set guard interval and MPDU buffer size
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(gi)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize",
                UintegerValue(useExtendedBlockAck ? 256 : 64));

    // Set the SSID of the 802.11ax network
    Ssid ssid = Ssid("ns3-80211ax");

    /*
     * SingleModelSpectrumChannel cannot be used with 802.11ax because two
     * spectrum models are required: one with 78.125 kHz bands for HE PPDUs
     * and one with 312.5 kHz bands for, e.g., non-HT PPDUs (for more details,
     * see issue #408 (CLOSED))
     */
    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

    Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
    spectrumChannel->AddPropagationLossModel(lossModel);

    // SpectrumWifiPhyHelper phy;

    for (int i = 1; i <= nStations; i++) {
        SpectrumWifiPhyHelper phy;

        // mcs<0? Why? Is it because we are setting the MCS later on?
        std::string channelStr = getChannelStr(frequency, channelWidth.getValue(i), -1, nullptr, nullptr);

        IF_CHANNELSTR_ERROR(channelStr) {
            NS_FATAL_ERROR(
                    "Error. Cannot retrieve a channelStr. Please check the code for bugs. Details: " << channelStr);
        }

        // wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

        if (mcs.getValue(i) >= 0 && mcs.getValue(i) <= 11) {
            std::ostringstream ossDataMode;
            ossDataMode << "HeMcs" << mcs.getValue(i);
            auto nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs.getValue(i)) / 1e6;
            StringValue ctrlRate;
            std::ostringstream ossControlMode;
            ossControlMode << "OfdmRate" << nonHtRefRateMbps << "Mbps";
            ctrlRate = StringValue(ossControlMode.str());

            wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                         "DataMode",
                                         StringValue(ossDataMode.str()),
                                         "ControlMode",
                                         ctrlRate);

            //std::cout << "Using ConstantRateWifiManager with MCS: " << mcs << " (rate adaptation off)." << std::endl;
        } else if (mcs.getValue(i) < 0) {
            wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

            //std::cout << "Using MinstrelHtWifiManager (rate adaptation on)." << std::endl;
        } else {
            NS_FATAL_ERROR("Error. MCS values greater than 11 are not valid!");
        }


        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.SetChannel(spectrumChannel);

        if (txPower_dBm >= 0) {
            phy.Set("TxPowerStart", DoubleValue(txPower_dBm));
            phy.Set("TxPowerEnd", DoubleValue(txPower_dBm));
        }

        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        phy.Set("ChannelSettings", StringValue(channelStr));

        phy.Set("Antennas", UintegerValue(num_antennas.getValue(i)));
        phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(num_antennas.getValue(i)));
        phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(num_antennas.getValue(i)));

        staDevices[CLASS_IDX_VECTOR(i)] = wifi.Install(phy, mac, wifiStaNodes[CLASS_IDX_VECTOR(i)]);

        phy.EnablePcap(std::string("outputTWT/80211ax_simulations_STA_") + std::to_string(i), staDevices[CLASS_IDX_VECTOR(i)]);

        //std::cout << "Setting up STA " << i << " with # antennas: " << num_antennas.getValue (i) << " - channel width: " << channelWidth.getValue (i) << " MHz." <<  std::endl;
    }

    SpectrumWifiPhyHelper APphy;

    APphy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    APphy.SetChannel(spectrumChannel);

    if (txPower_dBm >= 0) {
        APphy.Set("TxPowerStart", DoubleValue(txPower_dBm));
        APphy.Set("TxPowerEnd", DoubleValue(txPower_dBm));
    }

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    std::string AP_channelStr = getChannelStr(frequency, AP_channelWidth, -1, nullptr, nullptr);
    APphy.Set("ChannelSettings", StringValue(AP_channelStr));

    APphy.Set("Antennas", UintegerValue(num_antennas_AP));
    APphy.Set("MaxSupportedTxSpatialStreams", UintegerValue(num_antennas_AP));
    APphy.Set("MaxSupportedRxSpatialStreams", UintegerValue(num_antennas_AP));

    if (dlAckSeqType != "NO-OFDMA") {
        mac.SetMultiUserScheduler("ns3::TwtRrMultiUserScheduler",
                                  "EnableUlOfdma", BooleanValue(true),
                // "EnableBsrp", BooleanValue (true),
                                  // "EnableBsrp", BooleanValue(twtTriggerBased),
                                  "EnableBsrp", BooleanValue(true),
                                  "NStations", UintegerValue(maxMuSta));
    }

    mac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "BE_BlockAckThreshold", UintegerValue(1),
                "BE_MaxAmpduSize", UintegerValue(ampduLimitBytes),
                "BsrLifetime", TimeValue(MilliSeconds(bsrLife_ms)),
                "Ssid", SsidValue(ssid)
            //  "BeaconInterval", TimeValue(MicroSeconds(65535*1024))); // One beacon every 67 seconds. As we simulate less than 67 seconds, the effect is to practically disable beacons after the first one.
    );

    apDevice = wifi.Install(APphy, mac, wifiApNode);

    APphy.EnablePcap("80211ax_simulations_AP", apDevice);

    // These lines work as expected only if we have one Access Point
    // If we have more than one, only the AP with index 0 (apDevice.Get(0)) will be considered (i.e., the first AP that was configured in the simulation)
    // This is a way to get the PHY model object from the NetDevice (i.e., the AP NetDevice), to get (or set) some parameters such as the actual channel number
    Ptr<WifiNetDevice> wifiAPdevice = DynamicCast<WifiNetDevice>(apDevice.Get(0));
    Ptr<WifiPhy> actualAPPhy = wifiAPdevice->GetPhy();

    // As the simulation model may have set automatically the channel number (i.e., if we specify "0" as channel number in the ChannelSettings tuple),
    // we can get in this way the actual channel number that will be used (e.g., for logging, or for saving to the final CSV file)
    uint8_t actualAPChannelNumber = actualAPPhy->GetChannelNumber();

    std::vector<uint8_t> actualSTAChannelNumber(nStations);
    for (int c = 1; c <= nStations; c++) {
        // There is only one STA in each NodeContainer - that's why we call Get(0), which returns the NetDevice of the first and only STA in each staDevices[c] NodeContainer
        Ptr<WifiNetDevice> wifiSTAdevice = DynamicCast<WifiNetDevice>(staDevices[CLASS_IDX_VECTOR(c)].Get(0));
        Ptr<WifiPhy> actualSTAPhy = wifiSTAdevice->GetPhy();
        actualSTAChannelNumber[CLASS_IDX_VECTOR(c)] = actualSTAPhy->GetChannelNumber();
    }

    // Currently not used as we are using the RxSignalInfo tags inside each packet
    // actualAPPhy->SetReportSignalCallback (&signal_rx);

//    // Get the pointers to the PHY of all the stations, and set the ReportSignal callback
//    for(int i=0;i<(int) nStations;i++) {
//      Ptr<WifiNetDevice> wifiSTAdevice = DynamicCast<WifiNetDevice> (staDevices.Get(i));
//      Ptr<WifiPhy> actualSTAPhy = wifiSTAdevice->GetPhy ();

//      // Currently not used as we are using the RxSignalInfo tags inside each packet to retrieve the SNR and RSSI information
//      // actualSTAPhy->SetReportSignalCallback (&signal_rx);
//    }

    // Set the random number generators for the IEEE 802.11ax models
    int64_t streamNumber = 150;
    streamNumber += wifi.AssignStreams(apDevice, streamNumber);

    for (int i = 1; i <= nStations; i++) {
        streamNumber += wifi.AssignStreams(staDevices[CLASS_IDX_VECTOR(i)], streamNumber);
    }

    // Set static devices
    // We set here the position in space of the devices
    // Units are in meters
    // AP first
    MobilityHelper APmobility;
    Ptr<ListPositionAllocator> APpositionAlloc = CreateObject<ListPositionAllocator>();

    if (DEMO_POSITIONS == false) {
        APpositionAlloc->Add(Vector(1.0, 1.0, 0.0));
    //    APpositionAlloc->Add(Vector(0.0, 0.0, 0.0));
    } else {
        APpositionAlloc->Add(Vector(4.0, 0.0, 1.5));
    }

    APmobility.SetPositionAllocator(APpositionAlloc);
    APmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    APmobility.Install(wifiApNode);

    // Then, we set the position of the STAs
    for(int i=1;i<=nStations;i++) {
        MobilityHelper mobility;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

        double x,y;
        if (DEMO_POSITIONS == false) {
            //        positionAlloc->Add(Vector(distance.getValue(i), 0.0, 0.0));
            double theta = 2.0 * 3.14 * (i - 1) / nStations;
            x = 1.0 + 1.0 * cos(theta);
            y = 1.0 + 1.0 * sin(theta);
        } else {
            if (i <= 3) {
                x = 0.0;
                y = (i - 1) * 2.0;
            } else if(i>=4 && i <=6) {
                x = 2.0;
                y = (i - 4) * 2.0;
            } else {
                x = 4.0;
                y = (i - 6) * 2.0;
            }
        }

        positionAlloc->Add(Vector(x, y, 0.0));
        mobility.SetPositionAllocator(positionAlloc);

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

        mobility.Install(wifiStaNodes[CLASS_IDX_VECTOR(i)]);
    }

    // Set max missed beacons to avoid dis-association
    for (int ii=1;ii<=nStations;ii++) {
        std::stringstream nodeIndexStringTemp, maxBcnStr, advWakeStr, apsdStr;
        nodeIndexStringTemp << wifiStaNodes[CLASS_IDX_VECTOR(ii)].Get(0)->GetId();
        maxBcnStr << "/NodeList/" << nodeIndexStringTemp.str()
                  << "/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/MaxMissedBeacons";
        advWakeStr << "/NodeList/" << nodeIndexStringTemp.str()
                   << "/DeviceList/0/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::StaWifiMac/AdvanceWakeupPS";

        Config::Set(maxBcnStr.str(), UintegerValue(staMaxMissedBeacon));
        Config::Set(advWakeStr.str(), TimeValue(AdvanceWakeupPS));
    }


    // Setting up TWT
    Ptr<WifiNetDevice> apWifiDevice = apDevice.Get(0)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
    Ptr<WifiMac> apMacTemp = apWifiDevice->GetMac ();
    Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (apMacTemp);
    // Mac48Address apMacAddress = apMac->GetAddress();
    // std::cout<<"Ap MAC:"<<apMac<<"\n";
    if (enableTwt) {
        for (int ii = 1; ii <=nStations ; ii++) {
            // Setting up TWT for Sta Mac
            Ptr<WifiNetDevice> device = staDevices[CLASS_IDX_VECTOR(ii)].Get(0)->GetObject<WifiNetDevice> ();    //This returns the pointer to the object - works for all functions from WifiNetDevice
            Ptr<WifiMac> staMacTemp = device->GetMac ();
            Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (staMacTemp);

            // To ensure that offsets are not too large. That may cause frame drops before the first SP begins. Above line ensures that the first scheduled SP is no further than k*BI from the next beacon in the future
            Time scheduleTwtAgreement = MilliSeconds(SCHEDULE_TWT_AGREEMENT_TIME_MS);

            // Time scheduleTwtAgreement = (firstTwtSpStart);
            Time twtWakeInterval = beaconInterval;
            Time twtNominalWakeDuration = MilliSeconds(twtDuration.getValue(ii));
            Time time_t1 = MicroSeconds(t1.getValue(ii)*1000.0); // Time between next beacon and next TWT SP

            // TWT at AP MAC
            // Mac48Address staMacAddress = staMac->GetAddress();
            std::cout<<"\nTWT agreement for STA:"<<staMac->GetAddress()<<"\nAction frame acheduled at t = "<<scheduleTwtAgreement.GetSeconds() << "s;\nTWT SP starts at "<<time_t1.GetMicroSeconds()/1000.0<<" ms after next beacon;\nTWT Wake Interval = "<< twtWakeInterval.GetMicroSeconds()/1000.0<<" ms;\nTWT Nominal Wake Duration = "<< twtNominalWakeDuration.GetMicroSeconds()/1000.0<<" ms;\n\n\n";
            Simulator::Schedule(scheduleTwtAgreement, &initiateTwtAtAp, apMac,staMac->GetAddress(), twtWakeInterval, twtNominalWakeDuration, time_t1);
        }
    }


    // ---------------------------------------------
    // - enabling PHY of wifiStaNodes at random times - to avoid assoc req at same time
    for (int ii=1;ii<=nStations;ii++) {
        Ptr<Node> n = wifiStaNodes[CLASS_IDX_VECTOR(ii)].Get(0);
        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (n->GetDevice (0)); //assuming only one device
        Ptr<WifiPhy> phy = wifi_dev->GetPhy();
        phy->SetOffMode (); //turn all of them off

        //schedule random time when they switch their radio ON.
        Ptr<UniformRandomVariable> random = CreateObject<UniformRandomVariable> ();
        uint32_t start_time = random->GetInteger (0, MAX_TIME_STA_TURN_ON_MS);
        Simulator::Schedule (MilliSeconds (start_time), &RandomWifiStart, phy);
    }
    // ---------------------------------------------


    // Install an IPv4 stack on the nodes
    InternetStackHelper stack;
    stack.Install(wifiApNode);

    for(int i=1;i<=nStations;i++) {
        stack.Install(wifiStaNodes[CLASS_IDX_VECTOR(i)]);
    }

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    std::vector<Ipv4InterfaceContainer> staNodeInterfaces(nStations);
    Ipv4InterfaceContainer apNodeInterface;

    apNodeInterface = address.Assign(apDevice);

    for(int i=1;i<=nStations;i++) {
        staNodeInterfaces[CLASS_IDX_VECTOR(i)] = address.Assign(staDevices[CLASS_IDX_VECTOR(i)]);
    }

    // Set the server and client applications
    std::vector<ApplicationContainer> serverApp(nStations);
    std::vector<ApplicationContainer> clientApp(nStations);

    // One report manager for each class
    std::vector<ReportManager> reportMans(nStations);

    for(int staidx=1;staidx<=nStations;staidx++) {
        if(trafficType.getValue(staidx)==0) { // DL
            auto serverNodes = std::ref(wifiStaNodes[CLASS_IDX_VECTOR(staidx)]);
            NodeContainer clientNodes;
            Ipv4InterfaceContainer serverInterfaces;

            // Get(0) as we have only one STA in each "class"/NodeContainer/NetDeviceContainer, and there is only one AP
            serverInterfaces.Add(staNodeInterfaces[CLASS_IDX_VECTOR(staidx)].Get(0));
            clientNodes.Add(wifiApNode.Get(0));

            // UDP flow from the AP to the STA
            UdpMeasurementServerHelper server(49000); // The port value will be ignored, as we will force the port to be (49000 + STA number) = (49000 + staidx)
            server.SetAttribute ("STAClass",UintegerValue(staidx));
            reportMans[CLASS_IDX_VECTOR(staidx)].set_time_reference_shift(CLIENT_TRAFFIC_START_SECOND*1000.0);
            server.SetAttribute ("ReportManager",PointerValue(&(reportMans[CLASS_IDX_VECTOR(staidx)])));

            serverApp[CLASS_IDX_VECTOR(staidx)] = server.Install(serverNodes.get());
            serverApp[CLASS_IDX_VECTOR(staidx)].Start(Seconds(SERVER_LISTEN_START_SECOND));
            serverApp[CLASS_IDX_VECTOR(staidx)].Stop(Seconds(simulationTime + 1));

            // Get(0)/GetAddress(0) as we expect here only one STA in each container
            UdpMeasurementClientHelper client(serverInterfaces.GetAddress(0), 49000+staidx);
            client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
            client.SetAttribute("Interval", TimeValue(Time(std::to_string(packet_periodicity.getValue(staidx))))); // packets/s
            client.SetAttribute("PacketSize", UintegerValue(payloadSize.getValue(staidx)));
            client.SetAttribute("ReportManager",PointerValue(&(reportMans[CLASS_IDX_VECTOR(staidx)])));

            client.SetAttribute("t0",DoubleValue(t0.getValue(staidx)));
            client.SetAttribute("t1",DoubleValue(t1.getValue(staidx)));

            clientApp[CLASS_IDX_VECTOR(staidx)] = client.Install(clientNodes.Get(0));
            clientApp[CLASS_IDX_VECTOR(staidx)].Start(Seconds(CLIENT_TRAFFIC_START_SECOND));
            clientApp[CLASS_IDX_VECTOR(staidx)].Stop(Seconds(simulationTime + 1));
        } else if(trafficType.getValue (staidx)==1) { // UL
            auto serverNodes = std::ref(wifiApNode);
            NodeContainer clientNodes;
            Ipv4InterfaceContainer serverInterfaces;

            // Get(0) as we have only one STA in each "class"/NodeContainer/NetDeviceContainer, and there is only one AP
            serverInterfaces.Add(apNodeInterface.Get(0));
            clientNodes.Add(wifiStaNodes[CLASS_IDX_VECTOR(staidx)].Get(0));

            // UDP flow
            UdpMeasurementServerHelper server(49000); // The port value will be ignored, as we will force the port to be (49000 + STA number) = (49000 + staidx)
            server.SetAttribute ("STAClass",UintegerValue(staidx));
            reportMans[CLASS_IDX_VECTOR(staidx)].set_time_reference_shift(CLIENT_TRAFFIC_START_SECOND*1000.0);
            server.SetAttribute ("ReportManager",PointerValue(&(reportMans[CLASS_IDX_VECTOR(staidx)])));

            serverApp[CLASS_IDX_VECTOR(staidx)] = server.Install(serverNodes.get());
            serverApp[CLASS_IDX_VECTOR(staidx)].Start(Seconds(SERVER_LISTEN_START_SECOND));
            serverApp[CLASS_IDX_VECTOR(staidx)].Stop(Seconds(simulationTime + 1));

            // Get(0)/GetAddress(0) as we expect here only one STA in each container
            UdpMeasurementClientHelper client(serverInterfaces.GetAddress(0), 49000+staidx);
            client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
            client.SetAttribute("Interval", TimeValue(Time(std::to_string(packet_periodicity.getValue(staidx))))); // packets/s
            client.SetAttribute("PacketSize", UintegerValue(payloadSize.getValue(staidx)));
            client.SetAttribute("ReportManager",PointerValue(&(reportMans[CLASS_IDX_VECTOR(staidx)])));

            client.SetAttribute("t0",DoubleValue(t0.getValue(staidx)));
            client.SetAttribute("t1",DoubleValue(t1.getValue(staidx)));

            clientApp[CLASS_IDX_VECTOR(staidx)] = client.Install(clientNodes.Get(0));
            clientApp[CLASS_IDX_VECTOR(staidx)].Start(Seconds(CLIENT_TRAFFIC_START_SECOND));
            clientApp[CLASS_IDX_VECTOR(staidx)].Stop(Seconds(simulationTime + 1));
        } else {
            NS_FATAL_ERROR("Error. Invalid traffic type specified. Valid traffic types are either DL (0) or UL (1).");
        }
    }


    // Code for NetAnim
    AnimationInterface anim ("twt-animation.xml"); // Mandatory
    for (std::size_t ii = 1; ii <= static_cast<std::size_t>(nStations) ; ii++)
    {
        anim.UpdateNodeDescription (wifiStaNodes[CLASS_IDX_VECTOR(ii)].Get(0), "STA" + std::to_string(ii)); // Optional
        anim.UpdateNodeColor (wifiStaNodes[CLASS_IDX_VECTOR(ii)].Get(0), 255, 0, 0); // Optional
    }

    anim.UpdateNodeDescription (wifiApNode.Get(0), "AP"); // Optional
    anim.UpdateNodeColor (wifiApNode.Get(0), 0, 255, 0); // Optional

    anim.EnablePacketMetadata (); // Optional
    anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (5), Seconds (0.25)); //Optional
    anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
    anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional
    anim.EnableQueueCounters (Seconds (0), Seconds (10)); //Optional


    // For state tracing - to log file
    for (std::size_t ii = 1; ii <= static_cast<std::size_t>(nStations) ; ii++)
    {
        std::stringstream nodeIndexStringTemp, phyStateStr;
        nodeIndexStringTemp << wifiStaNodes[CLASS_IDX_VECTOR(ii)].Get(0)->GetId();
        phyStateStr << "/NodeList/" << nodeIndexStringTemp.str() << "/DeviceList/*/Phy/State/State";
        Config::Connect (phyStateStr.str(), MakeCallback (&PhyStateTrace_inPlace));

        printf("%s\n",phyStateStr.str().c_str());
    }

    // Enable state tracing for the AP too
    std::stringstream APnodeIndexStringTemp, APphyStateStr;
    APnodeIndexStringTemp << wifiApNode.Get(0)->GetId();
    APphyStateStr << "/NodeList/" << APnodeIndexStringTemp.str() << "/DeviceList/*/Phy/State/State";
    Config::Connect (APphyStateStr.str(), MakeCallback (&PhyStateTrace_inPlace));

    Simulator::Schedule(Seconds(0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

    // Pre-populate the ARP cache of all devices (defined in arp_utils.h)
    PopulateARPcache();

    // Uncomment these lines to enable the additional collection of metrics through FlowMonitor (see: https://www.nsnam.org/docs/models/html/flow-monitor.html)
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

    // Get the AP Node ID
    int APnodeID = apDevice.Get(0)->GetNode()->GetId();

    (void) APnodeID;
    //Simulator::Stop(Seconds(simulationTime + 1));
    Simulator::Stop(MicroSeconds(simulationTime*1000.0));
    Simulator::Run();

    // Uncomment this line to enable the additional collection of metrics through FlowMonitor (see: https://www.nsnam.org/docs/models/html/flow-monitor.html)
    flowMonitor->SerializeToXmlFile("flowMonOutput.xml", true, true);

    double *avgCurrent_mAForSTA = new double [nStations];
    double *totEnergyConsumedForSta_J = new double [nStations];

    // Map of STA MAC address to current in mA
    std::map<Mac48Address, double> staMacToCurrent_mA;
    std::map<Mac48Address, double> staMacToTimeElapsed_us;
    std::map<Mac48Address, double> staMacToTimeAwake_us;
    // Map of STA MAC address to energy consumed in J
    std::map<Mac48Address, double> staMacToEnergyConsumed_J;

    // In place current calculation
    for (int staidx=1;staidx<=nStations;staidx++)
    {
        int i=CLASS_IDX_VECTOR(staidx);
        Ptr<Node> n = wifiStaNodes[CLASS_IDX_VECTOR(staidx)].Get(0);
        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (n->GetDevice (0)); //assuming only one device

        Ptr<WifiMac> wifi_mac = wifi_dev->GetMac ();
        Ptr<StaWifiMac> sta_mac = DynamicCast<StaWifiMac> (wifi_mac);
        Mac48Address currentMacAddress = sta_mac->GetAddress ();
        avgCurrent_mAForSTA[i] = current_mA_TimesTime_ns_ForSta_TI[i]/timeElapsedForSta_ns_TI[i];
        totEnergyConsumedForSta_J [i] = (avgCurrent_mAForSTA[i]/1000.0) * (timeElapsedForSta_ns_TI[i]/1e9) * 3; // for 3 volts
        // Add this energy to map staMacToEnergyConsumed_J
        staMacToEnergyConsumed_J[currentMacAddress] = totEnergyConsumedForSta_J [i];
        // Add current mA to map
        staMacToCurrent_mA[currentMacAddress] = avgCurrent_mAForSTA[i];
        staMacToTimeElapsed_us[currentMacAddress] = timeElapsedForSta_ns_TI[i]/1000.0;
        staMacToTimeAwake_us[currentMacAddress] = awakeTimeElapsedForSta_ns_TI[i]/1000.0;
        std::cout<<"For STA "<<staidx<<";MacAddress="<<currentMacAddress<<"; Elapsed time (ms) = "<<timeElapsedForSta_ns_TI[i]/1e6<<"; Awake elapsed time (ms) = "<<awakeTimeElapsedForSta_ns_TI[i]/1e6<<"; Average current (mA) = "<<avgCurrent_mAForSTA[i]<<"; Total energy consumed (J) = "<<staMacToEnergyConsumed_J[currentMacAddress]<<std::endl;

        // g <<"simID="<<currentLoopIndex_string<<";" << "nSTA="<<nStations<<";" <<"randSeed="<<randSeed<<";" <<"useCase="<<useCase<<";"<<"triggerBased="<<twtTriggerBased<<";" <<"maxMuSta="<<maxMuSta<<";"
        //     <<"sta#="<<i<<";"
        //     <<"MacAddress="<<currentMacAddress<<";"
        //     <<"timeElapsedForSta_ms="<<timeElapsedForSta_ns_TI[i]/1e6<<";"
        //     <<"avgCurrentForSta_mA="<<avgCurrent_mAForSTA[i]<<";"
        //     <<"totEnergyConsumedForSta_J="<<staMacToEnergyConsumed_J[currentMacAddress]<<";\n";
    }

    std::map<std::tuple<int,std::string>,std::tuple<double,double,double,double>> stateEnergyTimeData_ForSta;
    for (const auto& outerPair : current_mA_TimesTime_ns_ForSta_ForState_TI) {
        double totEnergyForStaJ = 0.0;
        double totTimeForSta = 0.0;
        int outerKey = outerPair.first + 1;
        const auto& innerMap = outerPair.second;

        for (const auto& innerPair : innerMap) {
            double ampereTime = innerPair.second.ampereTime;
            totTimeForSta += innerPair.second.elapsedTime;
            double totEnergyConsumedForStaForState_J = (ampereTime/1000.0/1e9) * 3; // for 3 volts
            totEnergyForStaJ += totEnergyConsumedForStaForState_J;
        }

        // Iterate over the inner unordered_map
        std::cout << "For STA: " << outerKey << std::endl;
        for (const auto& innerPair : innerMap) {
            const std::string& innerKey = innerPair.first;
            double ampereTime = innerPair.second.ampereTime;
            double totEnergyConsumedForStaForState_J = (ampereTime/1000.0/1e9) * 3; // for 3 volts
            double energyPercentage = totEnergyConsumedForStaForState_J/totEnergyForStaJ*100.0;
            double timePercentage = innerPair.second.elapsedTime/totTimeForSta*100.0;
            std::cout << "    " << innerKey << ": " << totEnergyConsumedForStaForState_J << " J (" << energyPercentage << "%)" << std::endl;
            std::cout << "    " << innerKey << ": " << innerPair.second.elapsedTime/1e6 << " ms (" << timePercentage << "%)" << std::endl;

            if(outerKey!=APnodeID+1) {
                stateEnergyTimeData_ForSta[std::make_tuple(outerKey,innerKey)]=std::make_tuple(totEnergyConsumedForStaForState_J,energyPercentage,innerPair.second.elapsedTime/1e6,timePercentage);
            }
        }
        std::cout << "    " << "totEnergyJoule" << ": " << totEnergyForStaJ << " J" << std::endl;
    }

    Simulator::Destroy();

//            std::cout << "Average latency [ms]: " << metricsSup->getAverageLatency () << std::endl;
//            std::cout << "Minimum latency [ms]: " << metricsSup->getMinimumLatency () << std::endl;
//            std::cout << "Maximum latency [ms]: " << metricsSup->getMaximumLatency () << std::endl;
//            std::cout << "Packet loss [%]: " << metricsSup->getPacketLoss () << std::endl;
//            std::cout << "Duplicate packet count: " << metricsSup->getDuplicates () << std::endl;
//            std::cout << "Tx count: " << metricsSup->getTotalSentPackets () << std::endl;
//            std::cout << "Rx count: " << metricsSup->getTotalReceivedPackets () << std::endl;

    std::cout << "Terminated simulation [ID=" << static_cast<unsigned long long int>(simulation_id) << "] for freq=" << frequency << " GHz, AP channel width=" << AP_channelWidth
              << " MHz, GI=" << gi << " ns, dlAckSeqType=" << dlAckSeqType
              << ", AP channel #=" << static_cast<unsigned int>(actualAPChannelNumber)
              << ", number of STAs=" << nStations << ", buffer/payload size for STA 1 [B]=" << payloadSize.getValue (1) << std::endl;
//    std::cout << "TeffVector content=";
//    teff_vector.printVectorContent ();

//            std::cout << "Measured throughput [Mbit/s]: " << throughput << std::endl;
//            std::cout << "Average latency [ms]: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->getAverageLatency () << std::endl;
//            std::cout << "Minimum latency [ms]: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->getMinimumLatency () << std::endl;
//            std::cout << "Maximum latency [ms]: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->getMaximumLatency () << std::endl;
//            std::cout << "Latency standard deviation [ms]: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->getStdDevLatency () << std::endl;
//            std::cout << "Packet loss [%]: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->getLostPercentage () << std::endl;
//            std::cout << "Rx count: " << DynamicCast<UdpMeasurementServer>(serverApp.Get(0))->GetReceived () << std::endl;

    // Save the obtained metrics to a CSV file
    if(csv_outfile!="") {
        std::ofstream csv_ofstream;
        std::string full_csv_name = csv_outfile + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1) {
            // The file already exists
            csv_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        } else {
            // The file does not exist yet
            csv_ofstream.open(full_csv_name);
            csv_ofstream << "simulation_id,global_seed,policy,unix_timestamp_ms,STA_ID,trafficType,UL_OFDMA_enabled,distance_m,packet_periodicity_s,payload_size,num_antennas_mimo_STA,num_antennas_mimo_AP,frequency_band_GHz,AP_channel_number,STA_channel_number,txpower_dBm,ACK_seq_type,guard_interval,mcs,channel_width_MHz_STA,channel_width_MHz_AP,"
                            "num_STAs_total,"
                            "avg_lat_ms,max_lat_ms,min_lat_ms,stddev_lat_ms,"
                            "pktloss_perc,throughput_Mbps,"
                            "avg_snr_dB,max_snr_dB,min_snr_dB,"
                            "avg_rssi_dB,max_rssi_dB,min_rssi_dB,"
                            "most_used_MCS,MCS0,MCS1,MCS2,MCS3,MCS4,MCS5,MCS6,MCS7,MCS8,MCS9,MCS10,MCS11,MCS_unavailable_count,";

            for(ReportManager::latency_distr_iterator_t it=reportMans[0].getInitialLatencyDistributionIterator();
                it!=reportMans[0].getFinalLatencyDistributionIterator();
                it++) {
                csv_ofstream << "latency_distr_ms_" << reportMans[0].latencyDistributionGetKey(it)<<",";
            }

            csv_ofstream << "rx_bytes_per_class,tx_bytes_per_class,"
                            "t0,t1,twtDuration,last_trx,last_teff,last_AOI,t0>t1,effective_latency,"
                            "energymodel_elapsed_time,energymodel_awake_elapsed_time,energymodel_avg_current_mA,energymodel_total_energy_J,"
                            "deadline_ms,deadline_met,";

            csv_ofstream << "TX_energy_J,TX_energy_percentage,TX_time_ms,TX_time_percentage,";
            csv_ofstream << "RX_energy_J,RX_energy_percentage,RX_time_ms,RX_time_percentage,";
            csv_ofstream << "CCA_busy_energy_J,CCA_busy_energy_percentage,CCA_busy_time_ms,CCA_busy_time_percentage,";
            csv_ofstream << "SLEEP_energy_J,SLEEP_energy_percentage,SLEEP_time_ms,SLEEP_time_percentage,";
            csv_ofstream << "IDLE_energy_J,IDLE_energy_percentage,IDLE_time_ms,IDLE_time_percentage,";
            csv_ofstream << "desired_duration_ms,diff_duration_ms";

            csv_ofstream << std::endl;
        }

        uint64_t now = timeSinceEpochMilliseconds();

        // Save one line for each STA class
        for(int staidx=1;staidx<=nStations;staidx++) {
            Ptr<ReportManager> reportManPtr = &(reportMans[CLASS_IDX_VECTOR(staidx)]);
            std::vector mcsVector = reportManPtr->getMcsVector11 ();

            if(std::abs(reportManPtr->getPacketLossPercentage ()-reportManPtr->getPacketLossPercentageUsingTxRxBytes ())>0.5) {
                NS_FATAL_ERROR("There appears to be a bug in the code, as getPacketLossPercentage() didn't match getPacketLossPercentageUsingTxRxBytes () for more than 0.5 percent.");
            }

            csv_ofstream
                    << static_cast<unsigned long long int>(simulation_id) << ","
                    << (parameters.globalSeed<0 ? RngSeedManager::GetSeed() : parameters.globalSeed) << ","
                    << parameters.current_policy << ","
                    << now << ","
                    << staidx << ","
                    << (trafficType.getValue(staidx) == 0 ? "DL" : trafficType.getValue(staidx) == 1 ? "UL" : "UL") << ","
                    << enableUlOfdma << ","
                    << distance.getValue(staidx) << ","
                    << packet_periodicity.getValue(staidx) << ","
                    << payloadSize.getValue(staidx) << ","
                    << num_antennas.getValue(staidx) << ","
                    << num_antennas_AP << ","
                    << frequency << ","
                    << static_cast<unsigned int>(actualAPChannelNumber) << ","
                    << static_cast<unsigned int>(actualSTAChannelNumber[CLASS_IDX_VECTOR(staidx)]) << ","
                    << (txPower_dBm<0 ? -1 : txPower_dBm) << "," // -1 means that no txpower in dBm has been explicitely set in the SpectrumWifiPhy model
                    << dlAckSeqType << ","
                    << gi << ","
                    << (mcs.getValue(staidx)<0 ? "MinstrelHt" : std::to_string(mcs.getValue(staidx))) << ","
                    << channelWidth.getValue(staidx) << ","
                    << AP_channelWidth << ","
                    << nStations << ","
                    << reportManPtr->getAverageLatency () << ","
                    << (reportManPtr->getMaximumLatency () == -DBL_MAX ? -1 : reportManPtr->getMaximumLatency ()) << ","
                    << (reportManPtr->getMinimumLatency () == DBL_MAX ? -1 : reportManPtr->getMinimumLatency ()) << ","
                    << reportManPtr->getStdDevLatency () << ","
                    << reportManPtr->getPacketLossPercentage () << ","
                    << reportManPtr->getThroughputMbps(simulationTime) << "," // Throughput [Mbit/s]
                    << reportManPtr->getAverageSNR () << ","
                    << (reportManPtr->getMaximumSNR () == -DBL_MAX ? -1 : reportManPtr->getMaximumSNR ()) << ","
                    << (reportManPtr->getMinimumSNR () == DBL_MAX ? -1 : reportManPtr->getMinimumSNR ()) << ","
                    << reportManPtr->getAverageRSSI () << ","
                    << (reportManPtr->getMaximumRSSI () == -DBL_MAX ? -1 : reportManPtr->getMaximumRSSI ()) << ","
                    << (reportManPtr->getMinimumRSSI () == DBL_MAX ? -1 : reportManPtr->getMinimumRSSI ()) << ","
                    << reportManPtr->getMostUsedMcsAsInt () << ","
                    << mcsVector[0] << "," << mcsVector[1] << "," << mcsVector[2] << "," << mcsVector[3] << "," << mcsVector[4] << "," << mcsVector[5] << "," << mcsVector[6] << "," << mcsVector[7] << "," << mcsVector[8] << "," << mcsVector[9] << "," << mcsVector[10] << "," << mcsVector[11] << ","
                    << reportManPtr->getUnavailableMcsCount () << ",";

            for(ReportManager::latency_distr_iterator_t it=reportManPtr->getInitialLatencyDistributionIterator();
                it!=reportManPtr->getFinalLatencyDistributionIterator();
                it++) {
                csv_ofstream << reportManPtr->latencyDistributionGetValue (it) << ",";
            }

            csv_ofstream << std::fixed << std::setprecision(10)
                    << reportManPtr->getTotalBytesReceived () << ","
                    << reportManPtr->getTotalBytesTransmitted () << ","
                    << reportManPtr->get_last_t0 () << ","
                    << reportManPtr->get_last_t1 () << ","
                    << twtDuration.getValue(staidx) << ","
                    << reportManPtr->get_last_trx () << ","
                    << (reportManPtr->get_last_teff () <= 1e-6 ? 0 : reportManPtr->get_last_teff ()) << "," // If it is equal to 1 ns, set it to 0, as 1 ns is sometimes added while Scheduling the events in the client to make sure events are scheduled in order, even if they are actually meants to be scheduled at time 0
                    << reportManPtr->get_last_AOI () << ","
                    << (reportManPtr->get_last_t0 () > reportManPtr->get_last_t1 ()) << ","
                    << (reportManPtr->get_last_trx ()-reportManPtr->get_last_t1 ()) << "," // Effective latency
                    << timeElapsedForSta_ns_TI[CLASS_IDX_VECTOR(staidx)]/1e6 << ","
                    << awakeTimeElapsedForSta_ns_TI[CLASS_IDX_VECTOR(staidx)]/1e6 << ","
                    << avgCurrent_mAForSTA[CLASS_IDX_VECTOR(staidx)] << ","
                    << totEnergyConsumedForSta_J[CLASS_IDX_VECTOR(staidx)] << ","
                    << deadlines.getValue(staidx) << ","
                    << (reportManPtr->get_last_trx () <= deadlines.getValue(staidx));

            const std::vector<std::string> states={"TX","RX","CCA_BUSY","SLEEP","IDLE"};

            for(size_t st=0;st<states.size();st++) {
                csv_ofstream << std::fixed << std::setprecision(10) << ","
                             << std::get<0>(stateEnergyTimeData_ForSta[std::make_tuple(staidx, states[st])]) << ","
                             << std::get<1>(stateEnergyTimeData_ForSta[std::make_tuple(staidx, states[st])]) << ","
                             << std::get<2>(stateEnergyTimeData_ForSta[std::make_tuple(staidx, states[st])]) << ","
                             << std::get<3>(stateEnergyTimeData_ForSta[std::make_tuple(staidx, states[st])]);
            }

            csv_ofstream << "," << desiredDuration.getValue(staidx) << ",";
            csv_ofstream << (reportManPtr->getAverageLatency ()!=0 ? (reportManPtr->get_last_trx ()-reportManPtr->get_last_t1 ()) - desiredDuration.getValue(staidx) : 0);

            csv_ofstream << std::endl;
        }

        csv_ofstream.close ();
    } // end of csv logging

    if(direct_ns3_mode_last_iter_rem_B_STA_ID_ptr!=nullptr && direct_ns3_mode_last_iter_rem_B_ptr!=nullptr) {
        for(int staidx=1;staidx<=nStations;staidx++) {
            Ptr<ReportManager> reportManPtr = &(reportMans[CLASS_IDX_VECTOR(staidx)]);

            if(reportManPtr->get_time_limit_exceeded_STA_id()>0) {
                *direct_ns3_mode_last_iter_rem_B_STA_ID_ptr=reportManPtr->get_time_limit_exceeded_STA_id();
                *direct_ns3_mode_last_iter_rem_B_ptr=reportManPtr->get_time_limit_rem_data_bytes ();
                break;
            }
        }
    }
}


int main (int argc, char *argv[]) {
    // Declaring some variable and setting some default values for the simulation parameters
    // bool downlink = true; // True if DL traffic should be generated, false if UL traffic should be generated
    bool useRts = false;
    bool useExtendedBlockAck = false;
    // double simulationTime = CLIENT_TRAFFIC_START_SECOND*1000.0 + 1126.4; // ms (default: 1 second since the clients are started)
    double simulationTime = CLIENT_TRAFFIC_START_SECOND*1000.0+102.4; // Simulating a single beacon interval
    double frequency=2.4; // whether 2.4, 5 or 6 GHz
    std::string dlAckSeqType{"ACK-SU-FORMAT"};
    bool enableUlOfdma = true;
    bool enableBsrp = true;
    std::string mcs_str="";
    Time accessReqInterval{0};
    std::string csv_outfile="output";
    int txPower_dBm = 20; // Any value < 0 will not set any fixed transmission power level in dBm
    int nStations=1; // Number of STAs; each single STA is characterized by a traffic type, number of antennas/MIMO configuration and channel width
    int num_antennas_AP=4; // Number of antennas (MIMO configuration) for the AP
    bool enableTWT=true; // By default TWT is enabled
    int AP_channelWidth=20;
    uint64_t simulation_id=0;
    int gi = 800;
    int globalSeed=-1;

    // Uncomment this to enable a hugely verbose logging on the terminal, down to the PHY layer
    //LogComponentEnableAll(LOG_LEVEL_INFO);

    std::string packet_periodicity_str="";
    std::string num_antennas_str="";
    std::string channelWidth_str="";
    std::string payloadSize_str="";
    std::string trafficType_str="";
    std::string distance_str="";
    std::string t0_str="";
    std::string t1_str="";
    std::string twtDuration_str="";
    std::string STAtype_str="";
    std::string deadlines_str="";

    std::string desired_duration_str="";

    std::string current_policy="unset";

    // Command line arguments
    // STA parameters should be set with a string formatted in the following way:
    // value#value#value and so on (with each value separated by #), where the first value corresponds to the first STA, the second value to the second STA, ...
    CommandLine cmd(__FILE__);
    cmd.AddValue("globalSeed","Global seed for the random number generators in ns-3. Setting this to any number <0 will not set a specific seed and the default ns-3 global seed will be used.",globalSeed);
    cmd.AddValue("simId","Current simulation ID for logging. It can be any unsigned 64-bit integer, except 0. It should be mandatorily specified.",simulation_id);
    cmd.AddValue("frequency","Whether working in the 2.4, 5 or 6 GHz band (other values gets rejected)",frequency);
    cmd.AddValue("packetPeriodicity","[STA parameter] Packet periodicity in seconds.",packet_periodicity_str);
    cmd.AddValue("distance","[STA parameter] Distance in meters between the station and the access point",distance_str);
    cmd.AddValue("simulationTime", "Simulation time in milliseconds", simulationTime);
    cmd.AddValue("useRts", "Enable/disable RTS/CTS", useRts); // Disabled by default
    cmd.AddValue("useExtendedBlockAck", "Enable/disable use of extended BACK", useExtendedBlockAck);
    cmd.AddValue("dlAckType","Ack sequence type for DL OFDMA (ACK-SU-FORMAT, MU-BAR, AGGR-MU-BAR)",dlAckSeqType);
    cmd.AddValue("enableUlOfdma","Enable UL OFDMA (useful if DL OFDMA is enabled and TCP is used)",enableUlOfdma);
    cmd.AddValue("enableBsrp","Enable BSRP (useful if DL and UL OFDMA are enabled and TCP is used)",enableBsrp);
    cmd.AddValue("muSchedAccessReqInterval","Duration of the interval between two requests for channel access made by the MU scheduler",accessReqInterval);
    cmd.AddValue("payloadSize", "[STA parameter] The application payload size in bytes (default: 700)", payloadSize_str);
    cmd.AddValue("txpower","AP and STA transmission power, in dBm. Any value < 0 will not set any fixed transmission power in the Wi-Fi PHY model.",txPower_dBm);
    cmd.AddValue("csv-output-file","Set the name of the output CSV file (without extension!). Default name: output[.csv].",csv_outfile);
    cmd.AddValue("STAnumAntennas","[STA parameter] Number of antennas on each STA device. The number of supported TX and RX spatial streams will be set to be equal to the number of antennas on each STA.",num_antennas_str);
    cmd.AddValue("APnumAntennas","Number of antennas on the AP. The number of supported TX and RX spatial streams will be set to be equal to the number of antennas on the AP.",num_antennas_AP);
    cmd.AddValue("trafficType","[STA parameter] Traffic type for each STA. Can be UL or DL. The traffic type is specified as an integer: 0 = DL (default), 1 = UL.",trafficType_str);
    cmd.AddValue("STAChannelWidth","[STA parameter] Channel width in MHz for the STAs. Supported values: 20, 40, 80, 160 MHz. Maximum channel width: 160 MHz for 5 and 6 GHz, 40 MHz for 2.4 GHz.",channelWidth_str);
    cmd.AddValue("APChannelWidth","Channel width in MHz for the AP. Supported values: 20, 40, 80, 160 MHz. Maximum channel width: 160 MHz for 5 and 6 GHz, 40 MHz for 2.4 GHz.",AP_channelWidth);
    cmd.AddValue("guardInterval","Guard interval in ns. Supported values: 800, 1600, 3200 ns.",gi);
    cmd.AddValue("nStations","Number of STAs. Each STA is characterized by a traffic type, number of antennas/MIMO configuration and channel width. If set to 1, only one AP and STA will be simulated.",nStations);
    cmd.AddValue("mcs","[STA parameter] MCS to be used by each STA; any value <0 means that the simulation should employ rate adaptation with MinstrelHt instead of using a specific MCS.",mcs_str);
    cmd.AddValue("t0","[STA parameter] [TWT approach parameter] Time when traffic is generated by each STA. Used only for logging.",t0_str);
    cmd.AddValue("t1","[STA parameter] [TWT approach parameter] Time when each STA is granted permission to send traffic.",t1_str);
    cmd.AddValue("twtDuration","[STA parameter] [TWT approach parameter] The duration of TWT window.",twtDuration_str);
    cmd.AddValue("STAtype","[STA parameter] [TWT approach parameter] The type of STA (80211...).",STAtype_str);
    cmd.AddValue("twtTriggerBased", "TB if true; non-TB if false",twtTriggerBased);
    cmd.AddValue("maxMuSta", "Max number of STAs the AP can trigger in one MU_UL with Basic TF", maxMuSta);
    cmd.AddValue("deadlines", "[STA parameter] Deadline after which the traffic should be delivered. Used for offline computation after each simulation. A value of 0 or < 0 means \"no deadline\".", deadlines_str);
    cmd.AddValue("enableTWT","Decide whether to enable TWT or not. By default this option is true, i.e., TWT is enabled.",enableTWT);
    cmd.AddValue("policy","If applicable, specify the name of the current TWT scheduling policy. Not setting any policy name will make ns-3-twt write \"unset\" as policy name for every STA in the simulation.",current_policy);
    cmd.AddValue("desiredDuration","[STA parameter] If applicable, specifies a desired latency for each STA, that will be reported in the final output CSV file, besides the difference between the actual and the desired latency value. May be useful for debugging.",desired_duration_str);
    cmd.Parse(argc, argv);

    sim_params_t parameters;
    parameters.simulation_id=simulation_id;
    parameters.useRts=useRts;
    parameters.useExtendedBlockAck=useExtendedBlockAck;
    parameters.simulationTime=simulationTime;
    parameters.frequency=frequency;
    parameters.dlAckSeqType=dlAckSeqType;
    parameters.enableUlOfdma=enableUlOfdma;
    parameters.enableBsrp=enableBsrp;
    parameters.mcs_str=mcs_str;
    parameters.accessReqInterval=accessReqInterval;
    parameters.csv_outfile=csv_outfile;
    parameters.txPower_dBm=txPower_dBm;
    parameters.nStations=nStations;
    parameters.AP_channelWidth=AP_channelWidth;
    parameters.gi=gi;
    parameters.packet_periodicity_str=packet_periodicity_str;
    parameters.num_antennas_str=num_antennas_str;
    parameters.num_antennas_AP=num_antennas_AP;
    parameters.channelWidth_str=channelWidth_str;
    parameters.payloadSize_str=payloadSize_str;
    parameters.trafficType_str=trafficType_str;
    parameters.distance_str=distance_str;
    parameters.t0_str=t0_str;
    parameters.t1_str=t1_str;
    parameters.enableTwt=enableTWT;
    parameters.twtDuration_str=twtDuration_str;
    parameters.STAtype_str=STAtype_str;
    parameters.deadlines_str=deadlines_str;
    parameters.current_policy=current_policy;
    parameters.globalSeed=globalSeed;

    parameters.desired_duration_str=desired_duration_str;

    // Set the global ns-3 seed
    if(globalSeed>=0) {
        SeedManager::SetSeed(globalSeed);
        SeedManager::SetRun(globalSeed);
        RngSeedManager::SetSeed(globalSeed);
        RngSeedManager::SetSeed(globalSeed);
    }

    Ptr<UniformRandomVariable> uv=CreateObject<UniformRandomVariable>();
    uv->SetAttribute("Min",DoubleValue(0.0));
    uv->SetAttribute("Max",DoubleValue(100.0));
    std::cout << "Test random number (if it does not change with the seed there is something wrong): " << uv->GetInteger() << std::endl;

    double simTime_mod_bcn=std::fabs(std::fmod(simulationTime, beaconInterval.GetMicroSeconds()/1000.0));
    if(!(simTime_mod_bcn < 1e-7) && !(std::fabs(simTime_mod_bcn-beaconInterval.GetMicroSeconds()/1000.0)<1e-7)) {
        NS_FATAL_ERROR("Fatal error: the simulation time must be a multiple of the beacon interval for the energy model to work.");
    }

    // if non trigger based but maxMuSTA is not 1, exit program
    if (twtTriggerBased == false && maxMuSta != 1) {
        std::cout<<"Error. Non trigger based but maxMuSTA is not 1. Exiting.";
        return 0;
    }

    run_single_simulation(parameters,nullptr,nullptr);

  return 0;
}
