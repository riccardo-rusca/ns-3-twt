#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <unordered_map>
#include "mcs_size_map.h"

std::unordered_map<std::string,int> statype_to_index = {
    {"80211bgn", 0},
    {"80211bg", 1},
    {"80211ac", 2},
    {"80211ax", 3}
};

// Function that returns the closest element, in terms of latency ("key" of the map) to a given latency value, passed as "value"
DataPoint getClosestElement(const std::vector<DataPoint>& dataPoints,double value) {
    auto it = std::min_element(dataPoints.begin(), dataPoints.end(), [value] (DataPoint a, DataPoint b) {
        return std::abs(value - a.effective_latency) < std::abs(value - b.effective_latency);
    });

    if(it == dataPoints.end()) {
        // The closest element is actually the maximum element, in terms of latency, of the vector
        return *std::max_element(dataPoints.begin(),dataPoints.end(),[](DataPoint a, DataPoint b) {
            return std::abs(a.effective_latency) < std::abs(b.effective_latency);
        });
    }

    return *it;
}

// Function that finds the enclosing elements (two elements) of a given value ("target"), in terms of latency, given a vector of DataPoint (i.e., a vector of mappings (latency -> UDP payload size,MCS mapping))
// If "target" is lower than the minimum element of the vector, or higher than the maximum element of the vector (in terms of latency value), the minimum or maximum is returned (so, the size of the output will be just 1)
// If "target" matches the latency value of one of the elements of the vector, that element is returned (so, the size of the output will be just 1)
// IMPORTANT: this function expects "arr" to be SORTED IN ASCENDING ORDER!
std::vector<DataPoint> findEnclosingElements(const std::vector<DataPoint>& arr, double target) {
    // Use lower_bound to find the position
    auto it = std::lower_bound(arr.begin(), arr.end(), target,[](DataPoint pt, double val) {
        return pt.effective_latency<val;
    });

    if (it == arr.begin()) {
        // Target is less than the first element, return the first element
        return {*it};
    } else if (it == arr.end()) {
        // Target is greater than all elements, return the last element
        return {*(it - 1)};
    } else if ((*it).effective_latency == target) {
        // Target matches an element, return the matching element
        return {*it};
    } else {
        // Return the element before it and it
        return {*(it - 1), *it};
    }
}


// Given a vector of DataPoint and a latency value, this function returns either the desired (latency value -> UDP payload size,MCS mapping), if that element exists in "dataPoints",
// or the linear interpolation betwenn the two closest elements ("mappings") if the desired latency value is not directly available in the "dataPoints" vector
std::pair<int, int> interpolate_mcs_size_map(std::vector<DataPoint>& dataPoints, double inputLatency) {
    // Sort the DataPoint vector in ascending order
    std::sort(dataPoints.begin(),dataPoints.end(),[](DataPoint a, DataPoint b) {
        return a.effective_latency<b.effective_latency;
    });

    // If the dataPoint vector is empty, directly return -1 as "unavailable values"
    if (dataPoints.empty()) return {-1, -1};

    // Get the closest element, in term of latency, to the desired input latency value, to get the MCS leading to the closest result
    DataPoint closest_pt=getClosestElement(dataPoints,inputLatency);

    // Create a vector with only the datapoints with the selected MCS (extracting a subvector from the main "dataPoints" vector, selecting only a single MCS value)
    std::vector<DataPoint> mcsDataPoints;
    std::copy_if(dataPoints.begin(), dataPoints.end(), std::back_inserter(mcsDataPoints),
                 [&closest_pt](const DataPoint& dp) {
                     return dp.most_used_MCS == closest_pt.most_used_MCS;
                 }
    );

    // Get the closest values to the desired latency value from the subvector (so, we are fixing now an MCS value)
    std::vector<DataPoint> enclosing_pts=findEnclosingElements(mcsDataPoints,inputLatency);

    int interpolatedPayloadSize=-1;
    if(enclosing_pts.size()==1) {
        // There is an exact match or the desired latency is lower than the minimum or greater than the maximum -> return an element of the map, with no need to interpolate
        interpolatedPayloadSize=enclosing_pts[0].payload_size;
    } else if(enclosing_pts.size()==2) {
        // Linerarly interpolate if no exact match was found in the map
        DataPoint lb=enclosing_pts[0];
        DataPoint ub=enclosing_pts[1];
        double ratio = (inputLatency - lb.effective_latency) / (ub.effective_latency - lb.effective_latency);
        interpolatedPayloadSize = lb.payload_size + ratio * (ub.payload_size - lb.payload_size);
    } else {
        std::cerr << "Fatal error. Zero or more than two elements enclosing the desired latency have been found. There must be a bug in the code!" << std::endl;
        exit(EXIT_FAILURE);
    }

    return {interpolatedPayloadSize, closest_pt.most_used_MCS};
}


// Given a vector of DataPoint and a latency value, this function returns the desired (latency value -> UDP payload size,MCS mapping), if that element exists in "dataPoints", or the linear interpolation betwenn the two closest elements ("mappings") if the desired latency value is not directly available in the "dataPoints" vector
std::string interpolate_mcs_size_map(std::vector<DataPoint>& dataPoints, double inputLatency, int mcs) {
    // Sort the DataPoint vector in ascending order
    std::sort(dataPoints.begin(),dataPoints.end(),[](DataPoint a, DataPoint b) {
        return a.effective_latency < b.effective_latency;
    });

    // If the dataPoint vector is empty, directly return -1 as "unavailable values"
    if (dataPoints.empty()) return "-1";

    // Create a vector with only the datapoints with the selected MCS (extracting a subvector from the main "dataPoints" vector, selecting only a single MCS value)
    std::vector<DataPoint> selectedMcsPoints;
    for (const auto& dp : dataPoints) {
        if (dp.most_used_MCS == mcs) {
            selectedMcsPoints.push_back(dp);
        }
    }

    // If selectedMcsPoints is empty, return -1 as "unavailable values"
    if (selectedMcsPoints.empty()) return "-1";

    // Get the closest values to the desired latency value from the subvector
    std::vector<DataPoint> enclosing_pts=findEnclosingElements(selectedMcsPoints,inputLatency);

    int interpolatedPayloadSize=-1;
    if(enclosing_pts.size()==1) {
        // There is an exact match or the desired latency is lower than the minimum or greater than the maximum -> return an element of the map, with no need to interpolate
        interpolatedPayloadSize=enclosing_pts[0].payload_size;
    } else if(enclosing_pts.size()==2) {
        // Linerarly interpolate if no exact match was found in the map
        DataPoint lb=enclosing_pts[0];
        DataPoint ub=enclosing_pts[1];
        double ratio = (inputLatency - lb.effective_latency) / (ub.effective_latency - lb.effective_latency);
        interpolatedPayloadSize = lb.payload_size + ratio * (ub.payload_size - lb.payload_size);
    } else {
        std::cerr << "Fatal error. Zero or more than two elements enclosing the desired latency have been found. There must be a bug in the code!" << std::endl;
        exit(EXIT_FAILURE);
    }

    return std::to_string(interpolatedPayloadSize);
}


// This function returns an integer random value between min and max, using rand(), and trying to avoid any modulo bias in the result. May be slower to execute than rand_pseudouniform(). This code is taken from https://github.com/francescoraves483/LaMP_LaTe and licensed under GPLv2
int rand_uniform(int min,int max) {
	int u;
	int n=max-min+1;

	do {
		u=rand();
	} while(u>=(RAND_MAX-RAND_MAX%n));

	return min+rand()%n;
}

// Function that returns true if the two intervals [a,b] and [c,d] overlap
bool doIntervalsOverlap(int a, int b, int c, int d) {
    // Check if the start point of one interval lies within the other interval
    if ((a <= c && c <= b) || (c <= a && a <= d)) {
        return true;
    }

    // Check if the end point of one interval lies within the other interval
    if ((a <= d && d <= b) || (c <= b && b <= d)) {
        return true;
    }

    return false;
}

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char **argv) {
	int fixed_dist=3; // Fixed distance of the STAs in meters
	int gi=800; // 800 ns Guard Interval
	int txPower_dBm=20; // txpower fixed to 20 dBm (max limit in EU for standard Wi-Fi)
	int num_antennas_AP=1; // AP is 1x1 too
	double frequency=2.4; // We work in the 2.4 GHz frequency band
    double margin_percentage=0.0; // [%]

    int num_ss_simulations=1;

    std::string input_file="input.csv";

    char optc;
	while((optc=getopt (argc, argv, "f:m:h"))!=-1) {
		switch(optc) {
			case 'f':
				input_file=std::string(optarg);
				break;

            case 'm':
                margin_percentage=std::stod(optarg);
                break;

			case 'h':
				printf("You can optionally specify an input file name with -f. If not specified, the default name is \"input.csv\".\n");
				exit(EXIT_SUCCESS);
				break;

			default:
				fprintf(stderr,"Error. Wrong command line options specified. Aborting execution.\n");
				exit(EXIT_FAILURE);
				break;
		}
	}

    std::ifstream file(input_file.c_str());
    if (!file.is_open()) {
        std::cerr << "Error opening file " << input_file << std::endl;
        return 1;
    }

    std::string line;
    int globalSeed=1;
    //std::getline(file, line);
    while (std::getline(file, line)) {
        std::string trafficType_str="";
        std::string num_antennas_str="";
        std::string distance_str="";
        std::string payloadSize_str="";
        std::string TWTdurations_str="";
        std::string STAtypes_ns3_str="";
        std::string mcs_ns3_str="";
        std::string desiredDuration_ns3_str="";

        std::vector<std::string> tokens = split(line, ',');

        std::string simulation_id = tokens[0];
        int nStations = stoi(tokens[1]);
        std::string current_policy = tokens[2];
        std::string t0_str = tokens[3];
        std::string t1_str = tokens[4];
        std::string duration_str = tokens[5];
        std::string mcs_str = tokens[6];
        std::string deadlines_str = tokens[7];
        std::string STAtype_str = tokens[8];
        std::vector<std::string> mcs_values = split(mcs_str, '#');
        std::vector<std::string> duration_values = split(duration_str, '#');
        std::vector<std::string> statype_values = split(STAtype_str, '#');

        for(int i = 0; i < nStations; i++) {
            distance_str += std::to_string(fixed_dist); // See the fixed_dist parameter above
            trafficType_str += "1"; // UL traffic from STA to AP
            num_antennas_str += "1"; // 1x1 configuration to simulate a sensor-like configuration
            TWTdurations_str += std::to_string(std::ceil(std::stod(duration_values[i])+1.001));
            STAtypes_ns3_str += std::to_string(statype_to_index[statype_values[i]]);

            std::pair<int, int> payloadMCSpair;
            std::string payload;

            double margin_integer = margin_percentage/100.0;
            double scaled_duration = -1.0;

            if(duration_values[i]=="1") {
                scaled_duration=std::stod(duration_values[i]); // Do not scale for values equal to 1, corresponding to jobs that shall not be scheduled
            } else {
                scaled_duration=std::stod(duration_values[i])*(1-margin_integer);
            }

            // [TBR] Remove
            // mcs_values[i] = "7";

            // desiredDuration_ns3_str += std::to_string(scaled_duration);
            desiredDuration_ns3_str += duration_values[i];

            if(frequency==2.4) {
                if (std::stoi(mcs_values[i]) == -1) {
                    payloadMCSpair = interpolate_mcs_size_map(mcs_size_map_2_4_GHz_1perc, scaled_duration);
                } else {
                    payload = interpolate_mcs_size_map(mcs_size_map_2_4_GHz_1perc, scaled_duration, std::stoi(mcs_values[i]));
                }
            } else if(frequency==5) {
                if (std::stoi(mcs_values[i]) == -1) {
                    payloadMCSpair = interpolate_mcs_size_map(mcs_size_map_5_GHz_1perc, scaled_duration);
                } else {
                    payload = interpolate_mcs_size_map(mcs_size_map_5_GHz_1perc, scaled_duration, std::stoi(mcs_values[i]));
                }
            } else {
                std::cerr << "Error. Invalid frequency specified. The launcher currently supports frequencies of 2.4 GHz and 5 GHz only for IEEE 802.11ax." << std::endl;
            }

            if (std::stoi(mcs_values[i]) == -1) {
                payloadSize_str+= std::to_string(payloadMCSpair.first);
                mcs_ns3_str += std::to_string(payloadMCSpair.second);
            } else {
                payloadSize_str+=payload;
                mcs_ns3_str += mcs_values[i];
            }

            if(i<nStations-1) {
                trafficType_str+="#";
                num_antennas_str+="#";
                distance_str+="#";
                payloadSize_str+="#";
                TWTdurations_str+="#";
                STAtypes_ns3_str+="#";
                mcs_ns3_str+="#";
                desiredDuration_ns3_str+="#";
            }

            // [TBR] Remove
//            std::fprintf(stdout,"margin_percentage=%.6lf, margin_integer=%.6lf, scale_factor=%.6lf, duration_original=%.7lf, duration_scaled=%.7lf \n",margin_percentage,margin_integer,(1-margin_integer),std::stod(duration_values[i]),std::stod(duration_values[i])*(1-margin_integer));
//            std::pair<int, int> printPair_original = interpolate_mcs_size_map(mcs_size_map_2_4_GHz, std::stod(duration_values[i]));
//            std::pair<int, int> printPair_scaled = interpolate_mcs_size_map(mcs_size_map_2_4_GHz, std::stod(duration_values[i])*(1-margin_integer));
//            std::fprintf(stdout,"[ORIG] mcs_values=%d, payloadSize_str=%d \n",printPair_original.second,printPair_original.first);
//            std::fprintf(stdout,"[SCAL] mcs_values=%d, payloadSize_str=%d \n",printPair_scaled.second,printPair_scaled.first);
//            getchar();
        }

        // Generate the command
        // Replace "80211ax_simulations_v3" with your application name in case this is not the correct name
        // std::string command="cd .. && cd ns-3-dev-git && ./ns3 run \"80211ax_simulations_v3";
        std::string command="cd .. && ./ns3 --run \"twtUDP";

        // Add the options (remember the space before the option name!)
        command+=(" --simId=" + simulation_id);
        command+=(" --policy=" + current_policy);
        command+=(" --frequency=" + std::to_string(frequency));
        command+=(" --distance=" + distance_str);
        command+=(" --payloadSize=" + payloadSize_str);
        command+=(" --txpower=" + std::to_string(txPower_dBm));
        command+=(" --csv-output-file=" + std::string("outputTWT/output"));
        command+=(" --STAnumAntennas=" + num_antennas_str);
        command+=(" --APnumAntennas=" + std::to_string(num_antennas_AP));
        command+=(" --trafficType=" + trafficType_str);
        command+=(" --guardInterval=" + std::to_string(gi));
        command+=(" --nStations=" + std::to_string(nStations));
        command+=(" --mcs=" + mcs_ns3_str);
        command+=(" --t0=" + t0_str);
        command+=(" --t1=" + t1_str);
        command+=(" --twtDuration=" + TWTdurations_str);
        command+=(" --deadlines=" + deadlines_str);
        command+=(" --STAtype=" + STAtypes_ns3_str);
        command+=(" --desiredDuration=" + desiredDuration_ns3_str);
        // Set a different global seed for each simulation
        command+=(" --globalSeed=" + std::to_string(globalSeed*1000));
        command+=(" --enableTWT=true");
        command+="\"";

        // [TBR] Restore
        std::cout<<"["<<simulation_id<<"] Sim no: " << num_ss_simulations << " - mcs: " << mcs_ns3_str << " - payloadSizes: " << payloadSize_str
                 << " - nStations: " << nStations << " - guard_interval_ns: " << gi << " - dist_m: " << fixed_dist
                 << " - t0: " << t0_str << " - t1: " << t1_str << " - STAtype: " << STAtype_str << std::endl;

        std::cout << "Command: " << command << std::endl;
        system(command.c_str());

        num_ss_simulations++;
        // globalSeed++; // Assign a different global seed to each simulation

        // Trying a more complex seed pattern for each simulation
        globalSeed+=2;
        globalSeed*=3;
    }

    file.close();
	return 0;
}
