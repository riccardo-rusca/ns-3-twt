#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define MARGIN_FACTOR 0.01

// Define the DataPoint structure
struct DataPoint {
    int payload_size;
    int most_used_MCS;
    double effective_latency;

    DataPoint(int size, int mcs, double latency)
        : payload_size(size), most_used_MCS(mcs), effective_latency(latency) {}
};

int main() {
    std::string file_path = "mcs-size-map-5-new-red.csv"; // Update with the actual file path
    std::ifstream file(file_path);
    std::vector<DataPoint> mcs_size_map;
    std::string line;

    // Skip the header line
    std::getline(file, line);

    // Read each line from the file
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;

        // Split the line by commas
        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        // Convert tokens to appropriate data types and add to the vector
        if (tokens.size() == 3) {
            int payload_size = std::stoi(tokens[0]);
            int most_used_MCS = std::stoi(tokens[1]);
            double effective_latency = std::stod(tokens[2]);
            mcs_size_map.emplace_back(payload_size, most_used_MCS, effective_latency);
        }
    }

    // Output the C++ vector definition
    std::cout << "std::vector<DataPoint> mcs_size_map_5_GHz={";
    for (size_t i = 0; i < mcs_size_map.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << "{" << mcs_size_map[i].effective_latency*(1+MARGIN_FACTOR) << "," << mcs_size_map[i].payload_size << "," << mcs_size_map[i].most_used_MCS << "}";
    }
    std::cout << "};" << std::endl;

    return 0;
}

