#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>

#include "../DataStructures/Intermediate/Data.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: InspectTransferTimes <intermediate_binary>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::cout << "Loading " << filename << "..." << std::endl;
    Intermediate::Data data = Intermediate::Data::FromBinary(filename);

    size_t nonZeroCount = 0;
    size_t totalStops = data.stops.size();
    std::map<int, int> transferTimeHistogram;

    std::cout << "Total stops: " << totalStops << std::endl;

    for (size_t i = 0; i < totalStops; ++i) {
        int buffer = data.stops[i].minTransferTime;
        if (buffer > 0) {
            nonZeroCount++;
        }
        transferTimeHistogram[buffer]++;
        
        if (nonZeroCount <= 10 && buffer > 0) {
             std::cout << "Stop " << i << " has MinTransferTime: " << buffer << " seconds" << std::endl;
        }
    }

    std::cout << "\nResults:" << std::endl;
    std::cout << "Stops with MinTransferTime > 0: " << nonZeroCount << " (" << (double)nonZeroCount / totalStops * 100.0 << "%)" << std::endl;
    
    std::cout << "\nDistribution of MinTransferTimes:" << std::endl;
    for (auto const& [time, count] : transferTimeHistogram) {
        std::cout << "  " << time << "s: " << count << " stops" << std::endl;
    }

    return 0;
}
