#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../Helpers/Types.h"
#include "../DataStructures/Intermediate/Data.h"
#include "../DataStructures/Graph/TimeDependentGraph.h"

struct EdgeKeyHash {
    std::size_t operator()(const std::pair<Vertex, Vertex>& p) const noexcept {
        return std::hash<size_t>()(size_t(p.first)) ^ (std::hash<size_t>()(size_t(p.second)) << 1);
    }
};

struct DepArr {
    int dep;
    int arr;
};

struct ViolationSample {
    Vertex u;
    Vertex v;
    int prevDep;
    int prevArr;
    int dep;
    int arr;
};

struct StopInconsistencySample {
    size_t tripId;
    size_t stopIndex;
    Vertex stop;
    int arr;
    int dep;
};

struct CaseSpec {
    Vertex u1{noVertex};
    Vertex v1{noVertex};
    Vertex u2{noVertex};
    Vertex v2{noVertex};
    int dep1{0};
    int arr1{0};
    int dep2{0};
    int arr2{0};
    bool enabled{false};
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: NonFifoCheck <intermediate.binary> [maxSamples] [--trip <tripId>] [--check-seq u1 v1 dep1 arr1 u2 v2 dep2 arr2]\n";
        return 1;
    }

    std::string inputFile;
    int maxSamples = 10;
    bool maxSamplesSet = false;
    bool hasTripId = false;
    size_t tripId = 0;
    CaseSpec caseSpec;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--trip" && i + 1 < argc) {
            tripId = static_cast<size_t>(std::stoull(argv[++i]));
            hasTripId = true;
        } else if (arg == "--check-seq" && i + 8 < argc) {
            caseSpec.u1 = Vertex(std::stoul(argv[++i]));
            caseSpec.v1 = Vertex(std::stoul(argv[++i]));
            caseSpec.dep1 = std::stoi(argv[++i]);
            caseSpec.arr1 = std::stoi(argv[++i]);
            caseSpec.u2 = Vertex(std::stoul(argv[++i]));
            caseSpec.v2 = Vertex(std::stoul(argv[++i]));
            caseSpec.dep2 = std::stoi(argv[++i]);
            caseSpec.arr2 = std::stoi(argv[++i]);
            caseSpec.enabled = true;
        } else if (inputFile.empty()) {
            inputFile = arg;
        } else if (!maxSamplesSet) {
            maxSamples = std::stoi(arg);
            maxSamplesSet = true;
        }
    }

    if (inputFile.empty()) {
        std::cerr << "Usage: NonFifoCheck <intermediate.binary> [maxSamples] [--trip <tripId>] [--check-seq u1 v1 dep1 arr1 u2 v2 dep2 arr2]\n";
        return 1;
    }

    std::cout << "Loading Intermediate data from: " << inputFile << std::endl;
    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);

    if (hasTripId) {
        if (tripId >= inter.trips.size()) {
            std::cerr << "Trip id " << tripId << " is out of bounds (0, " << inter.trips.size() << ")\n";
            return 1;
        }
        const Intermediate::Trip& trip = inter.trips[tripId];
        std::cout << "\n=== Trip " << tripId << " ===\n";
        std::cout << trip << "\n";
        for (size_t i = 0; i < trip.stopEvents.size(); ++i) {
            const Intermediate::StopEvent& se = trip.stopEvents[i];
            std::cout << "  [" << i << "] stop=" << se.stopId
                      << " arr=" << se.arrivalTime
                      << " dep=" << se.departureTime << "\n";
        }

        if (caseSpec.enabled) {
            bool found = false;
            for (size_t i = 0; i + 2 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& s0 = trip.stopEvents[i];
                const Intermediate::StopEvent& s1 = trip.stopEvents[i + 1];
                const Intermediate::StopEvent& s2 = trip.stopEvents[i + 2];

                if (Vertex(s0.stopId) == caseSpec.u1 && Vertex(s1.stopId) == caseSpec.v1 &&
                    s0.departureTime == caseSpec.dep1 && s1.arrivalTime == caseSpec.arr1 &&
                    Vertex(s1.stopId) == caseSpec.u2 && Vertex(s2.stopId) == caseSpec.v2 &&
                    s1.departureTime == caseSpec.dep2 && s2.arrivalTime == caseSpec.arr2) {
                    found = true;
                    std::cout << "\nCase match found at stop index " << i << "\n";
                    break;
                }
            }
            if (!found) {
                std::cout << "\nCase match not found in trip " << tripId << "\n";
            }
        }
    }

    std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DepArr>, EdgeKeyHash> tripSegments;
    tripSegments.reserve(inter.trips.size() * 4);

    size_t totalSegments = 0;
    size_t totalStopEvents = 0;
    size_t inconsistentStopEvents = 0;
    int maxInconsistency = 0;
    StopInconsistencySample maxInconsistencySample{};
    std::vector<StopInconsistencySample> inconsistencySamples;
    inconsistencySamples.reserve(maxSamples);
    for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
        const Intermediate::Trip& trip = inter.trips[tripId];
        totalStopEvents += trip.stopEvents.size();
        for (size_t i = 0; i < trip.stopEvents.size(); ++i) {
            const Intermediate::StopEvent& stopEvent = trip.stopEvents[i];
            if (stopEvent.arrivalTime > stopEvent.departureTime) {
                inconsistentStopEvents++;
                const int diff = stopEvent.arrivalTime - stopEvent.departureTime;
                if (diff > maxInconsistency) {
                    maxInconsistency = diff;
                    maxInconsistencySample = {tripId, i, Vertex(stopEvent.stopId), stopEvent.arrivalTime, stopEvent.departureTime};
                }
                if (inconsistencySamples.size() < static_cast<size_t>(maxSamples)) {
                    inconsistencySamples.push_back({tripId, i, Vertex(stopEvent.stopId), stopEvent.arrivalTime, stopEvent.departureTime});
                }
            }
        }
        for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
            const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
            const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];
            const Vertex u = Vertex(stopEventU.stopId);
            const Vertex v = Vertex(stopEventV.stopId);

            const int buffer = (u < inter.stops.size()) ? inter.stops[u].minTransferTime : 0;
            const int dep = stopEventU.departureTime - buffer;
            const int arr = stopEventV.arrivalTime;

            tripSegments[{u, v}].push_back({dep, arr});
            totalSegments++;
        }
    }

    size_t edgesWithViolations = 0;
    size_t totalViolations = 0;
    int maxDecrease = 0;
    ViolationSample maxSample{};
    std::vector<ViolationSample> samples;
    samples.reserve(maxSamples);

    for (auto& [edge, trips] : tripSegments) {
        std::sort(trips.begin(), trips.end(), [](const DepArr& a, const DepArr& b) {
            if (a.dep != b.dep) return a.dep < b.dep;
            return a.arr < b.arr;
        });

        bool edgeHasViolation = false;
        int prevArr = trips.front().arr;
        int prevDep = trips.front().dep;

        for (size_t i = 1; i < trips.size(); ++i) {
            const int dep = trips[i].dep;
            const int arr = trips[i].arr;

            if (arr < prevArr) {
                edgeHasViolation = true;
                totalViolations++;
                const int decrease = prevArr - arr;
                if (decrease > maxDecrease) {
                    maxDecrease = decrease;
                    maxSample = {edge.first, edge.second, prevDep, prevArr, dep, arr};
                }
                if (samples.size() < static_cast<size_t>(maxSamples)) {
                    samples.push_back({edge.first, edge.second, prevDep, prevArr, dep, arr});
                }
            }

            prevArr = arr;
            prevDep = dep;
        }

        if (edgeHasViolation) edgesWithViolations++;
    }

    std::cout << "\n=== Non-FIFO Analysis ===\n";
    std::cout << "Trips: " << inter.trips.size() << "\n";
    std::cout << "Stop events: " << totalStopEvents << "\n";
    std::cout << "Segments (u->v pairs): " << totalSegments << "\n";
    std::cout << "Edges (unique u->v): " << tripSegments.size() << "\n";
    std::cout << "Edges with non-FIFO: " << edgesWithViolations << "\n";
    std::cout << "Total non-FIFO violations: " << totalViolations << "\n";

    if (edgesWithViolations > 0) {
        std::cout << "Max arrival decrease: " << maxDecrease << " (edge " << maxSample.u << "->" << maxSample.v
                  << ", prev dep/arr=" << maxSample.prevDep << "/" << maxSample.prevArr
                  << ", dep/arr=" << maxSample.dep << "/" << maxSample.arr << ")\n";
    }

    if (!samples.empty()) {
        std::cout << "\nSample violations (up to " << maxSamples << "):\n";
        for (const auto& s : samples) {
            std::cout << "  Edge " << s.u << "->" << s.v
                      << " | prev dep/arr=" << s.prevDep << "/" << s.prevArr
                      << " | dep/arr=" << s.dep << "/" << s.arr << "\n";
        }
    }

    std::cout << "\n=== Stop Event Inconsistency (arr > dep) ===\n";
    std::cout << "Inconsistent stop events: " << inconsistentStopEvents << "\n";
    if (inconsistentStopEvents > 0) {
        std::cout << "Max arr-dep: " << maxInconsistency << " (trip " << maxInconsistencySample.tripId
                  << ", stopIndex " << maxInconsistencySample.stopIndex
                  << ", stop " << maxInconsistencySample.stop
                  << ", arr/dep=" << maxInconsistencySample.arr << "/" << maxInconsistencySample.dep << ")\n";
    }

    if (!inconsistencySamples.empty()) {
        std::cout << "\nSample inconsistent stop events (up to " << maxSamples << "):\n";
        for (const auto& s : inconsistencySamples) {
            std::cout << "  Trip " << s.tripId << ", stopIndex " << s.stopIndex
                      << " (stop " << s.stop << ") arr/dep=" << s.arr << "/" << s.dep << "\n";
        }
    }

    return 0;
}
