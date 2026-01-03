#pragma once

// ============================================================================
// TimeDependentGraphFC - Fractional Cascading Optimization
// ============================================================================
// This graph variant implements fractional cascading to speed up binary search
// across multiple edges from the same vertex. Instead of doing separate binary
// searches on each outgoing edge, we build a cascaded structure that allows
// jumping between sorted lists efficiently.
//
// Based on the Python implementation in graph.fractional_cascading_precomputation()
// ============================================================================

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <map>
#include <unordered_map>
#include <iostream>

// --- CORE INCLUDES ---
#include "../../Helpers/Types.h"
#include "../../Helpers/Meta.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/Classes/DynamicGraph.h"
#include "../../Helpers/IO/Serialization.h"

// --- CONSTRUCTOR DEPENDENCIES ---
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../DataStructures/Graph/TimeDependentGraphClassic.h"  // For filterDominatedConnections

using TransferGraph = ::TransferGraph;

// =========================================================================
// Fractional Cascading Data Structures
// =========================================================================

// Pointer structure for fractional cascading
// [start_index, next_loc] where:
// - start_index: index in the original array (arr[i])
// - next_loc: location in the next cascaded array (m_arr[i+1])
struct FCPointer {
    int startIndex;
    int nextLoc;
};

// Per-vertex fractional cascading structure
struct FractionalCascadingData {
    // m_arr[i] = cascaded sorted departure times for level i
    // Each level includes its own departure times plus every other element from level i-1
    std::vector<std::vector<int>> m_arr;

    // arr[i] = original departure times for edge to reachable_nodes[i]
    std::vector<std::vector<int>> arr;

    // pointers[i][j] = {startIndex, nextLoc} for position j in m_arr[i]
    std::vector<std::vector<FCPointer>> pointers;

    // reachable_nodes[i] = the target vertex for level i (nodes with transit connections)
    std::vector<Vertex> reachableNodes;

    // walking_nodes = vertices reachable only by walking (no transit)
    std::vector<Vertex> walkingNodes;

    // Helper: given a departure time, returns the index in m_arr[0]
    inline int bisectLeft(int departureTime) const noexcept {
        if (m_arr.empty() || m_arr[0].empty()) return 0;

        const auto& firstLevel = m_arr[0];
        auto it = std::lower_bound(firstLevel.begin(), firstLevel.end(), departureTime);
        return std::distance(firstLevel.begin(), it);
    }
};

// =========================================================================
// TimeDependentGraphFC Class
// =========================================================================

class TimeDependentGraphFC {
public:
    using EdgeTripsHandle = ::EdgeTripsHandle;

private:
    using TDEdgeAttributes = Meta::List<
        ::Attribute<FromVertex, Vertex>,
        ::Attribute<ToVertex, Vertex>,
        ::Attribute<Valid, bool>,
        ::Attribute<IncomingEdgePointer, size_t>,
        ::Attribute<ReverseEdge, Edge>,
        ::Attribute<Function, EdgeTripsHandle>
    >;

    using TDVertexAttributes = Meta::List<
        ::Attribute<BeginOut, Edge>,
        ::Attribute<OutDegree, size_t>,
        ::Attribute<IncomingEdges, std::vector<Edge>>
    >;

    using UnderlyingGraph = DynamicGraphImplementation<TDVertexAttributes, TDEdgeAttributes>;

    UnderlyingGraph graph;
    std::vector<int> minTransferTimeByVertex;

public:
    struct TripLeg {
        int arrivalTime;
        Vertex stopId;
    };

    // Standard graph data
    std::vector<DiscreteTrip> allDiscreteTrips;
    std::vector<int> allSuffixMinArrivals;

private:
    std::vector<uint32_t> tripOffsets;
    std::vector<TripLeg> allTripLegs;

    // [FRACTIONAL CASCADING] Per-vertex cascading structures
    std::vector<FractionalCascadingData> fcData;

    // [OPTIMIZATION] Edge lookup map for O(1) access during FC relaxation
    // Maps (u, v) -> Edge to avoid iterating through adjacency list
    std::unordered_map<std::pair<Vertex, Vertex>, Edge, VertexPairHash> edgeMap;

public:
    TimeDependentGraphFC() = default;

    // Standard graph interface methods
    inline bool getNextStop(const int tripId, const uint16_t currentStopIndex, Vertex& outStop, int& outArrival) const noexcept {
        if (tripId < 0 || (size_t)tripId + 1 >= tripOffsets.size()) return false;
        const uint32_t currentTripStart = tripOffsets[tripId];
        const uint32_t nextTripStart = tripOffsets[tripId + 1];
        const uint32_t absoluteIndex = currentTripStart + currentStopIndex + 1;
        if (absoluteIndex < nextTripStart) {
            const TripLeg& leg = allTripLegs[absoluteIndex];
            outStop = leg.stopId;
            outArrival = leg.arrivalTime;
            return true;
        }
        return false;
    }

    inline const TripLeg& getTripLeg(const size_t index) const noexcept {
        return allTripLegs[index];
    }

    inline size_t getNumStopEvents() const noexcept {
        return allTripLegs.size();
    }

    inline uint32_t getTripOffset(const int tripId) const noexcept {
        return tripOffsets[tripId];
    }

    inline const DiscreteTrip* getTripsBegin(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex];
    }

    inline const DiscreteTrip* getTripsEnd(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex + h.tripCount];
    }

    inline const int* getSuffixMinBegin(const EdgeTripsHandle& h) const noexcept {
        return &allSuffixMinArrivals[h.firstSuffixIndex];
    }

    struct FoundTrip {
        int tripId = -1;
    };

    inline FoundTrip findMatchingTrip(Vertex u, Vertex v, int minDepTime, int atArrTime) const noexcept {
        for (const Edge e : graph.edgesFrom(u)) {
            if (graph.get(ToVertex, e) == v) {
                const EdgeTripsHandle& h = graph.get(Function, e);
                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);

                auto it = std::lower_bound(begin, end, minDepTime,
                    [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

                for (; it != end; ++it) {
                    if (it->arrivalTime == atArrTime) {
                         return {it->tripId};
                    }
                }
            }
        }
        return {-1};
    }

    // [FRACTIONAL CASCADING] Get the FC data for a vertex
    inline const FractionalCascadingData& getFCData(Vertex u) const noexcept {
        static const FractionalCascadingData emptyData;
        if (u >= fcData.size()) return emptyData;
        return fcData[u];
    }

    // [FRACTIONAL CASCADING] Check if vertex has FC data
    inline bool hasFCData(Vertex u) const noexcept {
        return u < fcData.size() && !fcData[u].m_arr.empty();
    }

    // [FACTORY] Build graph from Intermediate data
    inline static TimeDependentGraphFC FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraphFC tdGraph;
        const size_t numStops = inter.numberOfStops();
        const size_t numVertices = inter.transferGraph.numVertices();

        // 1. TOPOLOGY SETUP
        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        tdGraph.minTransferTimeByVertex.assign(numVertices, 0);
        for (size_t s = 0; s < std::min(numStops, numVertices); ++s) {
            tdGraph.minTransferTimeByVertex[s] = inter.stops[s].minTransferTime;
        }

        // 2. ROUTE DECOMPOSITION INTO EDGES
        std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DiscreteTrip>, VertexPairHash> tripSegments;
        tripSegments.reserve(inter.trips.size() * 10);

        std::cout << "Building trip segments..." << std::flush;
        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
                const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];
                const Vertex u = Vertex(stopEventU.stopId);
                const Vertex v = Vertex(stopEventV.stopId);

                const int buffer = (u < inter.stops.size()) ? inter.stops[u].minTransferTime : 0;

                tripSegments[{u, v}].emplace_back(DiscreteTrip{
                    .departureTime = stopEventU.departureTime - buffer,
                    .arrivalTime = stopEventV.arrivalTime,
                    .tripId = (int)tripId,
                    .departureStopIndex = (uint16_t)i
                });
            }
        }
        std::cout << " done." << std::endl;

        // 3. FLATTEN TRIP LEGS
        tdGraph.tripOffsets.reserve(inter.trips.size() + 1);

        size_t totalStops = 0;
        for (const auto& trip : inter.trips) {
            tdGraph.tripOffsets.push_back(totalStops);
            totalStops += trip.stopEvents.size();
        }

        tdGraph.tripOffsets.push_back(totalStops);
        tdGraph.allTripLegs.resize(totalStops);

        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];
            const size_t baseOffset = tdGraph.tripOffsets[tripId];

            for (size_t i = 0; i < trip.stopEvents.size(); ++i) {
                tdGraph.allTripLegs[baseOffset + i] = {
                    trip.stopEvents[i].arrivalTime,
                    Vertex(trip.stopEvents[i].stopId)
                };
            }
        }

        // 4. TRANSFER GRAPH PROCESSING
        std::unordered_map<std::pair<Vertex, Vertex>, int, VertexPairHash> minTransferTimes;
        minTransferTimes.reserve(inter.transferGraph.numEdges());

        const Intermediate::TransferGraph& interTransferGraph = inter.transferGraph;

        for (const Vertex u : interTransferGraph.vertices()) {
            for (const Edge edge : interTransferGraph.edgesFrom(u)) {
                const Vertex v = interTransferGraph.get(ToVertex, edge);
                const int travelTime = interTransferGraph.get(TravelTime, edge);
                auto key = std::make_pair(u, v);
                auto it = minTransferTimes.find(key);
                if (it == minTransferTimes.end()) {
                    minTransferTimes[key] = travelTime;
                } else {
                    it->second = std::min(it->second, travelTime);
                }
            }
        }

        // 5. GRAPH COMPILATION WITH DOMINATED CONNECTION FILTERING
        std::cout << "Creating time-dependent edges with domination filtering..." << std::flush;
        size_t edgeCount = 0;

        tdGraph.allDiscreteTrips.reserve(tripSegments.size() * 5);
        tdGraph.allSuffixMinArrivals.reserve(tripSegments.size() * 5);

        size_t totalTripsBeforeFilter = 0;
        size_t totalTripsAfterFilter = 0;

        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            auto transferIt = minTransferTimes.find({u, v});
            int walkTime = (transferIt != minTransferTimes.end()) ? transferIt->second : never;
            if (transferIt != minTransferTimes.end()) {
                minTransferTimes.erase(transferIt);
            }

            std::sort(trips.begin(), trips.end());

            totalTripsBeforeFilter += trips.size();

            // Apply dominated connection filtering (same as Classic)
            std::vector<DiscreteTrip> filteredTrips = TimeDependentGraphClassic::filterDominatedConnections(trips, walkTime);

            totalTripsAfterFilter += filteredTrips.size();

            if (filteredTrips.empty() && walkTime == never) {
                continue;
            }

            uint32_t firstTripIdx = tdGraph.allDiscreteTrips.size();
            uint32_t firstSuffixIdx = tdGraph.allSuffixMinArrivals.size();

            tdGraph.allDiscreteTrips.insert(tdGraph.allDiscreteTrips.end(), filteredTrips.begin(), filteredTrips.end());

            size_t startSize = tdGraph.allSuffixMinArrivals.size();
            tdGraph.allSuffixMinArrivals.resize(startSize + filteredTrips.size());
            if (!filteredTrips.empty()) {
                tdGraph.allSuffixMinArrivals.back() = filteredTrips.back().arrivalTime;
                for (int i = int(filteredTrips.size()) - 2; i >= 0; --i) {
                    tdGraph.allSuffixMinArrivals[startSize + i] =
                        std::min(filteredTrips[i].arrivalTime, tdGraph.allSuffixMinArrivals[startSize + i + 1]);
                }
            }

            EdgeTripsHandle handle;
            handle.firstTripIndex = firstTripIdx;
            handle.tripCount = (uint32_t)filteredTrips.size();
            handle.firstSuffixIndex = firstSuffixIdx;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        std::cout << " done (" << edgeCount << " edges created)" << std::endl;

        // Print filtering statistics
        if (totalTripsBeforeFilter > 0) {
            double reductionPercent = 100.0 * (1.0 - (double)totalTripsAfterFilter / totalTripsBeforeFilter);
            std::cout << "Domination filtering: " << totalTripsBeforeFilter
                      << " -> " << totalTripsAfterFilter << " trips ("
                      << reductionPercent << "% reduction)" << std::endl;
        }

        // 6. PURE WALKING EDGES
        for (const auto& pair : minTransferTimes) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            const int walkTime = pair.second;

            EdgeTripsHandle handle;
            handle.firstTripIndex = 0;
            handle.tripCount = 0;
            handle.firstSuffixIndex = 0;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        std::cout << " done (" << edgeCount << " edges created)" << std::endl;

        // 7. [NEW] BUILD FRACTIONAL CASCADING STRUCTURES
        std::cout << "Building fractional cascading structures..." << std::flush;
        tdGraph.buildFractionalCascading();
        std::cout << " done." << std::endl;

        return tdGraph;
    }

    // [FRACTIONAL CASCADING] Build the cascaded structures for all vertices
    inline void buildFractionalCascading() noexcept {
        fcData.resize(graph.numVertices());

        for (size_t i = 0; i < graph.numVertices(); ++i) {
            buildFractionalCascadingForVertex(Vertex(i));
        }
    }

private:
    // [FRACTIONAL CASCADING] Build cascaded structure for a single vertex
    // Based on Python: fractional_cascading_precomputation()
    // IMPORTANT: Builds on the FILTERED trip data (after dominated connection removal)
    inline void buildFractionalCascadingForVertex(Vertex u) noexcept {
        FractionalCascadingData& fc = fcData[u];

        // Collect all outgoing edges with their departure times (FROM FILTERED DATA)
        struct EdgeInfo {
            Vertex target;
            std::vector<int> departureTimes;
            size_t tripCount;
        };

        std::vector<EdgeInfo> edgeInfos;

        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const EdgeTripsHandle& h = graph.get(Function, e);

            if (h.tripCount > 0) {
                EdgeInfo info;
                info.target = v;
                info.tripCount = h.tripCount;

                // Extract departure times FROM THE FILTERED TRIPS
                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);
                for (const DiscreteTrip* it = begin; it != end; ++it) {
                    info.departureTimes.push_back(it->departureTime);
                }

                edgeInfos.push_back(std::move(info));
            } else if (h.walkTime != never) {
                // Walking-only edge
                fc.walkingNodes.push_back(v);
            }
        }

        if (edgeInfos.empty()) return;

        // Sort by number of departures (ascending - Python strategy)
        std::sort(edgeInfos.begin(), edgeInfos.end(),
            [](const EdgeInfo& a, const EdgeInfo& b) { return a.tripCount < b.tripCount; });

        // Build cascaded arrays (in reverse order, then reverse at end)
        std::vector<std::vector<int>> m_arr_temp;
        std::vector<std::vector<int>> arr_temp;
        std::vector<Vertex> reachable_temp;

        for (size_t i = 0; i < edgeInfos.size(); ++i) {
            arr_temp.push_back(edgeInfos[i].departureTimes);
            reachable_temp.push_back(edgeInfos[i].target);

            std::vector<int> cascaded;

            if (i == 0) {
                // First level: just the departure times plus sentinels
                cascaded.push_back(-1);
                cascaded.insert(cascaded.end(), edgeInfos[i].departureTimes.begin(), edgeInfos[i].departureTimes.end());
                cascaded.push_back(1000000000);
            } else {
                // Subsequent levels: merge with every other element from previous level
                cascaded = edgeInfos[i].departureTimes;

                // Add every other element from previous cascaded array
                for (size_t k = 1; k < m_arr_temp[i - 1].size(); k += 2) {
                    cascaded.push_back(m_arr_temp[i - 1][k]);
                }

                // Remove duplicates and sort
                std::sort(cascaded.begin(), cascaded.end());
                cascaded.erase(std::unique(cascaded.begin(), cascaded.end()), cascaded.end());

                // Add sentinels
                cascaded.insert(cascaded.begin(), -1);
                cascaded.push_back(1000000000);
            }

            m_arr_temp.push_back(std::move(cascaded));
        }

        // Reverse to match Python's order
        std::reverse(m_arr_temp.begin(), m_arr_temp.end());
        std::reverse(arr_temp.begin(), arr_temp.end());
        std::reverse(reachable_temp.begin(), reachable_temp.end());

        fc.m_arr = std::move(m_arr_temp);
        fc.arr = std::move(arr_temp);
        fc.reachableNodes = std::move(reachable_temp);

        // Build pointers
        buildPointersForVertex(fc);
    }

    // [FRACTIONAL CASCADING] Build pointer structures for a vertex
    inline void buildPointersForVertex(FractionalCascadingData& fc) noexcept {
        fc.pointers.resize(fc.m_arr.size());

        for (size_t i = 0; i < fc.m_arr.size(); ++i) {
            fc.pointers[i].resize(fc.m_arr[i].size());

            for (size_t j = 0; j < fc.m_arr[i].size(); ++j) {
                FCPointer ptr;

                // startIndex: position in original array arr[i]
                ptr.startIndex = std::lower_bound(fc.arr[i].begin(), fc.arr[i].end(), fc.m_arr[i][j])
                                 - fc.arr[i].begin();

                // nextLoc: position in next cascaded array m_arr[i+1]
                if (i == fc.m_arr.size() - 1) {
                    ptr.nextLoc = 0;
                } else {
                    ptr.nextLoc = std::lower_bound(fc.m_arr[i + 1].begin(), fc.m_arr[i + 1].end(), fc.m_arr[i][j])
                                  - fc.m_arr[i + 1].begin();
                }

                fc.pointers[i][j] = ptr;
            }
        }
    }

public:
    // [FRACTIONAL CASCADING] Get edge from u to v in O(1)
    inline Edge getEdge(Vertex u, Vertex v) const noexcept {
        auto it = edgeMap.find({u, v});
        if (it != edgeMap.end()) {
            return it->second;
        }
        return noEdge;  // Edge not found
    }

    // [FRACTIONAL CASCADING] Check if edge exists
    inline bool hasEdge(Vertex u, Vertex v) const noexcept {
        return edgeMap.find({u, v}) != edgeMap.end();
    }

    inline size_t numVertices() const noexcept {
        return graph.numVertices();
    }

    inline auto edgesFrom(const Vertex u) const noexcept {
        return graph.edgesFrom(u);
    }

    inline size_t numEdges() const noexcept {
        return graph.numEdges();
    }

    template<typename ATTRIBUTE>
    inline auto get(const ATTRIBUTE& attribute, const Edge edge) const noexcept {
        return graph.get(attribute, edge);
    }

    inline int getArrivalTime(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);

        int minArrivalTime = never;

        auto begin = allDiscreteTrips.begin() + h.firstTripIndex;
        auto end = begin + h.tripCount;

        auto it = std::lower_bound(begin, end, departureTime,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });

        if (it != end) {
            size_t localIdx = std::distance(begin, it);
            if (h.tripCount > 0) {
                minArrivalTime = allSuffixMinArrivals[h.firstSuffixIndex + localIdx];
            } else {
                minArrivalTime = it->arrivalTime;
            }
        }

        if (h.walkTime != never) {
            minArrivalTime = std::min(minArrivalTime, departureTime + h.walkTime);
        }
        return minArrivalTime;
    }

    inline int getWalkArrivalFrom(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);
        if (h.walkTime == never) return never;
        return departureTime + h.walkTime;
    }

    inline int getMinTransferTimeAt(const Vertex u) const noexcept {
        return (size_t(u) < minTransferTimeByVertex.size()) ? minTransferTimeByVertex[u] : 0;
    }

    inline Vertex addVertex() {
        return graph.addVertex();
    }

    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const EdgeTripsHandle& func) {
        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);
        handle.set(Function, func);

        // Add to edge map for O(1) lookup
        Edge edgeId = handle;
        edgeMap[{from, to}] = edgeId;

        return handle;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        graph.writeBinary(fileName);
        IO::serialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
        // TODO: Serialize FC data
    }

    inline void deserialize(const std::string& fileName) noexcept {
        graph.readBinary(fileName);
        IO::deserialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
        // TODO: Deserialize FC data
    }

    inline static TimeDependentGraphFC FromBinary(const std::string& fileName) noexcept {
        TimeDependentGraphFC tdGraph;
        tdGraph.deserialize(fileName);
        return tdGraph;
    }

    inline void printStatistics() const noexcept {
        std::cout << "=== TimeDependentGraphFC Statistics ===" << std::endl;
        std::cout << "Vertices: " << numVertices() << std::endl;
        std::cout << "Edges: " << numEdges() << std::endl;

        size_t totalFCVertices = 0;
        size_t totalFCLevels = 0;
        for (const auto& fc : fcData) {
            if (!fc.m_arr.empty()) {
                totalFCVertices++;
                totalFCLevels += fc.m_arr.size();
            }
        }

        std::cout << "Vertices with FC data: " << totalFCVertices << std::endl;
        if (totalFCVertices > 0) {
            std::cout << "Average FC levels per vertex: " << (double)totalFCLevels / totalFCVertices << std::endl;
        }
        std::cout << "==========================================" << std::endl;
    }
};