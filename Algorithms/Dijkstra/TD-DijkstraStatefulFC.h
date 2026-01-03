#pragma once

#include <vector>
#include <set>
#include <memory>
#include <deque>
#include <algorithm>
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/TimeDependentGraphFC.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "Profiler.h"

// Time-dependent Dijkstra with Fractional Cascading optimization
//
// OPTIMIZATION: Uses fractional cascading to avoid multiple binary searches
// when relaxing edges from the same vertex. Instead of O(k log n) searches
// for k outgoing edges, we do O(log n + k) using the cascaded structure.
//
// Based on Python implementation in time_table_nodes == 'fc' branch

template<typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStatefulFC {
public:
    using Graph = TimeDependentGraphFC;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    // --- NODE LABEL (simpler version for FC) ---
    struct NodeLabel : public ExternalKHeapElement {
        NodeLabel() : ExternalKHeapElement(), arrivalTime(intMax), timeStamp(-1),
                      parent(noVertex) {}

        inline void reset(int ts) {
            arrivalTime = intMax;
            timeStamp = ts;
            parent = noVertex;
        }

        inline bool hasSmallerKey(const NodeLabel* other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }

        int arrivalTime;
        int timeStamp;
        Vertex parent;
    };

public:
    TimeDependentDijkstraStatefulFC(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices())
        , nodeLabels(g.numVertices())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex)
        , fcSearchCount(0)
        , regularSearchCount(0) {
            if (chData) {
                initialTransfers = std::make_unique<CoreCHInitialTransfers>(*chData, FORWARD, numberOfStops);
            }
        }

    inline void clear() noexcept {
        profiler.startPhase(TDD::PHASE_CLEAR);
        Q.clear();
        timeStamp++;
        settleCount = 0;
        relaxCount = 0;
        fcSearchCount = 0;
        regularSearchCount = 0;
        timer.restart();
        targetVertex = noVertex;
        profiler.donePhase(TDD::PHASE_CLEAR);
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        NodeLabel& L = getNodeLabel(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.parent = noVertex;
            Q.update(&L);
            profiler.countMetric(TDD::METRIC_ENQUEUES);
        }
    }

    template<typename STOP = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const STOP& stop = NoOperation) noexcept {
        profiler.start();
        clear();
        targetVertex = target;

        profiler.startPhase(TDD::PHASE_INITIALIZATION);
        if (initialTransfers) {
            initialTransfers->run(source, target);
            for (const Vertex stop : initialTransfers->getForwardPOIs()) {
                const int arrivalTime = departureTime + initialTransfers->getForwardDistance(stop);
                addSource(stop, arrivalTime);
            }
            if (target != noVertex) {
                const int dist = initialTransfers->getDistance();
                if (dist != INFTY) {
                    addSource(target, departureTime + dist);
                }
            }
        } else {
            addSource(source, departureTime);
        }
        profiler.donePhase(TDD::PHASE_INITIALIZATION);

        runRelaxation(target, stop);
        profiler.done();
    }

    inline bool reachable(const Vertex v) const noexcept {
        if (v >= nodeLabels.size()) return false;
        const NodeLabel& a = nodeLabels[v];
        return (a.timeStamp == timeStamp && a.arrivalTime != never);
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        if (v >= nodeLabels.size()) return never;
        const NodeLabel& a = nodeLabels[v];
        if (a.timeStamp == timeStamp) return a.arrivalTime;
        return never;
    }

    inline int getSettleCount() const noexcept { return settleCount; }
    inline int getRelaxCount() const noexcept { return relaxCount; }
    inline double getElapsedMilliseconds() const noexcept { return timer.elapsedMilliseconds(); }

    struct PathEntry {
        Vertex vertex;
        int arrivalTime;
    };

    inline std::vector<PathEntry> getPath(const Vertex target) const noexcept {
        std::vector<PathEntry> path;
        if (!reachable(target)) return path;

        Vertex curVertex = target;

        while (curVertex != noVertex) {
            const NodeLabel& L = nodeLabels[curVertex];
            path.push_back({curVertex, L.arrivalTime});
            curVertex = L.parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    inline void printFCStatistics() const noexcept {
        std::cout << "FC searches: " << fcSearchCount << std::endl;
        std::cout << "Regular searches: " << regularSearchCount << std::endl;
        if (fcSearchCount + regularSearchCount > 0) {
            double fcPercent = 100.0 * fcSearchCount / (fcSearchCount + regularSearchCount);
            std::cout << "FC usage: " << fcPercent << "%" << std::endl;
        }
    }

private:
    inline NodeLabel& getNodeLabel(const Vertex v) noexcept {
        NodeLabel& L = nodeLabels[v];
        if (L.timeStamp != timeStamp) L.reset(timeStamp);
        return L;
    }

    template<typename STOP>
    inline void runRelaxation(const Vertex target, const STOP& stop) noexcept {
        profiler.startPhase(TDD::PHASE_MAIN_LOOP);

        while (!Q.empty()) {
            const NodeLabel* cur = Q.extractFront();

            const Vertex u = Vertex(cur - nodeLabels.data());
            const int t = cur->arrivalTime;

            settleCount++;
            profiler.countMetric(TDD::METRIC_SETTLES);

            int targetUpperBound = never;
            if constexpr (TARGET_PRUNING) {
                if (target != noVertex) {
                    const NodeLabel& targetLabel = nodeLabels[target];
                    if (targetLabel.timeStamp == timeStamp) {
                        targetUpperBound = targetLabel.arrivalTime;
                        if (t >= targetUpperBound) {
                            profiler.countMetric(TDD::METRIC_PRUNED_LABELS);
                            continue;
                        }
                    }
                }
            }

            if (stop()) break;

            // 1. CoreCH Backward (if enabled)
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxEdge(targetVertex, u, arrivalAtTarget);
                }
            }

            // 2. [FRACTIONAL CASCADING] Scan edges using FC if available
            if (graph.hasFCData(u)) {
                relaxEdgesWithFC(u, t);
            } else {
                relaxEdgesRegular(u, t);
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    // [FRACTIONAL CASCADING] Relax edges using the cascaded structure
    // Based on Python: time_table_nodes == 'fc' branch
    inline void relaxEdgesWithFC(const Vertex u, const int departureTime) noexcept {
        fcSearchCount++;

        const FractionalCascadingData& fc = graph.getFCData(u);

        if (fc.m_arr.empty()) return;

        // Get the first-level search position
        int loc = fc.bisectLeft(departureTime);

        if (loc >= (int)fc.m_arr[0].size()) {
            loc = fc.m_arr[0].size() - 1;
        }

        // Process first reachable node
        if (!fc.reachableNodes.empty() && loc < (int)fc.pointers[0].size()) {
            const FCPointer& ptr = fc.pointers[0][loc];
            const Vertex targetNode = fc.reachableNodes[0];

            relaxEdgeWithStartIndex(u, targetNode, departureTime, ptr.startIndex);

            int nextLoc = ptr.nextLoc;

            // Cascade through remaining levels
            for (size_t i = 1; i < fc.m_arr.size(); ++i) {
                if (nextLoc >= (int)fc.m_arr[i].size()) {
                    nextLoc = fc.m_arr[i].size() - 1;
                }

                // Check which pointer to use based on comparison
                if (nextLoc > 0 && departureTime <= fc.m_arr[i][nextLoc - 1]) {
                    nextLoc = nextLoc - 1;
                }

                if (nextLoc < (int)fc.pointers[i].size()) {
                    const FCPointer& ptr_i = fc.pointers[i][nextLoc];
                    const Vertex targetNode_i = fc.reachableNodes[i];

                    relaxEdgeWithStartIndex(u, targetNode_i, departureTime, ptr_i.startIndex);

                    nextLoc = ptr_i.nextLoc;
                }
            }
        }

        // Process walking-only nodes
        for (const Vertex walkNode : fc.walkingNodes) {
            // O(1) edge lookup
            const Edge e = graph.getEdge(u, walkNode);
            if (e != noEdge) {
                const int arrivalAtV = graph.getWalkArrivalFrom(e, departureTime);
                if (arrivalAtV < never) {
                    relaxEdge(walkNode, u, arrivalAtV);
                }
            }
        }
    }

    // Relax edges the standard way (when no FC data available)
    inline void relaxEdgesRegular(const Vertex u, const int departureTime) noexcept {
        regularSearchCount++;

        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const int arrivalAtV = graph.getArrivalTime(e, departureTime);

            if (arrivalAtV < never) {
                relaxEdge(v, u, arrivalAtV);
            }
        }
    }

    // Relax a specific edge using the pre-computed start index from FC
    // NO binary search - the FC structure has already done the work!
    inline void relaxEdgeWithStartIndex(const Vertex u, const Vertex v, const int departureTime, const int startIndex) noexcept {
        // O(1) edge lookup using the edge map
        const Edge e = graph.getEdge(u, v);
        if (e == noEdge) return;

        const EdgeTripsHandle& h = graph.get(Function, e);

        int bestArrival = never;

        // Python: if start_index < f.size: l = f.buses[start_index].a
        // Use startIndex DIRECTLY - no search needed!
        if (startIndex >= 0 && (uint32_t)startIndex < h.tripCount) {
            // Get the trip at this index
            const DiscreteTrip* trips = graph.getTripsBegin(h);
            const DiscreteTrip& trip = trips[startIndex];

            // Verify this trip is valid (departs at or after departureTime)
            if (trip.departureTime >= departureTime) {
                // Use suffix minima for best arrival from this index onward
                const int* suffixMin = graph.getSuffixMinBegin(h);
                bestArrival = suffixMin[startIndex];
            }
        }

        // Check walking option
        if (h.walkTime != never) {
            int walkArrival = departureTime + h.walkTime;
            bestArrival = std::min(bestArrival, walkArrival);
        }

        if (bestArrival < never) {
            relaxEdge(v, u, bestArrival);
        }
    }

    inline void relaxEdge(const Vertex v, const Vertex parent, const int newTime) noexcept {
        relaxCount++;
        profiler.countMetric(TDD::METRIC_RELAXES_WALKING);

        NodeLabel& L = getNodeLabel(v);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.parent = parent;
            Q.update(&L);
            profiler.countMetric(TDD::METRIC_ENQUEUES);
        }
    }

public:
    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    const Graph& graph;
    const size_t numberOfStops;
    ExternalKHeap<2, NodeLabel> Q;

    std::vector<NodeLabel> nodeLabels;

    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
    Profiler profiler;

    // FC statistics
    size_t fcSearchCount;
    size_t regularSearchCount;
};