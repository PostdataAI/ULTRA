#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

#include "CH.h"
#include "CHUtils.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"

namespace CH {

// Extracts minimal (pruned) hub labels from a Contraction Hierarchy.
//
// Uses the standard pruned labeling algorithm: vertices are processed from
// highest to lowest CH rank. For each vertex v, an upward Dijkstra is run
// in the CH, but a hub h is only added to v's label if the existing labels
// cannot yet cover the v-to-h shortest distance (2-hop pruning).
//
// Forward labels use ch.forward, backward labels use ch.backward.
// Both directions are built together per vertex to satisfy the pruning
// dependency (forward pruning needs backward labels, and vice versa).
//
// Requires a full CH (no core vertices). A Core CH will produce incorrect labels.
class HubLabelExtractor {

    struct HubEntry {
        Vertex hub;
        int distance;
    };

    using Label = std::vector<HubEntry>;

public:
    HubLabelExtractor(const CH& ch) :
        ch(ch),
        numVertices(ch.numVertices()),
        forwardLabels(ch.numVertices()),
        backwardLabels(ch.numVertices()) {
    }

    void run() {
        Timer timer;

        std::cout << "Computing processing order..." << std::endl;
        timer.restart();
        const std::vector<Vertex> order = computeProcessingOrder();
        std::cout << "  Done in " << String::msToString(timer.elapsedMilliseconds()) << std::endl;

        std::cout << "Extracting pruned hub labels..." << std::endl;
        timer.restart();
        extractPrunedLabels(order);
        std::cout << "  Done in " << String::msToString(timer.elapsedMilliseconds()) << std::endl;

        std::cout << "Building TransferGraphs..." << std::endl;
        timer.restart();
        buildGraph(forwardLabels, outHubs);
        buildGraph(backwardLabels, inHubs);
        std::cout << "  Done in " << String::msToString(timer.elapsedMilliseconds()) << std::endl;

        printStatistics();
    }

    TransferGraph& getOutHubs() noexcept { return outHubs; }
    TransferGraph& getInHubs() noexcept { return inHubs; }
    const TransferGraph& getOutHubs() const noexcept { return outHubs; }
    const TransferGraph& getInHubs() const noexcept { return inHubs; }

private:
    // Returns vertices in processing order: highest CH rank first.
    // CH::getOrder() returns contraction order (lowest rank first), so we reverse.
    std::vector<Vertex> computeProcessingOrder() const {
        std::vector<Vertex> order = getOrder(ch);
        std::reverse(order.begin(), order.end());
        return order;
    }

    void extractPrunedLabels(const std::vector<Vertex> &order) {
        size_t totalForward = 0;
        size_t totalBackward = 0;

        Progress progress(numVertices);
        for (const Vertex v : order) {
            // Self-entry: every vertex is its own hub at distance 0
            forwardLabels[v].push_back({v, 0});
            backwardLabels[v].push_back({v, 0});

            // Build forward label (upward search in ch.forward)
            prunedUpwardSearch(v, ch.forward, forwardLabels, backwardLabels);

            // Build backward label (upward search in ch.backward)
            prunedUpwardSearch(v, ch.backward, backwardLabels, forwardLabels);

            // Sort both labels by hub vertex ID for efficient merge-queries
            sortLabel(forwardLabels[v]);
            sortLabel(backwardLabels[v]);

            totalForward += forwardLabels[v].size();
            totalBackward += backwardLabels[v].size();

            progress++;
        }

        std::cout << "  Forward labels: " << String::prettyInt(totalForward)
                  << " entries (avg " << String::prettyDouble(static_cast<double>(totalForward) / numVertices, 1) << ")" << std::endl;
        std::cout << "  Backward labels: " << String::prettyInt(totalBackward)
                  << " entries (avg " << String::prettyDouble(static_cast<double>(totalBackward) / numVertices, 1) << ")" << std::endl;
    }

    // Runs a pruned upward Dijkstra from vertex v in the given CH graph.
    // myLabels[v] is being built (has self-entry already).
    // otherLabels contains the completed labels in the OTHER direction
    // for all higher-ranked vertices (used for the pruning check).
    void prunedUpwardSearch(const Vertex v, const CHGraph& graph,
                            std::vector<Label>& myLabels,
                            const std::vector<Label>& otherLabels) {
        using QueueEntry = std::pair<int, Vertex>;
        // Thread-unsafe but single-threaded: reuse across calls
        static std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pq;
        static std::vector<int> distance;
        static std::vector<Vertex> touched;

        if (distance.size() < numVertices) {
            distance.assign(numVertices, INFTY);
            touched.reserve(512);
        }

        // Seed with upward neighbors of v
        for (const Edge edge : graph.edgesFrom(v)) {
            const Vertex w = graph.get(ToVertex, edge);
            const int d = graph.get(Weight, edge);
            if (d < distance[w]) {
                if (distance[w] == INFTY) touched.push_back(w);
                distance[w] = d;
                pq.push({d, w});
            }
        }

        while (!pq.empty()) {
            const auto [dist, u] = pq.top();
            pq.pop();

            if (dist > distance[u]) continue;

            // Pruning check: is v->u already covered by existing labels?
            if (twoHopDistance(myLabels[v], otherLabels[u]) <= dist) {
                // This hub (and its upward neighborhood) is redundant
                continue;
            }

            // Hub u is necessary -- add to v's label
            myLabels[v].push_back({u, dist});

            // Relax upward edges
            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex w = graph.get(ToVertex, edge);
                const int newDist = dist + graph.get(Weight, edge);
                if (newDist < distance[w]) {
                    if (distance[w] == INFTY) touched.push_back(w);
                    distance[w] = newDist;
                    pq.push({newDist, w});
                }
            }
        }

        // Reset touched vertices
        for (const Vertex u : touched) {
            distance[u] = INFTY;
        }
        touched.clear();
    }

    // Computes the 2-hop distance between the owners of labelA and labelB.
    // labelA is being built (unsorted, small), labelB is already sorted by hub ID.
    static int twoHopDistance(const Label& labelA, const Label& labelB) {
        int result = INFTY;
        for (const auto& [h, dh] : labelA) {
            // Binary search for h in sorted labelB
            auto it = std::lower_bound(labelB.begin(), labelB.end(), h,
                [](const HubEntry& e, Vertex hub) { return e.hub < hub; });
            if (it != labelB.end() && it->hub == h) {
                const int candidate = dh + it->distance;
                if (candidate < result) {
                    result = candidate;
                    if (result == 0) return 0;
                }
            }
        }
        return result;
    }

    static void sortLabel(Label& label) {
        std::sort(label.begin(), label.end(),
            [](const HubEntry& a, const HubEntry& b) { return a.hub < b.hub; });
    }

    void buildGraph(const std::vector<Label>& labels, TransferGraph& result) const {
        DynamicTransferGraph dynamicGraph;
        dynamicGraph.addVertices(numVertices);

        for (size_t v = 0; v < numVertices; v++) {
            for (const auto& [hub, dist] : labels[v]) {
                if (Vertex(v) == hub) continue; // Skip self-entries for TransferGraph
                dynamicGraph.addEdge(Vertex(v), hub).set(TravelTime, dist);
            }
        }

        Graph::move(std::move(dynamicGraph), result);
    }

    void printStatistics() const {
        std::cout << "\nHub label statistics:" << std::endl;
        std::cout << "  Vertices:  " << String::prettyInt(numVertices) << std::endl;
        std::cout << "  Out-hubs:  " << String::prettyInt(outHubs.numEdges()) << " edges" << std::endl;
        std::cout << "  In-hubs:   " << String::prettyInt(inHubs.numEdges()) << " edges" << std::endl;

        // Count unique hubs
        std::vector<bool> isHub(numVertices, false);
        for (const Vertex v : outHubs.vertices()) {
            for (const Edge edge : outHubs.edgesFrom(v)) {
                isHub[outHubs.get(ToVertex, edge)] = true;
            }
        }
        for (const Vertex v : inHubs.vertices()) {
            for (const Edge edge : inHubs.edgesFrom(v)) {
                isHub[inHubs.get(ToVertex, edge)] = true;
            }
        }
        size_t hubCount = 0;
        for (size_t i = 0; i < numVertices; i++) {
            if (isHub[i]) hubCount++;
        }
        std::cout << "  Unique hubs: " << String::prettyInt(hubCount) << std::endl;
    }

    const CH& ch;
    const size_t numVertices;
    std::vector<Label> forwardLabels;
    std::vector<Label> backwardLabels;
    TransferGraph outHubs;
    TransferGraph inHubs;
};

}
