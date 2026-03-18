#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <random>

#include "../../Helpers/MultiThreading.h"

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/HubLabelExtractor.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Shell/Shell.h"
using namespace Shell;

inline constexpr int ShortcutWeight = 1024;
inline constexpr int DegreeWeight = 0;
inline constexpr int UnidirectionalPopLimit = 500;
inline constexpr int BidirectionalPopLimit = 200;

template<typename PROFILER>
using UnidirectionalWitnessSearch = CH::WitnessSearch<CHCoreGraph, PROFILER, UnidirectionalPopLimit>;
template<typename PROFILER>
using BidirectionalWitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, PROFILER, BidirectionalPopLimit>;

template<typename WITNESS_SEARCH>
using GreedyKey = CH::GreedyKey<WITNESS_SEARCH>;
template<typename WITNESS_SEARCH>
using PartialKey = CH::PartialKey<WITNESS_SEARCH, GreedyKey<WITNESS_SEARCH>>;
using StopCriterion = CH::NoStopCriterion;

template<typename CH_BUILDER>
inline CH::CH finalizeCH(CH_BUILDER&& chBuilder, const std::string& orderOutputFile, const std::string& chOutputFile) noexcept {
    chBuilder.copyCoreToCH();
    Order order;
    for (const Vertex vertex : chBuilder.getOrder()) {
        order.emplace_back(vertex);
    }
    order.serialize(orderOutputFile);
    std::cout << "Obtaining CH" << std::endl;
    CH::CH ch(std::move(chBuilder));
    ch.writeBinary(chOutputFile);
    std::cout << std::endl;
    return ch;
}

template<typename PROFILER, typename WITNESS_SEARCH, typename GRAPH, typename KEY_FUNCTION, typename STOP_CRITERION = StopCriterion>
inline CH::CH buildCH(GRAPH& originalGraph, const std::string& orderOutputFile, const std::string& chOutputFile, const KEY_FUNCTION& keyFunction, const STOP_CRITERION& stopCriterion = StopCriterion()) noexcept {
    TravelTimeGraph graph;
    Graph::copy(originalGraph, graph);
    Graph::printInfo(graph);
    CH::Builder<PROFILER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), graph[TravelTime], keyFunction, stopCriterion);
    chBuilder.run();
    return finalizeCH(chBuilder, orderOutputFile, chOutputFile);
}

class BuildCH : public ParameterizedCommand {

public:
    BuildCH(BasicShell& shell) :
        ParameterizedCommand(shell, "buildCH", "Computes a CH with greedy key for the input graph.") {
        addParameter("Graph binary");
        addParameter("Order output file");
        addParameter("CH output file");
        addParameter("Use full profiler?", "true");
        addParameter("Witness search type", "bidirectional", { "normal", "bidirectional" });
        addParameter("Level weight", "256");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use full profiler?")) {
            chooseWitnessSearch<CH::FullProfiler>();
        } else {
            chooseWitnessSearch<CH::TimeProfiler>();
        }
    }

private:
    template<typename PROFILER>
    inline void chooseWitnessSearch() const noexcept {
        if (getParameter("Witness search type") == "normal") {
            build<PROFILER, UnidirectionalWitnessSearch<PROFILER>>();
        } else {
            build<PROFILER, BidirectionalWitnessSearch<PROFILER>>();
        }
    }

    template<typename PROFILER, typename WITNESS_SEARCH>
    inline void build() const noexcept {
        TransferGraph graph(getParameter("Graph binary"));
        GreedyKey<WITNESS_SEARCH> keyFunction(ShortcutWeight, getParameter<int>("Level weight"), DegreeWeight);
        buildCH<PROFILER, WITNESS_SEARCH>(graph, getParameter("Order output file"), getParameter("CH output file"), keyFunction);
    }
};

class BuildCoreCH : public ParameterizedCommand {

public:
    BuildCoreCH(BasicShell& shell) :
        ParameterizedCommand(shell, "buildCoreCH", "Computes a core-CH for the input network, where all stops are kept uncontracted.") {
        addParameter("Network input file");
        addParameter("Order output file");
        addParameter("CH output file");
        addParameter("Network output file");
        addParameter("Max core degree", "14");
        addParameter("Network type", "raptor", {"intermediate", "csa", "raptor"});
        addParameter("Use full profiler?", "true");
        addParameter("Witness search type", "bidirectional", { "normal", "bidirectional" });
        addParameter("Level weight", "256");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use full profiler?")) {
            return chooseWitnessSearch<CH::FullProfiler>();
        } else {
            return chooseWitnessSearch<CH::TimeProfiler>();
        }
    }

private:
    template<typename PROFILER>
    inline void chooseWitnessSearch() noexcept {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            chooseNetworkType<PROFILER, UnidirectionalWitnessSearch<PROFILER>>();
        } else {
            chooseNetworkType<PROFILER, BidirectionalWitnessSearch<PROFILER>>();
        }
    }

    template<typename PROFILER, typename WITNESS_SEARCH>
    inline void chooseNetworkType() noexcept {
        const std::string networkType = getParameter("Network type");
        if (networkType == "raptor") {
            build<RAPTOR::Data, PROFILER, WITNESS_SEARCH>();
        } else if (networkType == "csa") {
            build<CSA::Data, PROFILER, WITNESS_SEARCH>();
        } else {
            build<Intermediate::Data, PROFILER, WITNESS_SEARCH>();
        }
    }

    template<typename NETWORK_TYPE, typename PROFILER, typename WITNESS_SEARCH>
    inline void build() const noexcept {
        NETWORK_TYPE data(getParameter("Network input file"));
        data.printInfo();

        std::vector<bool> contractable(data.numberOfStops(), false);
        contractable.resize(data.transferGraph.numVertices(), true);

        const double maxCoreDegree = getParameter<double>("Max core degree");
        std::cout << "Min. core size: " << String::prettyInt(data.numberOfStops()) << std::endl;
        std::cout << "Max. core degree: " << String::prettyInt(maxCoreDegree) << std::endl;
        GreedyKey<WITNESS_SEARCH> greedyKey(ShortcutWeight, getParameter<int>("Level weight"), DegreeWeight);
        PartialKey<WITNESS_SEARCH> keyFunction(contractable, data.transferGraph.numVertices(), greedyKey);
        CH::CoreCriterion stopCriterion(data.numberOfStops(), maxCoreDegree);
        const CH::CH ch = buildCH<PROFILER, WITNESS_SEARCH>(data.transferGraph, getParameter("Order output file"), getParameter("CH output file"), keyFunction, stopCriterion);

        Intermediate::TransferGraph resultGraph;
        resultGraph.addVertices(data.transferGraph.numVertices());
        resultGraph[Coordinates] = data.transferGraph[Coordinates];
        for (const Vertex vertex : resultGraph.vertices()) {
            if (ch.isCoreVertex(vertex)) {
                for (const Edge edge : ch.forward.edgesFrom(vertex)) {
                    resultGraph.addEdge(vertex, ch.forward.get(ToVertex, edge)).set(TravelTime, ch.forward.get(Weight, edge));
                }
            }
        }
        Graph::move(std::move(resultGraph), data.transferGraph);
        data.serialize(getParameter("Network output file"));
    }
};

class ExtractHubLabels : public ParameterizedCommand {

public:
    ExtractHubLabels(BasicShell& shell) :
        ParameterizedCommand(shell, "extractHubLabels", "Extracts hub labels from a full CH for use with HLRAPTOR/HLCSA.") {
        addParameter("CH input file");
        addParameter("Out-hub output file");
        addParameter("In-hub output file");
    }

    virtual void execute() noexcept {
        Timer totalTimer;

        std::cout << "Loading CH..." << std::endl;
        CH::CH ch(getParameter("CH input file"));
        std::cout << "CH: " << String::prettyInt(ch.numVertices()) << " vertices, "
                  << String::prettyInt(ch.numEdges()) << " edges" << std::endl;

        CH::HubLabelExtractor extractor(ch);
        extractor.run();

        std::cout << "\nWriting out-hub labels..." << std::endl;
        extractor.getOutHubs().writeBinary(getParameter("Out-hub output file"));
        std::cout << "Writing in-hub labels..." << std::endl;
        extractor.getInHubs().writeBinary(getParameter("In-hub output file"));
        std::cout << "\nTotal preprocessing time (including I/O): " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
    }
};

class ImportHubLabels : public ParameterizedCommand {

public:
    ImportHubLabels(BasicShell& shell) :
        ParameterizedCommand(shell, "importHubLabels",
            "Imports hub labels from lviennot/hub-labeling text format into TransferGraph binary files.") {
        addParameter("Hub label text file");
        addParameter("Number of vertices");
        addParameter("Out-hub output file");
        addParameter("In-hub output file");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Hub label text file");
        const size_t numVertices = getParameter<size_t>("Number of vertices");

        std::ifstream is(inputFile);
        Assert(is.is_open(), "Cannot open hub label file: " << inputFile);

        DynamicTransferGraph outGraph, inGraph;
        outGraph.addVertices(numVertices);
        inGraph.addVertices(numVertices);

        size_t outCount = 0, inCount = 0;
        std::string line;
        while (std::getline(is, line)) {
            if (line.empty()) continue;
            char type = line[0];
            if (type != 'o' && type != 'i') continue;

            std::istringstream ss(line.substr(2));
            size_t a, b;
            int64_t dist;
            if (!(ss >> a >> b >> dist)) continue;

            if (type == 'o') {
                // o vertex hub distance → outHubs: vertex → hub
                outGraph.addEdge(Vertex(a), Vertex(b)).set(TravelTime, static_cast<int>(dist));
                outCount++;
            } else {
                // i hub vertex distance → inHubs: vertex → hub
                inGraph.addEdge(Vertex(b), Vertex(a)).set(TravelTime, static_cast<int>(dist));
                inCount++;
            }
        }
        is.close();

        std::cout << "Parsed " << outCount << " out-hub edges, "
                  << inCount << " in-hub edges" << std::endl;

        TransferGraph outHubs, inHubs;
        Graph::move(std::move(outGraph), outHubs);
        Graph::move(std::move(inGraph), inHubs);

        double avgOut = static_cast<double>(outCount) / numVertices;
        double avgIn = static_cast<double>(inCount) / numVertices;
        std::cout << "Out-hubs: " << outHubs.numVertices() << " vertices, "
                  << outHubs.numEdges() << " edges (avg " << avgOut << ")" << std::endl;
        std::cout << "In-hubs:  " << inHubs.numVertices() << " vertices, "
                  << inHubs.numEdges() << " edges (avg " << avgIn << ")" << std::endl;

        outHubs.writeBinary(getParameter("Out-hub output file"));
        inHubs.writeBinary(getParameter("In-hub output file"));
        std::cout << "Hub labels written." << std::endl;
    }
};

