#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

class RunTransitiveCSAQueries : public ParameterizedCommand {

public:
    RunTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveCSAQueries", "Runs the given number of random transitive CSA queries.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Target pruning?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        const bool targetPruning = getParameter<bool>("Target pruning?");

        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunDijkstraCSAQueries : public ParameterizedCommand {

public:
    RunDijkstraCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraCSAQueries", "Runs the given number of random Dijkstra-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));
        CSA::DijkstraCSA<RAPTOR::CoreCHInitialTransfers, true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunHLCSAQueries : public ParameterizedCommand {

public:
    RunHLCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLCSAQueries", "Runs the given number of random HL-CSA queries.") {
        addParameter("CSA input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        CSA::HLCSA<CSA::AggregateProfiler> algorithm(csaData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunULTRACSAQueries : public ParameterizedCommand {

public:
    RunULTRACSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRACSAQueries", "Runs the given number of random ULTRA-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));
        CSA::ULTRACSA<true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveRAPTORQueries", "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        raptorData.writeCSV("customnetwork");
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;

    }
};

class RunDijkstraRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueries", "Runs the given number of random Dijkstra RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunHLRAPTORQueries : public ParameterizedCommand {

public:
    RunHLRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLRAPTORQueries", "Runs the given number of random HL-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        RAPTOR::HLRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveTBQueries : public ParameterizedCommand {

public:
    RunTransitiveTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveTBQueries", "Runs the given number of random transitive TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(tripBasedData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(tripBasedData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRATBQueries : public ParameterizedCommand {

public:
    RunULTRATBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRATBQueries", "Runs the given number of random ULTRA-TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::Query<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};


class CheckCSAPruning : public ParameterizedCommand {

public:
    CheckCSAPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkCSAPruning", "Checks if pruning rules yield the same results as no pruning for CSA.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Precompute directed transitive graph?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        // RETRIEVE THE NEW PARAMETER'S VALUE
        const bool precomputeTransitive = getParameter<bool>("Precompute directed transitive graph?");

        // CONDITIONAL PRECOMPUTATION
        if (precomputeTransitive) {
            std::cout << "--- Precomputing directed transitive stop graph... ---" << std::endl;
            csaData.makeDirectedTransitiveStopGraph(true); // 'true' enables verbose output
            std::cout << "--- Done. ---" << std::endl;
        }

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;

        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running queries with No Pruning (Rule 0) ---" << std::endl;
        CSA::CSA<false, CSA::AggregateProfiler> algo_no_pruning(csaData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running queries with Pruning Rule 1 ---" << std::endl;
        csaData.sortTransferGraphEdgesByTravelTime();
        CSA::CSA_prune<false, CSA::AggregateProfiler> algo_pruning_1(csaData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_is_correct = (results_no_pruning == results_pruning_1);

        if (pruning_is_correct) {
            std::cout << "\nPruning rule 1 yields the same results as no pruning." << std::endl;
        } else {
            std::cout << "\nPruning rule 1 failed comparison." << std::endl;
        }
    }
};

class CheckRAPTORPruning : public ParameterizedCommand {

public:
    CheckRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkRAPTORPruning", "Checks if RAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        const size_t n = getParameter<size_t>("Number of queries");
        // Generate StopQueries, not VertexQueries
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;

        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running with No Pruning (Rule 0) ---" << std::endl;
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo_no_pruning(raptorData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running with Pruning Rule 1 ---" << std::endl;
        // The transfer graph must be sorted for pruning rule 1 to be effective
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algo_pruning_1(raptorData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_1_correct = (results_no_pruning == results_pruning_1);
        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_1_correct) {
            std::cout << "Pruning rule 1 results match no-pruning results. The pruning is correct." << std::endl;
        } else {
            std::cout << "ERROR: Pruning rule 1 failed comparison. Results are not identical." << std::endl;
        }
    }
};


