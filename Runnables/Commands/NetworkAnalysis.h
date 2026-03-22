#pragma once

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Geometry/Point.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class AnalyzeShortcutDistances : public ParameterizedCommand {

public:
    AnalyzeShortcutDistances(BasicShell& shell) :
        ParameterizedCommand(shell, "analyzeShortcutDistances",
            "Analyzes walking time and geographic distance distribution of stop-to-stop shortcuts.") {
        addParameter("Shortcut RAPTOR data file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data data(getParameter("Shortcut RAPTOR data file"));
        data.printInfo();

        const size_t numStops = data.numberOfStops();
        std::vector<int> walkingTimes;
        std::vector<double> geoDistancesKm;
        std::vector<size_t> outDegree(numStops, 0);

        for (const Vertex from : data.transferGraph.vertices()) {
            if (static_cast<size_t>(from) >= numStops) continue;
            for (const Edge edge : data.transferGraph.edgesFrom(from)) {
                const Vertex to = data.transferGraph.get(ToVertex, edge);
                if (static_cast<size_t>(to) >= numStops) continue;
                const int travelTime = data.transferGraph.get(TravelTime, edge);
                walkingTimes.emplace_back(travelTime);
                outDegree[from]++;

                const Geometry::Point& fromCoord = data.transferGraph.get(Coordinates, from);
                const Geometry::Point& toCoord = data.transferGraph.get(Coordinates, to);
                geoDistancesKm.emplace_back(Geometry::geoDistanceInCM(fromCoord, toCoord) / 100000.0);
            }
        }

        const size_t totalShortcuts = walkingTimes.size();
        std::cout << "\n=== Shortcut Distance Analysis ===" << std::endl;
        std::cout << "Total shortcuts:  " << String::prettyInt(totalShortcuts) << std::endl;
        std::cout << "Total stops:      " << String::prettyInt(numStops) << std::endl;

        if (totalShortcuts == 0) {
            std::cout << "No shortcuts found." << std::endl;
            return;
        }

        // Walking time distribution
        std::cout << "\n=== Walking Time Distribution ===" << std::endl;
        const int buckets[] = {0, 300, 600, 900, 1800, 3600, 7200, 14400, 36000, 86400, INT_MAX};
        const char* labels[] = {"0-5min", "5-10min", "10-15min", "15-30min", "30-60min", "1-2hr", "2-4hr", "4-10hr", "10-24hr", "24hr+"};
        for (int i = 0; buckets[i] != INT_MAX; i++) {
            size_t count = 0;
            for (const int t : walkingTimes) {
                if (t >= buckets[i] && t < buckets[i + 1]) count++;
            }
            if (count > 0) {
                std::cout << "  " << labels[i] << ":  " << String::prettyInt(count)
                          << "  (" << String::percent(static_cast<double>(count) / totalShortcuts) << ")" << std::endl;
            }
        }

        // Walking time percentiles
        std::sort(walkingTimes.begin(), walkingTimes.end());
        std::cout << "\n=== Walking Time Percentiles ===" << std::endl;
        std::cout << "  Min:    " << String::secToString(walkingTimes.front()) << std::endl;
        std::cout << "  P25:    " << String::secToString(walkingTimes[totalShortcuts / 4]) << std::endl;
        std::cout << "  Median: " << String::secToString(walkingTimes[totalShortcuts / 2]) << std::endl;
        std::cout << "  P75:    " << String::secToString(walkingTimes[3 * totalShortcuts / 4]) << std::endl;
        std::cout << "  P90:    " << String::secToString(walkingTimes[9 * totalShortcuts / 10]) << std::endl;
        std::cout << "  P99:    " << String::secToString(walkingTimes[99 * totalShortcuts / 100]) << std::endl;
        std::cout << "  Max:    " << String::secToString(walkingTimes.back()) << std::endl;

        // Geographic distance percentiles
        std::sort(geoDistancesKm.begin(), geoDistancesKm.end());
        std::cout << "\n=== Geographic Distance (km) ===" << std::endl;
        std::cout << "  Min:    " << geoDistancesKm.front() << " km" << std::endl;
        std::cout << "  P25:    " << geoDistancesKm[totalShortcuts / 4] << " km" << std::endl;
        std::cout << "  Median: " << geoDistancesKm[totalShortcuts / 2] << " km" << std::endl;
        std::cout << "  P75:    " << geoDistancesKm[3 * totalShortcuts / 4] << " km" << std::endl;
        std::cout << "  P90:    " << geoDistancesKm[9 * totalShortcuts / 10] << " km" << std::endl;
        std::cout << "  P99:    " << geoDistancesKm[99 * totalShortcuts / 100] << " km" << std::endl;
        std::cout << "  Max:    " << geoDistancesKm.back() << " km" << std::endl;

        // Out-degree distribution
        std::sort(outDegree.begin(), outDegree.end());
        size_t zeroCount = 0, low = 0, mid = 0, high = 0, vhigh = 0, extreme = 0;
        for (const size_t d : outDegree) {
            if (d == 0) zeroCount++;
            else if (d <= 10) low++;
            else if (d <= 50) mid++;
            else if (d <= 200) high++;
            else if (d <= 1000) vhigh++;
            else extreme++;
        }
        std::cout << "\n=== Out-degree Distribution (shortcuts per origin stop) ===" << std::endl;
        std::cout << "  0:         " << String::prettyInt(zeroCount) << " stops" << std::endl;
        std::cout << "  1-10:      " << String::prettyInt(low) << " stops" << std::endl;
        std::cout << "  11-50:     " << String::prettyInt(mid) << " stops" << std::endl;
        std::cout << "  51-200:    " << String::prettyInt(high) << " stops" << std::endl;
        std::cout << "  201-1000:  " << String::prettyInt(vhigh) << " stops" << std::endl;
        std::cout << "  1000+:     " << String::prettyInt(extreme) << " stops" << std::endl;
        std::cout << "  Max degree:" << outDegree.back() << std::endl;
    }
};

class PrintRaptorStatistics : public ParameterizedCommand {

public:
    PrintRaptorStatistics(BasicShell& shell) :
        ParameterizedCommand(shell, "printRaptorStatistics",
            "Prints detailed route structure statistics for a RAPTOR network.") {
        addParameter("RAPTOR data file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data data(getParameter("RAPTOR data file"));
        data.printInfo();

        const size_t numRoutes = data.numberOfRoutes();
        const size_t numStops = data.numberOfStops();

        std::vector<size_t> stopsPerRoute(numRoutes);
        std::vector<size_t> tripsPerRoute(numRoutes);
        for (const RouteId route : data.routes()) {
            stopsPerRoute[route] = data.numberOfStopsInRoute(route);
            tripsPerRoute[route] = data.numberOfTripsInRoute(route);
        }

        std::vector<size_t> routesPerStop(numStops, 0);
        for (const StopId stop : data.stops()) {
            routesPerStop[stop] = data.routesContainingStop(stop).size();
        }

        auto printStats = [](const std::string& name, std::vector<size_t>& v) {
            std::sort(v.begin(), v.end());
            double sum = std::accumulate(v.begin(), v.end(), 0.0);
            std::cout << "  " << name << ": min=" << v.front()
                      << " max=" << v.back()
                      << " avg=" << (sum / v.size())
                      << " median=" << v[v.size() / 2]
                      << " p90=" << v[9 * v.size() / 10]
                      << " p99=" << v[99 * v.size() / 100] << std::endl;
        };

        std::cout << "\n=== Route Structure Statistics ===" << std::endl;
        printStats("Stops/route", stopsPerRoute);
        printStats("Trips/route", tripsPerRoute);
        printStats("Routes/stop", routesPerStop);

        // Count unique ordered stop pairs across all routes
        size_t totalPairs = 0;
        for (const RouteId route : data.routes()) {
            size_t n = data.numberOfStopsInRoute(route);
            totalPairs += n * (n - 1) / 2;
        }
        std::cout << "  Ordered stop pairs in routes: " << String::prettyInt(totalPairs) << std::endl;

        // Histogram: stops per route
        std::cout << "\n=== Histogram: Stops per Route ===" << std::endl;
        size_t bins[] = {1, 5, 10, 20, 50, 100, 200, 10000};
        const char* binLabels[] = {"1-4", "5-9", "10-19", "20-49", "50-99", "100-199", "200+"};
        for (int i = 0; bins[i] != 10000; i++) {
            size_t count = 0;
            for (const size_t s : stopsPerRoute) {
                if (s >= bins[i] && s < bins[i + 1]) count++;
            }
            if (count > 0) std::cout << "  " << binLabels[i] << ": " << String::prettyInt(count) << " routes" << std::endl;
        }

        // Histogram: trips per route
        std::cout << "\n=== Histogram: Trips per Route ===" << std::endl;
        size_t tbins[] = {1, 2, 5, 10, 20, 50, 100, 10000};
        const char* tLabels[] = {"1", "2-4", "5-9", "10-19", "20-49", "50-99", "100+"};
        for (int i = 0; tbins[i] != 10000; i++) {
            size_t count = 0;
            for (const size_t t : tripsPerRoute) {
                if (t >= tbins[i] && t < tbins[i + 1]) count++;
            }
            if (count > 0) std::cout << "  " << tLabels[i] << ": " << String::prettyInt(count) << " routes" << std::endl;
        }
    }
};

class DumpStopNames : public ParameterizedCommand {

public:
    DumpStopNames(BasicShell& shell) :
        ParameterizedCommand(shell, "dumpStopNames",
            "Dumps stop names from a RAPTOR binary, with optional grep pattern.") {
        addParameter("RAPTOR data file");
        addParameter("Pattern", "");
    }

    virtual void execute() noexcept {
        RAPTOR::Data data(getParameter("RAPTOR data file"));
        const std::string pattern = getParameter("Pattern");

        size_t platformCount = 0;
        size_t printed = 0;
        for (const StopId stop : data.stops()) {
            const std::string& name = data.stopData[stop].name;
            bool hasplatform = (name.find("Platform") != std::string::npos
                             || name.find("platform") != std::string::npos
                             || name.find("Platform") != std::string::npos
                             || name.find("Gleis") != std::string::npos
                             || name.find("Perron") != std::string::npos
                             || name.find("Quai") != std::string::npos
                             || name.find("Stand") != std::string::npos);
            if (hasplatform) platformCount++;
            if (!pattern.empty() && name.find(pattern) == std::string::npos) continue;
            if (printed < 200 || !pattern.empty()) {
                std::cout << static_cast<size_t>(stop) << "\t" << name << std::endl;
                printed++;
            }
        }
        std::cout << "\nTotal stops: " << data.numberOfStops() << std::endl;
        std::cout << "Stops with platform/Gleis/Perron/Quai/Stand in name: " << platformCount << std::endl;
    }
};
