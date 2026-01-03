#pragma once

// Include TimeDependentGraph.h first to get the DiscreteTrip and EdgeTripsHandle definitions
#include "../../DataStructures/Graph/TimeDependentGraph.h"
// Then include the Classic variant which uses the same structs
#include "../../DataStructures/Graph/TimeDependentGraphClassic.h"
#include "TD-DijkstraStateful.h"

// This is a compatibility header that allows using TimeDependentDijkstraStateful
// with TimeDependentGraphClassic without needing a separate implementation.
//
// The standard TimeDependentDijkstraStateful template works with both
// TimeDependentGraph and TimeDependentGraphClassic because they
// have the same interface (same member function names and signatures).

// Type alias for convenience
template<typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
using TimeDependentDijkstraStatefulClassic =
    TimeDependentDijkstraStateful<TimeDependentGraphClassic, PROFILER, DEBUG, TARGET_PRUNING>;