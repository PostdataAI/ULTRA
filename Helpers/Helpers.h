#pragma once

#include <set>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <thread>
#include <chrono>

#include <strings.h>
#include <string.h>

#include "HighlightText.h"

inline double degreesToRadians(const double radius) {
    return (radius / 180.0) * M_PI;
}

inline double radiansToDegrees(const double radius) {
    return (radius / M_PI) * 180;
}

template<typename T>
inline T floor(const T value, const T unit) noexcept {
    return std::floor((value - unit) / unit) * unit;
}

template<typename T>
inline T ceil(const T value, const T unit) noexcept {
    return std::ceil((value + unit) / unit) * unit;
}

template<typename T>
inline void sort(std::vector<T>& data) noexcept {
    std::sort(data.begin(), data.end());
}

template<typename T, typename LESS>
inline void sort(std::vector<T>& data, const LESS& less) noexcept {
    std::sort(data.begin(), data.end(), less);
}

template<typename T>
inline void suppressUnusedParameterWarning(const T&) noexcept {}

inline int branchlessConditional(const bool predicate, const int ifTrue, const int ifFalse) noexcept {
    // Cross-platform branchless conditional using standard C++
    // This generates efficient branchless code on modern compilers for any architecture
    return predicate ? ifTrue : ifFalse;
}
