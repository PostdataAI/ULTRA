# ARM Android Compatibility Guide

This document explains the changes made to enable ULTRA to compile and run on ARM Android CPUs while maintaining compatibility with x86 systems.

## Summary of Changes

The ULTRA codebase has been modified to support cross-platform compilation targeting ARM Android devices. All x86-specific code has been replaced with portable alternatives.

### 1. Removed Unused SSE Dependencies
Simply SSE is a set of special CPU instructions created by INTEL/AMD

**File:** `Helpers/Meta.h`
- **Change:** Commented out unused `#include <emmintrin.h>`
- **Reason:** The SSE2 header was included but no SSE intrinsics were actually used
- **Impact:** Eliminates x86-specific dependency without affecting functionality
- **Explanation:** We removed a CPU-specific header that wasn't needed. This helps the project compile cleanly on phones and tablets (ARM) and keeps the code simpler.

### 2. Replaced x86 Inline Assembly

**File:** `Helpers/Helpers.h`
- **Change:** Replaced x86 inline assembly in `branchlessConditional()` with portable C++
- **Before:** Used x86 `mov`, `test`, and `cmovz` instructions
- **After:** Simple ternary operator `predicate ? ifTrue : ifFalse`
- **Impact:** Modern compilers generate efficient branchless code on all architectures
- **Explanation:** We swapped special low-level CPU instructions for normal C++ code. It works the same on all devices, and modern compilers keep it fast.

### 3. Cross-Platform Build Configuration

**File:** `CMakeLists.txt`
- **Changes:**
  - Architecture detection for optimization flags
  - ARM-specific optimization flags (`-mcpu=native` or `-march=armv8-a+simd` for Android)
  - x86 systems continue using `-march=native`
  - Optional SIMDe integration for future SIMD needs
- **Explanation:** The build now automatically chooses the best settings for your device (ARM Android or x86 PC) so it runs well everywhere without manual tweaks.

### 4. Platform-Aware Threading Code

**File:** `Helpers/MultiThreading.h`
- **Changes:**
  - Conditional NUMA support (Linux only)
  - Fallback implementations for non-Linux platforms
  - Thread affinity functions made platform-aware
  - Graceful degradation when NUMA is unavailable
- **Explanation:** Code that uses multiple CPU cores now adapts to each platform. It uses extra features when available and falls back safely when they aren't.

### 5. Future SIMD Support

**File:** `Helpers/SIMD.h` (new)
- **Purpose:** Ready-to-use header for cross-platform SIMD operations
- **Technology:** SIMDe (SIMD Everywhere) for SSE ↔ NEON translation
- **Usage:** Include this header and use standard SSE intrinsics on any platform
- **Explanation:** We prepared a simple way to write fast number-crunching code once and have it work on any CPU (ARM or x86) without changes.
