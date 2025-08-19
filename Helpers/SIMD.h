#pragma once

/**
 * Cross-platform SIMD support using SIMDe
 * 
 * This header provides a cross-platform abstraction for SIMD operations.
 * It automatically detects the platform and provides the appropriate
 * intrinsics (SSE for x86/x86_64, NEON for ARM/ARM64).
 * 
 * Usage:
 * 1. Include this header instead of platform-specific headers
 * 2. Use standard SSE intrinsic names (e.g., _mm_add_ps)
 * 3. SIMDe will automatically translate to appropriate platform code
 * 
 * To enable SIMDe support:
 * 1. Clone SIMDe: git clone https://github.com/simd-everywhere/simde.git
 * 2. Add to CMakeLists.txt: target_include_directories(target PRIVATE simde)
 * 3. Define ULTRA_USE_SIMDE to enable SIMD operations
 */

#ifdef ULTRA_USE_SIMDE
    // Enable SIMDe for cross-platform SIMD support
    #define SIMDE_ENABLE_NATIVE_ALIASES  // Allow using _mm_* without simde_ prefix
    
    #if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
        // x86/x86_64 - use native SSE
        #include <simde/x86/sse.h>
        #include <simde/x86/sse2.h>
        #include <simde/x86/sse3.h>
        #include <simde/x86/ssse3.h>
        #include <simde/x86/sse4.1.h>
        #include <simde/x86/sse4.2.h>
        #include <simde/x86/avx.h>
        #include <simde/x86/avx2.h>
    #elif defined(__aarch64__) || defined(__arm__) || defined(_M_ARM) || defined(_M_ARM64)
        // ARM/ARM64 - SIMDe will translate SSE to NEON
        #include <simde/x86/sse.h>
        #include <simde/x86/sse2.h>
        #include <simde/x86/sse3.h>
        #include <simde/x86/ssse3.h>
        #include <simde/x86/sse4.1.h>
        #include <simde/x86/sse4.2.h>
        
        // Native ARM NEON is also available if needed
        #include <arm_neon.h>
    #else
        // Other architectures - SIMDe provides portable fallbacks
        #include <simde/x86/sse.h>
        #include <simde/x86/sse2.h>
    #endif
    
    // Define platform detection macros
    #if defined(__aarch64__) || defined(__arm__) || defined(_M_ARM) || defined(_M_ARM64)
        #define ULTRA_ARM_PLATFORM 1
    #else
        #define ULTRA_ARM_PLATFORM 0
    #endif
    
    #if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
        #define ULTRA_X86_PLATFORM 1
    #else
        #define ULTRA_X86_PLATFORM 0
    #endif
    
#else
    // SIMD support disabled - provide empty defines to avoid compilation errors
    #warning "SIMDe not enabled. Define ULTRA_USE_SIMDE to enable cross-platform SIMD support."
    
    #define ULTRA_ARM_PLATFORM 0
    #define ULTRA_X86_PLATFORM 0
    
#endif

// Platform-specific optimization hints
#if defined(__GNUC__) || defined(__clang__)
    #define ULTRA_FORCE_INLINE __attribute__((always_inline)) inline
    #define ULTRA_LIKELY(x) __builtin_expect(!!(x), 1)
    #define ULTRA_UNLIKELY(x) __builtin_expect(!!(x), 0)
#elif defined(_MSC_VER)
    #define ULTRA_FORCE_INLINE __forceinline
    #define ULTRA_LIKELY(x) (x)
    #define ULTRA_UNLIKELY(x) (x)
#else
    #define ULTRA_FORCE_INLINE inline
    #define ULTRA_LIKELY(x) (x)
    #define ULTRA_UNLIKELY(x) (x)
#endif

// Memory alignment helpers
#if defined(__GNUC__) || defined(__clang__)
    #define ULTRA_ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER)
    #define ULTRA_ALIGN(n) __declspec(align(n))
#else
    #define ULTRA_ALIGN(n)
#endif

// Common SIMD alignment (16 bytes for SSE, 16 bytes for NEON)
#define ULTRA_SIMD_ALIGN ULTRA_ALIGN(16)

/**
 * Example usage (when ULTRA_USE_SIMDE is defined):
 * 
 * #include "SIMD.h"
 * 
 * void vectorAdd(const float* a, const float* b, float* result, size_t count) {
 *     for (size_t i = 0; i < count; i += 4) {
 *         __m128 va = _mm_load_ps(&a[i]);
 *         __m128 vb = _mm_load_ps(&b[i]);
 *         __m128 vr = _mm_add_ps(va, vb);
 *         _mm_store_ps(&result[i], vr);
 *     }
 * }
 * 
 * This code will work on x86 (using SSE) and ARM (using NEON via SIMDe).
 */
