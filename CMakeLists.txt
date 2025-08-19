cmake_minimum_required(VERSION 3.22)
project(ULTRA)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#Compiler settings
set(CMAKE_CXX_STANDARD 23)

# Architecture-specific optimization flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64|ARM64")
    # ARM/ARM64 optimization flags
    set(ARCH_FLAGS "-mcpu=native")
    if(ANDROID)
        # Android ARM-specific optimizations
        set(ARCH_FLAGS "-march=armv8-a+simd")
    endif()
else()
    # x86/x86_64 optimization flags
    set(ARCH_FLAGS "-march=native")
endif()

set(CMAKE_CXX_FLAGS "-pipe ${ARCH_FLAGS} -Wfatal-errors")
set(CMAKE_CXX_FLAGS_DEBUG "-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3")

#Libraries
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

# Optional SIMDe support for cross-platform SIMD
option(ULTRA_USE_SIMDE "Enable SIMDe for cross-platform SIMD support" OFF)
if(ULTRA_USE_SIMDE)
    find_path(SIMDE_INCLUDE_DIR 
        NAMES simde/simde.h
        HINTS ${CMAKE_SOURCE_DIR}/third_party/simde
              ${CMAKE_SOURCE_DIR}/simde
              ${CMAKE_SOURCE_DIR}/../simde
    )
    
    if(SIMDE_INCLUDE_DIR)
        message(STATUS "Found SIMDe at: ${SIMDE_INCLUDE_DIR}")
        add_compile_definitions(ULTRA_USE_SIMDE)
    else()
        message(WARNING "SIMDe not found. Clone from https://github.com/simd-everywhere/simde")
    endif()
endif()

function(add_runnable name)
    add_executable(${ARGV})
    target_link_libraries(${name} PUBLIC OpenMP::OpenMP_CXX)
    
    # Add SIMDe include directory if available
    if(ULTRA_USE_SIMDE AND SIMDE_INCLUDE_DIR)
        target_include_directories(${name} PRIVATE ${SIMDE_INCLUDE_DIR})
    endif()
endfunction()

set(RUNNABLES DelayExperiments Network ULTRA ULTRAPHAST)
foreach(f ${RUNNABLES})
    add_runnable(${f} Runnables/${f}.cpp)
endforeach()

add_custom_target(All)
add_dependencies(All ${RUNNABLES})