# Copyright (c) 2025 Robert Bosch GmbH.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

# --- Allow manual CPU target specification (cache it so every CMake invocation sees it) ---
if(NOT DEFINED TARGET_CPU)
  set(TARGET_CPU CACHE STRING "Manually specify target CPU (x86, A53, A72, A76)" FORCE)
endif()

message(STATUS "TARGET_CPU (from cache): ${TARGET_CPU}")

# --- Skip architecture detection if TARGET_CPU is set ---
if(TARGET_CPU)
    message(STATUS "Using manually specified TARGET_CPU: ${TARGET_CPU}")
else()
    # --- Detect Architecture from lscpu if TARGET_CPU is NOT set ---
    if(NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
        execute_process(COMMAND lscpu
                        OUTPUT_VARIABLE LSC_INFO
                        OUTPUT_STRIP_TRAILING_WHITESPACE)

        # Extract CPU architecture and model name from lscpu output
        string(REGEX REPLACE ".*Architecture:[ \t]+([^ \t\n]+).*" "\\1" CPU_ARCH ${LSC_INFO})
        string(REGEX REPLACE ".*Model name:[ \t]+([^ \t\n]+).*" "\\1" CPU_MODEL ${LSC_INFO})

        # Debugging the lscpu output
        message(STATUS "Detected CPU_ARCH: ${CPU_ARCH}")
        message(STATUS "Detected CPU_MODEL: ${CPU_MODEL}")

        # Fail if CPU_ARCH or CPU_MODEL could not be determined
        if(NOT CPU_ARCH)
            message(FATAL_ERROR "Failed to detect CPU architecture from lscpu.")
        endif()

        if(NOT CPU_MODEL)
            message(FATAL_ERROR "Failed to detect CPU model from lscpu.")
        endif()

        set(CMAKE_SYSTEM_PROCESSOR ${CPU_ARCH} CACHE STRING "Detected processor" FORCE)
    endif()
endif()

# --- Detect Architecture and Set Flags ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    message(STATUS "Detected x86_64 architecture: Using x86 build flags.")
    set(ARCH_FLAGS "-march=x86-64" "-mtune=generic" "-pipe")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    if(CPU_MODEL MATCHES "Cortex-A53")
        message(STATUS "Using ARM Cortex-A53: Using ARMv8-A architecture")
        set(ARCH_FLAGS "-mcpu=cortex-a53" "-mtune=cortex-a53" "-pipe")
    else()
        message(WARNING "Unknown ARM CPU model: ${CPU_MODEL}. Defaulting to ARMv8-A")
        set(ARCH_FLAGS "-march=armv8-a" "-pipe")
    endif()
else()
    message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}. Defaulting to generic flags.")
    set(ARCH_FLAGS "-march=native" "-pipe")
endif()

# --- Optimization Level: -O2 (default) or -O3 (optional) ---
option(USE_O3 "Enable -O3 optimization" OFF)
if(USE_O3)
    set(OPT_FLAGS "-O3")
else()
    set(OPT_FLAGS "-O2")
endif()

# --- General ROS 2-specific optimization flags ---
set(ROS2_OPTIMIZATION_FLAGS "-ffast-math -funsafe-math-optimizations -fomit-frame-pointer")

# --- Combine all flags ---
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPT_FLAGS} ${ROS2_OPTIMIZATION_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPT_FLAGS} ${ROS2_OPTIMIZATION_FLAGS} -pthread")

# Append ARCH_FLAGS separately to ensure proper handling
foreach(FLAG ${ARCH_FLAGS})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
endforeach()

# --- Set build type to Release if not specified ---
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release") # Default to Release for optimization
endif()

# --- Generate compile_commands.json for development tools ---
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Debugging Information ---
message(STATUS "Using toolchain.cmake")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
