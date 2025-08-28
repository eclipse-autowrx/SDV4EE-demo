# Copyright (c) 2025 Robert Bosch GmbH.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gimbal_can_gateway)

# Add library for CAN
add_library(can STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can/can.c
)

# Include paths for the CAN library
target_include_directories(can PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp
)

# Link with Zephyr (if required, depending on Zephyr configuration)
target_link_libraries(can PUBLIC zephyr_interface)

# Add library for UDP
add_library(udp STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp/udp.c
)

# Include paths for the UDP library
target_include_directories(udp PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp  
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can
)

# Link with Zephyr (if required, depending on Zephyr configuration)
target_link_libraries(udp PUBLIC zephyr_interface)

# Create the Zephyr executable
target_sources(app PRIVATE src/main.c)

# Include paths for the executable
target_include_directories(app PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src  # Include the directory containing debug.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can  # Include the directory containing can.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/udp  # Include the directory containing udp.h
)

# Link libraries
target_link_libraries(app PRIVATE can udp)