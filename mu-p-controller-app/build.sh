#!/bin/bash

# Copyright (c) 2025 Robert Bosch GmbH.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

# Set the build directory
BUILD_DIR="build"

# If no TARGET_CPU is provided, leave it empty for auto-detection in the toolchain
TARGET_CPU="$1"  # Accept TARGET_CPU as the first argument to the script

# # Remove the existing build directory to ensure a clean build
# if [ -d "$BUILD_DIR" ]; then
#     echo "Removing existing build directory..."
#     rm -rf "$BUILD_DIR"
# fi

# Create a fresh build directory
if [ ! -d "$BUILD_DIR" ]; then
mkdir -p "$BUILD_DIR"
fi
cd "$BUILD_DIR"

# If TARGET_CPU is not specified, we pass nothing (let the toolchain autodetect)
if [ -z "$TARGET_CPU" ]; then
    echo "No TARGET_CPU specified. Letting the toolchain autodetect."
else
    echo "Using manually specified TARGET_CPU: $TARGET_CPU"
fi

# Run CMake with the toolchain file, passing the TARGET_CPU (if provided)
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
      -DUSE_O3=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DTARGET_CPU="$TARGET_CPU" ..
      
# Build the project using all available CPU cores
cmake --build . -- -j$(nproc)
