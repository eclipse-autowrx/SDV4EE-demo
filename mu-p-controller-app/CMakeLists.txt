# Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

project(Haptic_Force_Controller)
cmake_minimum_required(VERSION 3.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(${ARCH_FLAGS} ${OPT_FLAGS} -Wall -Wextra -Wpedantic)
endif()

set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(modules/kuksa-incubation/kuksa-cpp-client)

include_directories(/usr/include/jsoncpp)


option(BUILD_HAPTIC_FORCE_CONTROLLER "Build haptic_force_controller target" ON)


if(BUILD_HAPTIC_FORCE_CONTROLLER)
  add_executable(haptic_force_controller
    src/haptic_force_controller.cpp
    src/helper/motor_control.cpp
    src/helper/motor_state.cpp
    src/helper/multi_control3.cpp
    src/helper/udp_communication.cpp
    src/helper/serializer.cpp
    src/helper/kuksa_thread.cpp
    src/config/motor_control_parameters.cpp
    src/config/com_parameters.cpp
    src/config/com_parameters_statistic.cpp
  )

  target_include_directories(haptic_force_controller PRIVATE
    include/
    include/config
  )

  target_link_libraries(haptic_force_controller PRIVATE ${MOSQUITTO_LIBRARIES} jsoncpp ${YAML_CPP_LIBRARIES} kuksaclient)
  target_include_directories(haptic_force_controller PRIVATE ${MOSQUITTO_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})

  install(TARGETS haptic_force_controller DESTINATION ${INSTALL_DIR})
endif()

