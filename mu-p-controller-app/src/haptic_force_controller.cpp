/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include <csignal>
#include <mosquitto.h>
#include <iostream>
#include <cstdlib>
#include <cstring>  // For strlen()
#include <thread>   // For potential multithreading use
#include <atomic>
#include <yaml-cpp/yaml.h>
#include "com_parameters.hpp"
#include "com_parameters_statistic.hpp"
#include "multi_control3.hpp"
#include "receiver_thread_multi3.hpp"
#include "sender_thread_multi3.hpp"
#include "statistic_thread.hpp"
#include "udp_communication.hpp"
#include "kuksa_thread.hpp"
#include <yaml-cpp/yaml.h>
#include "kuksaclient.h"
#include <spdlog/spdlog.h>

#define MQTT_RECONNECT_DELAY 30
#define MQTT_IDLE_DELAY 30


SenderThreadMulti3<ControlData_Tx>* calcAndSendTorqueThreadMulti3Ptr = nullptr;  // Global pointer

// Signal handler for safe shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal received (" << signum << "). Exiting..." << std::endl;
    if (calcAndSendTorqueThreadMulti3Ptr) {  // Check for null
        calcAndSendTorqueThreadMulti3Ptr->triggerShutdown();
    }
    
    exit(signum);
}

void mqttReconnectManager(StatisticThread<StatisticData>& statisticThread, struct mosquitto *mosq, const std::string& host, int port) {
    bool is_connected = false;
    while(true){
        if (!is_connected) {
            int ret;
            ret = mosquitto_connect(mosq, host.c_str(), port, 60);
            if (ret == MOSQ_ERR_SUCCESS) {
                // Start or restart the MQTT network loop.
                mosquitto_loop_start(mosq);
                statisticThread.setMQTTStatus(true);
                is_connected = true;
                std::cout << "Reconnected to MQTT broker." << std::endl;
            } else {
                std::cerr << "Reconnect attempt failed: " << mosquitto_strerror(ret) << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(MQTT_RECONNECT_DELAY));
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(MQTT_IDLE_DELAY));
            is_connected = statisticThread.getMQTTStatus();
        }
    }
}

int main(int argc, char* argv[]) {
    // Check if the correct number of arguments is provided
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
        return 1;
    }

    // Read ythe yaml config file
    std::string configFile = argv[1];

    YAML::Node config;

    try {
        config = YAML::LoadFile(configFile);

        if (config["buzzwire"]) {
            std::cout << "Buzzwire device: " << (config["buzzwire"].as<bool>() ? "On" : "Off") << std::endl;
        }
        if (config["priority"]) {
            std::cout << "Priority: " << config["priority"].as<int>() << std::endl;
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        return 1;
    }


    spdlog::set_level(spdlog::level::warn); // Set global log level to debug    

    const bool is_buzzwire = config["buzzwire"].IsDefined() ? config["buzzwire"].as<bool>() : false;
    const int priority = config["priority"].IsDefined() ? config["priority"].as<int>() : 0;

    std::string ZONE_LEFT_IP = config["zone_left"]["ip"].IsDefined()
        ? config["zone_left"]["ip"].as<std::string>()
        : ZONE_LEFT_IP_DEFAULT;

    std::string ZONE_RIGHT_IP = config["zone_right"]["ip"].IsDefined()
        ? config["zone_right"]["ip"].as<std::string>()
        : ZONE_RIGHT_IP_DEFAULT;

    std::string ZONE_BUZZWIRE_LEFT_IP = config["zone_buzzwire_left"]["ip"].IsDefined()
        ? config["zone_buzzwire_left"]["ip"].as<std::string>()
        : ZONE_BUZZWIRE_LEFT_IP_DEFAULT;

    std::string ZONE_BUZZWIRE_RIGHT_IP = config["zone_buzzwire_right"]["ip"].IsDefined()
            ? config["zone_buzzwire_right"]["ip"].as<std::string>()
            : ZONE_BUZZWIRE_RIGHT_IP_DEFAULT;

    uint16_t ZONE_LEFT_PORT = config["zone_left"]["port"].IsDefined()
        ? static_cast<uint16_t>(config["zone_left"]["port"].as<int>())
        : ZONE_DEFAULT_PORT;
    
    uint16_t ZONE_RIGHT_PORT = config["zone_right"]["port"].IsDefined()
        ? static_cast<uint16_t>(config["zone_right"]["port"].as<int>())
        : ZONE_DEFAULT_PORT;
    
    uint16_t ZONE_BUZZWIRE_LEFT_PORT = config["zone_buzzwire_left"]["port"].IsDefined()
        ? static_cast<uint16_t>(config["zone_buzzwire_left"]["port"].as<int>())
        : ZONE_DEFAULT_PORT;
    
    uint16_t ZONE_BUZZWIRE_RIGHT_PORT = config["zone_buzzwire_right"]["port"].IsDefined()
        ? static_cast<uint16_t>(config["zone_buzzwire_right"]["port"].as<int>())
        : ZONE_DEFAULT_PORT;
        
    // Check if the priority is within the valid range
    if (priority < 0 || priority > 99) {
        std::cerr << "Error: Priority must be between 0 and 99." << std::endl;
        return 1;
    }

    // Parse configuration
    // YAML::Node config = YAML::LoadFile(argv[2]);
    const std::string kuksa_host = config["kuksa"]["host"].as<std::string>();
    const int kuksa_port = config["kuksa"]["port"].as<int>();
    const unsigned kuksa_periodic_task_ms = config["kuksa"]["periodic_task_ms"].as<unsigned>();
    const unsigned kuksa_periodic_task_ns = kuksa_periodic_task_ms * 1000 * 1000;
    const unsigned kuksa_task_priority = config["kuksa"]["task_priority"].as<unsigned>();

    // Connect to KUKSA databroker
    kuksa::KuksaClient kuksa_client;
    spdlog::set_level(spdlog::level::warn); // Set global log level to debug
    const std::string kuksa_connection = kuksa_host + ":" + std::to_string(kuksa_port);
    bool is_kuksa_connected = kuksa_client.connect_v2(kuksa_connection);
    if (!is_kuksa_connected) {
        std::cerr << "Error: Failed to connect to KUKSA databroker" << std::endl;
    } else {
        std::cout << "Connected to KUKSA broker " << kuksa_connection << std::endl;
    }

    // Set Scheduler appropriately, use FIFO unless prio is 0
    const int schedule_type = (priority == 0) ? SCHED_OTHER : SCHED_FIFO;
    signal(SIGINT, signalHandler);

    // --- MQTT Initialization ---
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new("client_id", true, NULL);
    if (!mosq) {
        std::cerr << "Failed to create Mosquitto instance." << std::endl;
        return 1;
    }

    // Parse MQTT params
    const std::string mqtt_host = config["mqtt"]["host"].as<std::string>();
    const int mqtt_port = config["mqtt"]["port"].as<int>();
    const std::string mqtt_user = config["mqtt"]["user"].as<std::string>();
    const std::string mqtt_pw = config["mqtt"]["pw"].as<std::string>();
    bool is_mqtt_connected = false;

    // Set username & password
    if (mosquitto_username_pw_set(mosq, mqtt_user.c_str(), mqtt_pw.c_str()) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to set MQTT authentication." << std::endl;
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // Connect to the MQTT broker
    if (mosquitto_connect(mosq, mqtt_host.c_str(), mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker. Launching reconnect thread later." << std::endl;
        is_mqtt_connected = false;
        // mosquitto_destroy(mosq);
        // mosquitto_lib_cleanup();
    }
    else {
        std::cout << "Connected to MQTT broker as " << mqtt_user << std::endl;
        is_mqtt_connected = true;

        // Start the MQTT loop
        mosquitto_loop_start(mosq); 
    }
    // --- Existing Motor and UDP Communication Setup ---
    UdpCommunication udpcom_(ZONE_LEFT_IP, ZONE_LEFT_PORT, buffer_rx_, buffer_size_rx_, buffer_tx_,
                             buffer_size_tx_);
    MotorControl motor_control_small_motor_left(controllerSmallMotorLeft);

    udpcom_.addSocket(ZONE_RIGHT_IP, ZONE_RIGHT_PORT, buffer2_rx_, buffer2_size_rx_, buffer2_tx_,
                      buffer2_size_tx_);
    MotorControl motor_control_small_motor_right(controllerSmallMotorRight);

    if (is_buzzwire) {
        udpcom_.addSocket(ZONE_BUZZWIRE_LEFT_IP, ZONE_BUZZWIRE_LEFT_PORT, buffer4_rx_, buffer4_size_rx_, buffer4_tx_,
                        buffer4_size_tx_);

        udpcom_.addSocket(ZONE_BUZZWIRE_RIGHT_IP, ZONE_BUZZWIRE_RIGHT_PORT, buffer5_rx_, buffer5_size_rx_, buffer5_tx_,
                        buffer5_size_tx_);
    }

    MotorControl motor_control_buzz_wire_left(controllerBuzzWireLeft);
    MotorControl motor_control_buzz_wire_right(controllerBuzzWireRight);

    MultiControl3 multicontrol3(motor_control_small_motor_left, motor_control_small_motor_right,
                                motor_control_buzz_wire_left, motor_control_buzz_wire_right, is_buzzwire);

    // Create the statistic thread (Ensure you have a valid statistic_data object)
    StatisticData statistic_data;  // You need to initialize this data as per your requirement
    StatisticThread<StatisticData> calcAndSendStatistic(priority, schedule_type, senderCPUCore, 
                                                        send_period_statistic_ns, statistic_data, multicontrol3, udpcom_, mosq, is_mqtt_connected);

    if(!is_mqtt_connected) {
        std::thread reconnectThread(mqttReconnectManager, std::ref(calcAndSendStatistic), mosq, mqtt_host, mqtt_port);
        reconnectThread.detach();
    }


    ReceiverThreadMulti3<MotorData_Rx> receiveMeasurementThreadMulti3(priority, schedule_type, receiverCPUCore, multicontrol3,
                                                                    udpcom_, calcAndSendStatistic);

    const unsigned period_ms = config["period_ms"].as<unsigned>();
    const unsigned period_ns = period_ms * 1000 * 1000;
    SenderThreadMulti3<ControlData_Tx> calcAndSendTorqueThreadMulti3(priority, schedule_type, senderCPUCore, period_ns,
                                                                     multicontrol3, udpcom_, calcAndSendStatistic,
                                                                     is_buzzwire);
   
    calcAndSendTorqueThreadMulti3Ptr = &calcAndSendTorqueThreadMulti3; 
   
    KuksaThread kuksaThread(kuksa_task_priority, schedule_type, kuksaCPUCore,
            kuksa_periodic_task_ns, kuksa_client, is_kuksa_connected, multicontrol3);

    // Start the threads
    receiveMeasurementThreadMulti3.start();
    calcAndSendTorqueThreadMulti3.start();
    calcAndSendStatistic.start();
    kuksaThread.start();

    // Join the threads
    receiveMeasurementThreadMulti3.join();
    calcAndSendTorqueThreadMulti3.join();
    calcAndSendStatistic.join();
    kuksaThread.join();

    // Cleanup MQTT before exit
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}

