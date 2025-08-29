/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <iostream>
#include <sstream>
#include <json/json.h>  // Include JSON library
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <atomic>
#include <iomanip> 

#define STAT_UPDATE_DT 1000
#define CONTROL_UPDATE_DT 100

template <typename statisticData>
class StatisticThread : public CyclicRTthread {
   public:
    StatisticThread(int priority, int policy, std::vector<int> cores, int period_ns, statisticData &statistic_data,
                    MultiControl3 &multi_control3, UdpCommunication &udpcom, mosquitto* mosq, bool is_mqtt_connected)
        : CyclicRTthread(priority, policy, cores, period_ns),
          statistic_data_(statistic_data),
          multi_control3_(multi_control3),
          udpcom_(udpcom),
          mosq_(mosq),
          statistic_counter_(0),
          old_time_stamp_(0),
          walltimer_counter_(0),
          display_line_(1),
          walltimer_jitter_(0),
          walltimer_avg_(0),
          walltimer_sum_(0),
          walltimer_min_(65535),
          walltimer_max_(0),
          roundtrip_jitter_left_(0),
          roundtrip_jitter_right_(0),
          roundtrip_avg_left_(0),
          roundtrip_avg_right_(0),
          roundtrip_sum_left_(0),
          roundtrip_sum_right_(0),
          roundtrip_min_left_(65535),
          roundtrip_min_right_(65535),
          roundtrip_max_left_(0),
          roundtrip_max_right_(0),
          is_mqtt_connected_(is_mqtt_connected) {}

    bool loop() override { return timerCallback(); }
    void setMQTTStatus(bool status) { is_mqtt_connected_ = status; }
    bool getMQTTStatus() { return is_mqtt_connected_; }
    void calcStatistic();
    void publishToMQTT(const std::string &message, const std::string &topic);
    void sendStatistic();
    void sendMqttStatistics();
    void sendPingMsg(int socket_number);
    void sendPongMsg(int socket_number);
    void sendFastPingMsg(int socket_number);
    


   private:
    statisticData &statistic_data_;
    MultiControl3 &multi_control3_;
    UdpCommunication &udpcom_;
    mosquitto* mosq_;  // MQTT Client

    uint16_t statistic_counter_;
    uint64_t old_time_stamp_;
    uint64_t walltimer_counter_;
    size_t display_line_;
    uint16_t walltimer_jitter_;
    uint16_t walltimer_avg_;
    float walltimer_sum_;
    uint16_t walltimer_min_;
    uint16_t walltimer_max_;
    uint16_t roundtrip_jitter_left_;
    uint16_t roundtrip_jitter_right_;
    uint16_t roundtrip_avg_left_;
    uint16_t roundtrip_avg_right_;
    float roundtrip_sum_left_;
    float roundtrip_sum_right_;
    uint16_t roundtrip_min_left_;
    uint16_t roundtrip_min_right_;
    uint16_t roundtrip_max_left_;
    uint16_t roundtrip_max_right_;
    bool is_mqtt_connected_;

    bool timerCallback();
    
    void sendStatisticCanFrame(uint16_t roundtrip_avg, uint16_t roundtrip_jitter, 
                  uint16_t walltimer_avg, uint16_t walltimer_jitter, 
                  int socket_number);

    // Functions to get CPU and Memory Usage
    float getCpuUtilization();
    float getMemoryUsage();
};

template <typename statisticData>
float StatisticThread<statisticData>::getCpuUtilization() {
    float cpu_usage = 0.0;

    std::ifstream loadavg_file("/proc/loadavg");
    if (loadavg_file.is_open()) {
        std::string loadavg;
        // Read the first line of /proc/loadavg
        std::getline(loadavg_file, loadavg);

        // The first value is the 1-minute load average
        std::stringstream ss(loadavg);
        ss >> cpu_usage;  // Extract the 1-minute load average

        loadavg_file.close();
    } else {
        std::cerr << "Error: Unable to read /proc/loadavg" << std::endl;
    }

    return cpu_usage * 100.0;
}


template <typename statisticData>
float StatisticThread<statisticData>::getMemoryUsage() {
    std::ifstream mem_file("/proc/meminfo");
    std::string line;
    long mem_total = 0;
    long mem_free = 0;
    long buffers = 0;
    long cached = 0;
    long s_reclaimable = 0;

    // Read values from /proc/meminfo
    while (std::getline(mem_file, line)) {
        std::stringstream ss(line);
        std::string key;
        ss >> key;

        if (key == "MemTotal:") {
            ss >> mem_total;
        }
        if (key == "MemFree:") {
            ss >> mem_free;
        }
        if (key == "Buffers:") {
            ss >> buffers;
        }
        if (key == "Cached:") {
            ss >> cached;
        }
        if (key == "SReclaimable:") {
            ss >> s_reclaimable;
        }
    }

    // Calculate used memory like htop does:
    // Used Memory = MemTotal - MemFree - Buffers - Cached - SReclaimable
    long used_memory_kb = mem_total - mem_free - buffers - cached - s_reclaimable;

    // Calculate memory usage percentage
    float used_memory_percentage = (used_memory_kb / static_cast<float>(mem_total)) * 100.0;

    return used_memory_percentage; // Return memory usage percentage
}


template <typename statisticData>
bool StatisticThread<statisticData>::timerCallback() {
    sendMqttStatistics();
    return false;
}


template <typename statisticData>
void StatisticThread<statisticData>::sendPongMsg(int socket_number) {
    // Create an empty CAN frame (no data payload)
    CANFrame frame;
    frame.id = 0x70;  
    frame.dlc = 0;    

    // Serialize the CAN frame into the UDP send buffers
    char* buffer = udpcom_.getSocketInfo(socket_number).sendBuffer;
    size_t bufferSize = udpcom_.getSocketInfo(socket_number).sendBufferSize;

    // Copy the CAN frame to the buffer and send it if buffer size is sufficient
    if (bufferSize >= sizeof(CANFrame)) {
        std::memcpy(buffer, &frame, sizeof(CANFrame));
        
        // Send the data through the specified socket
        udpcom_.sendToSocket(socket_number);
    }
}


template <typename statisticData>
void StatisticThread<statisticData>::sendPingMsg(int socket_number) {

    char* buffer = udpcom_.getSocketInfo(socket_number).sendBuffer;
    size_t bufferSize = udpcom_.getSocketInfo(socket_number).sendBufferSize;

    if (bufferSize < sizeof(CANFrame)) {
        throw std::runtime_error("Buffer size is too small");
    }
    
    // Create CAN frame
    CANFrame frame;
    frame.id = 0x50;  // Set CAN ID
    frame.dlc = 8;    // 8 bytes for 64-bit timestamp

    std::size_t offset = 0;
   
    std::memcpy(buffer, &frame.id, sizeof(frame.id));
    offset += sizeof(frame.id);

    std::memcpy(buffer + offset, &frame.dlc, sizeof(frame.dlc));
    offset += sizeof(frame.dlc);

    // Get current timestamp in microseconds
    uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

     // Copy data payload (8 bytes)
    std::memcpy(buffer + offset, &timestamp_us, sizeof(timestamp_us));

    // Send the data through sockets
    udpcom_.sendToSocket(socket_number);
    
}

template <typename statisticData>
void StatisticThread<statisticData>::sendFastPingMsg(int socket_number) {

    char* buffer = udpcom_.getSocketInfo(socket_number).sendBuffer;
    size_t bufferSize = udpcom_.getSocketInfo(socket_number).sendBufferSize;

    if (bufferSize < sizeof(CANFrame)) {
        throw std::runtime_error("Buffer size is too small");
    }
        
    // Create an empty CAN frame
    CANFrame frame;
    frame.id = 0x60;   // Set CAN ID
    frame.dlc = 0;     // Set data length to 0 — empty payload
    
    std::size_t offset = 0;

    std::memcpy(buffer, &frame.id, sizeof(frame.id));
    offset += sizeof(frame.id);

    std::memcpy(buffer + offset, &frame.dlc, sizeof(frame.dlc));
    offset += sizeof(frame.dlc);
  
    if (socket_number == 0) {
        multi_control3_.getLeftState().setTimepoint();
    } else {
        multi_control3_.getRightState().setTimepoint();
    }
    // Send the data through both sockets
    udpcom_.sendToSocket(socket_number);
    
}


template <typename statisticData>
void StatisticThread<statisticData>::calcStatistic() {
    auto time_point = std::chrono::steady_clock::now();
    auto duration = time_point.time_since_epoch();
    uint64_t time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();


    walltimer_counter_++;
    if (old_time_stamp_ > 0) {
        uint16_t wt_duration = time_stamp - old_time_stamp_;

        if (wt_duration > walltimer_max_)
            walltimer_max_ = wt_duration;
        if (wt_duration < walltimer_min_)
            walltimer_min_ = wt_duration;
        walltimer_sum_ += wt_duration;

        uint16_t roundtrip_latency_left = multi_control3_.getLeftRoundtripLatency();
        uint16_t roundtrip_latency_right = multi_control3_.getRightRoundtripLatency();

        roundtrip_sum_left_ += roundtrip_latency_left;
        roundtrip_sum_right_ += roundtrip_latency_right;

        roundtrip_max_left_ = std::max(roundtrip_max_left_, roundtrip_latency_left);
        roundtrip_min_left_ = std::min(roundtrip_min_left_, roundtrip_latency_left);

        roundtrip_max_right_ = std::max(roundtrip_max_right_, roundtrip_latency_right);
        roundtrip_min_right_ = std::min(roundtrip_min_right_, roundtrip_latency_right);


        if (walltimer_counter_ % STAT_UPDATE_DT == 0) {
            walltimer_avg_ = walltimer_sum_ / STAT_UPDATE_DT;
            walltimer_jitter_ = std::max(walltimer_max_ - walltimer_avg_, walltimer_avg_ - walltimer_min_); 

            roundtrip_avg_left_ = roundtrip_sum_left_ / STAT_UPDATE_DT;
            roundtrip_jitter_left_ = roundtrip_max_left_ - roundtrip_min_left_;

            roundtrip_avg_right_ = roundtrip_sum_right_ / STAT_UPDATE_DT;
            roundtrip_jitter_right_ = roundtrip_max_right_ - roundtrip_min_right_;

            // Reset counters
            walltimer_sum_ = roundtrip_sum_left_ = roundtrip_sum_right_ = 0;
            walltimer_min_ = roundtrip_min_left_ = roundtrip_min_right_ = 65535;
            walltimer_max_ = roundtrip_max_left_ = roundtrip_max_right_ = 0;

        }
    }
    old_time_stamp_ = time_stamp;
}


template <typename statisticData>
void StatisticThread<statisticData>::sendMqttStatistics() {

    // Get CPU and memory usage
    int cpu_usage = getCpuUtilization();
    int memory_usage = getMemoryUsage();


    if (display_line_ == 2) {
        std::cout << std::endl << std::endl;

    }
    if (display_line_ % 100 == 0 || display_line_ == 2) {
        std::cout << "---------------|----------------|----------------------|-------------------|----------------|-------------------" << std::endl;
        std::cout << "OS Jitter (µs) | Ping Time (µs) | Eth Latency (µs)     | CAN Latency (µs)  | CPU Usage (%)  | Memory Usage (%)" << std::endl;
        std::cout << "---------------|----------------|----------------------|-------------------|----------------|-------------------" << std::endl;
    }
    if (display_line_ > 2) {
        std::cout 
            << std::setw(14) << walltimer_jitter_ << " | "
            << std::setw(14) << roundtrip_avg_left_ << " | "
            << std::setw(20) << multi_control3_.getLeftEthLatency() << " | "
            << std::setw(17) << multi_control3_.getLeftCanLatency() << " | "
            << std::setw(14) << cpu_usage << " | "
            << std::setw(17) << memory_usage
            << std::endl;
    }
    display_line_++;

    Json::Value message;
    message["motor_angle"] = multi_control3_.getLeftAngle();
    message["ping_time"] = roundtrip_avg_left_;
    message["can_latency"] = multi_control3_.getLeftCanLatency() ;
    message["eth_latency"] = multi_control3_.getLeftEthLatency() ;
    message["CPU_usage"] = cpu_usage;
    message["memory_usage"] = memory_usage;
 
    std::stringstream ss;
    ss << message;
    publishToMQTT(ss.str(), "statistics/data");
}


template <typename statisticData>
void StatisticThread<statisticData>::publishToMQTT(const std::string &message, const std::string &topic) {
    if (is_mqtt_connected_) {
        int ret = mosquitto_publish(mosq_, NULL, topic.c_str(), message.length(), message.c_str(), 0, false);
        if (ret != MOSQ_ERR_SUCCESS) {
            // std::cerr << "Failed to publish MQTT message to topic: " << topic << std::endl;
            is_mqtt_connected_ = false;
        }
    }
}

template <typename statisticData>
void StatisticThread<statisticData>::sendStatistic() {
    uint16_t left_can_latency = multi_control3_.getLeftCanLatency();
    uint16_t right_can_latency = multi_control3_.getRightCanLatency();
    uint16_t left_eth_latency = multi_control3_.getLeftEthLatency();
    uint16_t right_eth_latency = multi_control3_.getRightEthLatency();    
    sendStatisticCanFrame(roundtrip_avg_left_, left_can_latency , left_eth_latency, walltimer_jitter_, 0);
    sendStatisticCanFrame(roundtrip_avg_right_, right_can_latency, right_eth_latency, walltimer_jitter_, 1);
}

template <typename statisticData>
void StatisticThread<statisticData>::sendStatisticCanFrame(uint16_t roundtrip_avg, uint16_t can_latency, 
                                                  uint16_t eth_latency, uint16_t os_jitter, 
                                                  int socket_number) {
    // Create CAN frame
    CANFrame frame;
    frame.id = 0x30;  // Set CAN ID
    frame.dlc = 8;    // 8 bytes for 4 uint16_t values

    // Serialize data into the frame's data field
    std::memcpy(frame.data, &roundtrip_avg, sizeof(roundtrip_avg));
    std::memcpy(frame.data + 2, &can_latency, sizeof(can_latency));
    std::memcpy(frame.data + 4, &eth_latency, sizeof(eth_latency));
    std::memcpy(frame.data + 6, &os_jitter, sizeof(os_jitter));

    // Get socket buffer
    char* buffer = udpcom_.getSocketInfo(socket_number).sendBuffer;
    size_t bufferSize = udpcom_.getSocketInfo(socket_number).sendBufferSize;

    // Copy the CAN frame to the buffer and send
    if (bufferSize >= sizeof(CANFrame)) {
        std::memcpy(buffer, &frame, sizeof(CANFrame));
        udpcom_.sendToSocket(socket_number);
    }
}

