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

#include <arpa/inet.h>

#include "MixTools/RTthreads/RTthreads.hpp"
#include "multi_control3.hpp"
#include "serializer.hpp"
#include "udp_communication.hpp"
#include "statistic_thread.hpp" 

template <typename receiveData>
class ReceiverThreadMulti3 : public RTthread {
   public:
   ReceiverThreadMulti3(int priority, int policy, std::vector<int> cores, MultiControl3 &multi_control3,
    UdpCommunication &udpcom_, StatisticThread<StatisticData>& statistic_thread)
            : RTthread(priority, policy, cores),
            multi_control3_(multi_control3),
            udpcom_(udpcom_),
            statistic_thread_(statistic_thread),  
            packet_rx_left_{},
            packet_rx_right_{},
            packet_rx_buzzwire_left_{},
            packet_rx_buzzwire_right_{} {}

    inline void run() noexcept override { receiveMessagesCallbacks(); }

   private:
    MultiControl3 &multi_control3_;
    UdpCommunication &udpcom_;
    StatisticThread<StatisticData>& statistic_thread_;
    receiveData packet_rx_left_;
    receiveData packet_rx_right_;
    receiveData packet_rx_buzzwire_left_;
    receiveData packet_rx_buzzwire_right_;

    void receiveMessagesCallbacks();
};

template <typename receiveData>
void ReceiverThreadMulti3<receiveData>::receiveMessagesCallbacks() {
    int receiveCallbackSocketIndex_left = 0;
    int receiveCallbackSocketIndex_right = 1;
    int receiveCallbackSocketIndex_buzzwire_left = 2;
    int receiveCallbackSocketIndex_buzzwire_right = 3;

    // Lambda function to handle incoming UDP packets on socket 0
    auto callback_left = [this, receiveCallbackSocketIndex_left]() -> void {
        // Deserialize the received buffer into an MotorData_Rx packet
        Serializer<MotorData_Rx>::deserialize(packet_rx_left_,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_left).receiveBuffer,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_left).receiveBufferSize);

        if (packet_rx_left_.can_frame_id == 0x21){

            // Update the motor control with the new data
            if (multi_control3_.getLeftState().getTimeCount() < 10) {
                multi_control3_.getLeftState().setInitialAngle(packet_rx_left_.motor_angle);
            }
            multi_control3_.getLeftState().updateMotorAngle(packet_rx_left_.motor_angle);
           // std::cout << "current left angle: " << packet_rx_left_.motor_angle << std::endl;
        }

        if (packet_rx_left_.can_frame_id == 0x51){

            auto time_point = std::chrono::steady_clock::now();
            auto duration = time_point.time_since_epoch();
            uint64_t time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

            uint16_t roundtrip_latency = time_stamp - packet_rx_left_.time_stamp;
            //std::cout << "current left roundtrip_latency: " << roundtrip_latency << std::endl;
            multi_control3_.getLeftState().setRoundtripLatency(roundtrip_latency);
        }

        
        if (packet_rx_left_.can_frame_id == 0x61){
            uint16_t roundtrip_latency = multi_control3_.getLeftState().getDuration();
           //  std::cout << "current right roundtrip_latency: " << roundtrip_latency << std::endl;
            multi_control3_.getLeftState().setRoundtripLatency(roundtrip_latency);
        }

        if (packet_rx_left_.can_frame_id == 0x71){

            statistic_thread_.sendPongMsg(receiveCallbackSocketIndex_left);

            // std::cout << "current left CAN latency (roundtrip)): " << packet_rx_left_.can_latency << std::endl;
            multi_control3_.getLeftState().setCanLatency(packet_rx_left_.can_latency);
            // std::cout << "current left Eth latency (roundtrip)): " << packet_rx_left_.eth_latency << std::endl;
            multi_control3_.getLeftState().setEthLatency(packet_rx_left_.eth_latency);

        }

    };

    // Lambda function to handle incoming UDP packets on socket 1
    auto callback_right = [this, receiveCallbackSocketIndex_right]() -> void {
        // Deserialize the received buffer into an MotorData_Rx packet
        Serializer<MotorData_Rx>::deserialize(packet_rx_right_,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_right).receiveBuffer,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_right).receiveBufferSize);


        if (packet_rx_right_.can_frame_id == 0x21){

            // Update the motor control with the new data
            if (multi_control3_.getRightState().getTimeCount() < 10) {
                multi_control3_.getRightState().setInitialAngle(packet_rx_right_.motor_angle);
            }
            multi_control3_.getRightState().updateMotorAngle(packet_rx_right_.motor_angle);
            //std::cout << "current right angle: " << packet_rx_right_.motor_angle << std::endl;
        }

        if (packet_rx_right_.can_frame_id == 0x51){

            auto time_point = std::chrono::steady_clock::now();
            auto duration = time_point.time_since_epoch();
            uint64_t time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

            uint16_t roundtrip_latency = time_stamp - packet_rx_right_.time_stamp;
           //  std::cout << "current right roundtrip_latency: " << roundtrip_latency << std::endl;
            multi_control3_.getRightState().setRoundtripLatency(roundtrip_latency);
        }

        if (packet_rx_right_.can_frame_id == 0x61){

            uint16_t roundtrip_latency = multi_control3_.getRightState().getDuration();
           //  std::cout << "current right roundtrip_latency: " << roundtrip_latency << std::endl;
            multi_control3_.getRightState().setRoundtripLatency(roundtrip_latency);
        }
        if (packet_rx_right_.can_frame_id == 0x71){

            statistic_thread_.sendPongMsg(receiveCallbackSocketIndex_right);

            //std::cout << "current right CAN latency (roundtrip)): " << packet_rx_right_.can_latency << std::endl;
            multi_control3_.getRightState().setCanLatency(packet_rx_right_.can_latency);
            //std::cout << "current right Eth latency (roundtrip)): " << packet_rx_right_.eth_latency << std::endl;
            multi_control3_.getRightState().setEthLatency(packet_rx_right_.eth_latency);

        }

    };


    // Lambda function to handle incoming UDP packets
    auto callback_buzzwire_left = [this, receiveCallbackSocketIndex_buzzwire_left]() -> void {
        // Deserialize the received buffer into an MotorData_Rx packet
        Serializer<MotorData_Rx>::deserialize(packet_rx_buzzwire_left_,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_buzzwire_left).receiveBuffer,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_buzzwire_left).receiveBufferSize);

        // Update the motor control with the new data
        if (multi_control3_.getBuzzWireLeftState().getTimeCount() < 10) {
            multi_control3_.getBuzzWireLeftState().setInitialAngle(packet_rx_buzzwire_left_.motor_angle);
        }
        multi_control3_.getBuzzWireLeftState().updateMotorAngle(packet_rx_buzzwire_left_.motor_angle);


    };

    // Lambda function to handle incoming UDP packets
    auto callback_buzzwire_right = [this, receiveCallbackSocketIndex_buzzwire_right]() -> void {
        // Deserialize the received buffer into an MotorData_Rx packet
        Serializer<MotorData_Rx>::deserialize(packet_rx_buzzwire_right_,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_buzzwire_right).receiveBuffer,
                                            udpcom_.getSocketInfo(receiveCallbackSocketIndex_buzzwire_right).receiveBufferSize);

        // Update the motor control with the new data
        if (multi_control3_.getBuzzWireRightState().getTimeCount() < 10) {
            multi_control3_.getBuzzWireRightState().setInitialAngle(packet_rx_buzzwire_right_.motor_angle);
        }
        multi_control3_.getBuzzWireRightState().updateMotorAngle(packet_rx_buzzwire_right_.motor_angle);
    };

    udpcom_.registerReceiveCallback(receiveCallbackSocketIndex_left, callback_left);
    udpcom_.registerReceiveCallback(receiveCallbackSocketIndex_right, callback_right);
    udpcom_.registerReceiveCallback(receiveCallbackSocketIndex_buzzwire_left, callback_buzzwire_left);
    udpcom_.registerReceiveCallback(receiveCallbackSocketIndex_buzzwire_right, callback_buzzwire_right);

    // Start receiving messages using the callback function
    udpcom_.startReceiveCallbacks();
}

