/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <arpa/inet.h>

#include <vector>

#include "MixTools/RTthreads/RTthreads.hpp"
#include "config/com_parameters.hpp"
#include "config/control_parameters.hpp"
#include "logging.hpp"
#include "multi_control3.hpp"
#include "serializer.hpp"
#include "udp_communication.hpp"
#include "statistic_thread.hpp" 


#define MAX_BUZZWIRE_TORQUE 5

template <typename sendData>
class SenderThreadMulti3 : public CyclicRTthread {
   public:
SenderThreadMulti3(int priority, int policy, std::vector<int> cores, int period_ns, MultiControl3 &multi_control3,
                   UdpCommunication &udpcom_, StatisticThread<StatisticData>& statistic_thread, bool is_buzzwire)

    :

      CyclicRTthread(priority, policy, cores, period_ns),
      multi_control3_(multi_control3),  // 1st (matches declaration order)
      udpcom_(udpcom_),  // 2nd
      statistic_thread_(statistic_thread),  // 3rd (moved up to match declaration order)
      statistic_counter_(0),
      packet_tx_left_{0, 0.0},  // 4th
      packet_tx_right_{0, 0.0},  // 5th
      packet_tx_buzzwire_left_{0, 0.0},  // 6th
      packet_tx_buzzwire_right_{0, 0.0},  // 7th
      logging_data_{},
      is_buzzwire{is_buzzwire} 
    {
        clearBufferLeft();  // Initialize buffer
        clearBufferRight();
        if (is_buzzwire) {
            clearBufferBuzzWireLeft();
            clearBufferBuzzWireRight();
        }

    }

    inline bool loop() override {        
        if (stop_flag_) {
            return false;
        }
        return timerCallback();
    }
    
 
    void triggerShutdown() noexcept {
       shutdown();  
    }
  
   private:
    MultiControl3 &multi_control3_;
    UdpCommunication &udpcom_;
    StatisticThread<StatisticData>& statistic_thread_;
    uint32_t statistic_counter_;
    sendData packet_tx_left_;
    sendData packet_tx_right_;
    sendData packet_tx_buzzwire_left_;
    sendData packet_tx_buzzwire_right_;
    std::vector<double> logging_data_;
    bool is_buzzwire;
    volatile bool stop_flag_ = false; 
     
    void stop() noexcept {
        stop_flag_ = true;
    }
    
    void shutdown() noexcept {
        std::cout << "SenderThreadMulti3 shutting down..." << std::endl;

        auto time_point = std::chrono::steady_clock::now();
        auto duration = time_point.time_since_epoch();
        uint64_t time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

        packet_tx_left_ = {time_stamp, 0.0};
        packet_tx_right_ = {time_stamp, 0.0};
        if (is_buzzwire) {
            packet_tx_buzzwire_left_ = {time_stamp, 0.0};
            packet_tx_buzzwire_right_ = {time_stamp, 0.0};
        }

        serializeAndSend();
        
        stop();  // Stop the thread
    }



    bool timerCallback();
    void clearBufferLeft() noexcept;
    void clearBufferRight() noexcept;
    void clearBufferBuzzWireLeft() noexcept;
    void clearBufferBuzzWireRight() noexcept;


    void storeDatavec();
    void updatePacket();
    void loggingAndInfo();
    void serializeAndSend();


};

template <typename sendData>
bool SenderThreadMulti3<sendData>::timerCallback() {
    // update packet & execute motor_control calc()
    updatePacket();

// store in Datavec
#if STORE_DATAVEC
    storeDatavec();
#endif

    statistic_counter_++;
    // When counter reaches 1000, call calcStatistic and sendStatistic, then reset counter
    if (statistic_counter_ >= 1000) {
        statistic_thread_.sendStatistic();
        statistic_counter_ = 0;
    } 
    else 
    if (statistic_counter_ % 50 == 0) {
        if (statistic_counter_ % 100 == 0) {
            statistic_thread_.sendFastPingMsg(0);
        } else {
            statistic_thread_.sendFastPingMsg(1);
        }
    }
    else {
        serializeAndSend();
    }

    // logging & info to console
    loggingAndInfo();

    return false;
}

template <typename sendData>
void SenderThreadMulti3<sendData>::clearBufferLeft() noexcept {
    std::memset(udpcom_.getSocketInfo(0).sendBuffer, 0, udpcom_.getSocketInfo(0).sendBufferSize);
    // std::memset(buffer_tx_left_, 0, buffer_size_tx_left_);  // Clear the buffer
}

template <typename sendData>
void SenderThreadMulti3<sendData>::clearBufferRight() noexcept {
    std::memset(udpcom_.getSocketInfo(1).sendBuffer, 0, udpcom_.getSocketInfo(1).sendBufferSize);
    // std::memset(buffer_tx_right_, 0, buffer_size_tx_right_);  // Clear the buffer
}

template <typename sendData>
void SenderThreadMulti3<sendData>::clearBufferBuzzWireLeft() noexcept {
    std::memset(udpcom_.getSocketInfo(2).sendBuffer, 0, udpcom_.getSocketInfo(2).sendBufferSize);
}

template <typename sendData>
void SenderThreadMulti3<sendData>::clearBufferBuzzWireRight() noexcept {
    std::memset(udpcom_.getSocketInfo(3).sendBuffer, 0, udpcom_.getSocketInfo(3).sendBufferSize);
}


template <typename sendData>
void SenderThreadMulti3<sendData>::storeDatavec() {
    logging_data_ = {multi_control3_.getLeftAngle(), multi_control3_.getLeftTorque(), multi_control3_.getRightAngle(),
                     multi_control3_.getRightTorque()};

}

template <typename sendData>
void SenderThreadMulti3<sendData>::updatePacket() {

    multi_control3_.calc();  // execute business logic and do actual motor_control calculation
        // Calculate statistics
    statistic_thread_.calcStatistic();  // Call calcStatistic from StatisticThread

    auto time_point = std::chrono::steady_clock::now();
    auto duration = time_point.time_since_epoch();
    uint64_t time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();



    packet_tx_left_ = {time_stamp,  (float)-multi_control3_.getLeftTorque()};
    packet_tx_right_ = {time_stamp, (float)-multi_control3_.getRightTorque()};
    if (is_buzzwire) {
        float torque = -multi_control3_.getBuzzWireLeftTorque();
        torque = torque > MAX_BUZZWIRE_TORQUE ? MAX_BUZZWIRE_TORQUE : torque < -MAX_BUZZWIRE_TORQUE ? -MAX_BUZZWIRE_TORQUE : torque; 
        packet_tx_buzzwire_left_ = {time_stamp, torque};

        torque = -multi_control3_.getBuzzWireRightTorque();
        torque = torque > MAX_BUZZWIRE_TORQUE ? MAX_BUZZWIRE_TORQUE : torque < -MAX_BUZZWIRE_TORQUE ? -MAX_BUZZWIRE_TORQUE : torque; 
        packet_tx_buzzwire_right_ = {time_stamp, torque};
    }
}

template <typename sendData>
void SenderThreadMulti3<sendData>::serializeAndSend() {

   // if (multi_control3_.getLeftState().getTimeCount() % 2 != 0) {
    
        Serializer<sendData>::serialize(packet_tx_left_, udpcom_.getSocketInfo(0).sendBuffer,
                                        udpcom_.getSocketInfo(0).sendBufferSize);
        udpcom_.sendToSocket(0);


        Serializer<sendData>::serialize(packet_tx_right_, udpcom_.getSocketInfo(1).sendBuffer,
                                        udpcom_.getSocketInfo(1).sendBufferSize);
        //std::this_thread::sleep_for(std::chrono::microseconds(30)); 
        udpcom_.sendToSocket(1);
    //}

    //if (is_buzzwire && multi_control3_.getLeftState().getTimeCount() % 2 == 0) {
    if (is_buzzwire) {

        Serializer<sendData>::serialize(packet_tx_buzzwire_left_, udpcom_.getSocketInfo(2).sendBuffer,
                                        udpcom_.getSocketInfo(2).sendBufferSize);
        //std::this_thread::sleep_for(std::chrono::microseconds(30)); 
        udpcom_.sendToSocket(2);


        Serializer<sendData>::serialize(packet_tx_buzzwire_right_, udpcom_.getSocketInfo(3).sendBuffer,
                                        udpcom_.getSocketInfo(3).sendBufferSize);
        //std::this_thread::sleep_for(std::chrono::microseconds(30)); 
        udpcom_.sendToSocket(3);
    }


}


template <typename sendData>
void SenderThreadMulti3<sendData>::loggingAndInfo() {
    if (multi_control3_.getLeftState().getTimeCount() % 10000 == 0) {
        LOG(multi_control3_.getLeftState().getTimeCount() << " torque left: " << multi_control3_.getLeftTorque()
                                                          << " torque right: " << multi_control3_.getRightTorque());
    }
    if (multi_control3_.getLeftState().getTimeCount() % 1000 == 0) {
        LOG(multi_control3_.getLeftState().getTimeCount()
            << " angle: " << multi_control3_.getLeftState().getActualAngle());
    }
}
