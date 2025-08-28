/**
* Copyright (c) 2025 Robert Bosch GmbH.
*
* This program and the accompanying materials are made available under the
* terms of the MIT License which is available at
* https://opensource.org/licenses/MIT.
*
* SPDX-License-Identifier: MIT
*/

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_config.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include "debug.h"
#include "udp.h"
#include "can.h"

LOG_MODULE_REGISTER(udp_server, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(udp_server_stack, STACK_SIZE);
struct k_thread udp_server_thread_data;

int udp_sock; // Global socket descriptor
bool udp_client_connected = false;
struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);

// Timers
struct k_timer client_timer;
struct k_timer can_timeout_timer;

// Latency measurement
uint16_t can_latency;
uint64_t start_ticks;

uint16_t eth_latency;
uint64_t eth_start_ticks;

// Callback for CAN timeout
void can_timeout(struct k_timer *timer_id) {
    DEBUG_WARNING("CAN state set to BUS_OFF due to timeout\n");
}

// Timer expiry function to reset client connection status
void client_timer_expiry(struct k_timer *timer_id) {
    udp_client_connected = false;
    DEBUG_WARNING("Client connection timed out\n");
}

void udp_send_can_data(uint32_t can_frame_id, const uint8_t *can_frame_data, uint8_t dlc) {
    uint8_t udp_payload[5 + MAX_CAN_DLC];
    size_t payload_size;

    uint32_t can_id_network = htonl(can_frame_id);
    memcpy(&udp_payload[0], &can_id_network, sizeof(can_id_network));
    udp_payload[4] = dlc;
    memcpy(&udp_payload[5], can_frame_data, dlc);
    payload_size = 5 + dlc;

    // Debug print
    char hex_str[3 * (5 + MAX_CAN_DLC) + 1] = {0};
    char *ptr = hex_str;
    for (size_t i = 0; i < payload_size; i++) {
        snprintf(ptr, 4, "%02X ", udp_payload[i]);
        ptr += 3;
    }
    DEBUG_INFO("Sending UDP Data: [%s]", hex_str);

    if (can_frame_id == 0x51 || can_frame_id == 0x61) {
        uint64_t end_ticks = sys_clock_cycle_get_64();
        uint64_t duration_ticks = end_ticks - start_ticks;
        uint64_t duration_us = k_cyc_to_us_floor64(duration_ticks);

        can_latency = (duration_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)duration_us;
        DEBUG_INFO("CAN Latency measured: %u us\n", can_latency);
    } else {
        can_latency = 0;
    }

    if (sendto(udp_sock, udp_payload, payload_size, 0,
               (struct sockaddr *)&client_addr, client_addr_len) < 0) {
        DEBUG_ERROR("Error sending data over UDP\n");
    }

    if (can_frame_id == 0x51 || can_frame_id == 0x61) {
        uint32_t latency_frame_id = 0x71;
        can_id_network = htonl(latency_frame_id);
        memcpy(&udp_payload[0], &can_id_network, sizeof(can_id_network));
        udp_payload[4] = MAX_CAN_DLC;
        payload_size = 5 + MAX_CAN_DLC;
        DEBUG_INFO("CAN Latency measured: %u us\n", can_latency);
        DEBUG_INFO("Eth Latency measured: %u us\n", eth_latency);
        memcpy(&udp_payload[5], &can_latency, sizeof(uint16_t));
        uint16_t eth_latency_network = htons(eth_latency);
        memcpy(&udp_payload[7], &eth_latency, sizeof(uint16_t));
        eth_start_ticks = sys_clock_cycle_get_64();     

        if (sendto(udp_sock, udp_payload, payload_size, 0,
                   (struct sockaddr *)&client_addr, client_addr_len) < 0) {
            DEBUG_ERROR("Error sending CAN latency over UDP\n");
        }
    }
}

void udp_server_thread(void *arg1, void *arg2, void *arg3) {
    struct sockaddr_in addr;
    char buffer[5 + MAX_CAN_DLC];
    int received;

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        DEBUG_ERROR("Failed to create UDP socket\n");
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udp_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        DEBUG_ERROR("Failed to bind UDP socket\n");
        close(udp_sock);
        return;
    }

    k_timer_init(&client_timer, client_timer_expiry, NULL);
    k_timer_init(&can_timeout_timer, can_timeout, NULL);

    while (1) {
        received = recvfrom(udp_sock, buffer, sizeof(buffer), 0,
                            (struct sockaddr *)&client_addr, &client_addr_len);
        if (received < 0) {
            DEBUG_WARNING("Error receiving data over UDP\n");
            continue;
        }

        if (received < 5) {
            DEBUG_WARNING("Received UDP data of unexpected size: %d (minimum: 5)\n", received);
            continue;
        }

        udp_client_connected = true;
        k_timer_start(&client_timer, K_MSEC(UDP_CLIENT_TIMEOUT_MS), K_NO_WAIT);

        uint32_t can_frame_id = sys_get_le32((const uint8_t *)buffer);
        uint8_t dlc = buffer[4];

        if (dlc > MAX_CAN_DLC) {
            DEBUG_WARNING("Invalid DLC received: %d\n", dlc);
            continue;
        }

        const uint8_t *can_frame_data = buffer + 5;

        // 🟡 Check for Ethernet roundtrip probe frame
        if (can_frame_id == 0x70) {
            uint64_t end_ticks = sys_clock_cycle_get_64();
            uint64_t duration_ticks = end_ticks - eth_start_ticks;
            uint64_t duration_us = k_cyc_to_us_floor64(duration_ticks);
            eth_latency = (duration_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)duration_us;

            DEBUG_INFO("Ethernet roundtrip latency: %u us\n", eth_latency);
            continue;  // Do not send to CAN
        }

        DEBUG_INFO("Received CAN frame ID: 0x%X, DLC: %d\n", can_frame_id, dlc);
        can_send_frame(can_frame_id, can_frame_data, dlc);

      
        start_ticks = sys_clock_cycle_get_64();     // For CAN latency

        k_timer_start(&can_timeout_timer, K_SECONDS(1), K_SECONDS(1));
    }

    close(udp_sock);
}

void start_udp(void) {
    if (IS_ENABLED(CONFIG_NET_IPV4)) {
        DEBUG_INFO("Start UDP thread\n");
        k_tid_t udp_tid = k_thread_create(&udp_server_thread_data, udp_server_stack,
                                          K_THREAD_STACK_SIZEOF(udp_server_stack),
                                          udp_server_thread,
                                          NULL, NULL, NULL,
                                          THREAD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(udp_tid, "udp4");
    } else {
        DEBUG_ERROR("Failed to start UDP thread, CONFIG_NET_IPV4 is not set\n");
    }
}
