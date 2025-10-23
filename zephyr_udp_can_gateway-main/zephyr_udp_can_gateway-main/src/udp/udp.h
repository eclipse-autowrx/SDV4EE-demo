/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#define SERVER_PORT 12345


#define STACK_SIZE 4096
#define THREAD_PRIORITY -2


#define UDP_CLIENT_TIMEOUT_MS 20 //recognize udp diconnect, if nothing received from udp after this timeout

void udp_send_can_data(uint32_t can_frame_id, const uint8_t *can_frame_data, uint8_t dlc);



