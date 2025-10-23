/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <zephyr/drivers/can.h>

#define CAN_CFG_BUSTYPE CAN_MODE_NORMAL // CAN_MODE_NORMAL or CAN_MODE_FD


void can_send_frame(uint32_t frame_id, const uint8_t *data, uint8_t dlc);

extern enum can_state zone_can_state;

extern int can_Init(void);
extern void can_Start(void);
extern uint8_t can_isRunning(void);




#define CAN_STATE_POLL_THREAD_STACK_SIZE 1024
#define CAN_STATE_POLL_THREAD_PRIORITY 5

#define CAN_RX_THREAD_STACK_SIZE 4096
#define CAN_RX_THREAD_PRIORITY -2
#define MAX_CAN_DLC 8

#endif /* CAN_H */
