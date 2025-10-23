/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
// General includes
#include <stdio.h>
// Zephyr stuff
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
// Project includes
#include "can.h"
#include "debug.h"
#include "udp.h"

enum can_state zone_can_state = CAN_STATE_BUS_OFF;

K_THREAD_STACK_DEFINE(can_Poll_State_Stack, CAN_STATE_POLL_THREAD_STACK_SIZE);

static struct can_bus_err_cnt can_Err_Cnt;
static struct k_work can_State_Change_Work;
static struct k_thread can_Poll_State_Thread_Data;

K_THREAD_STACK_DEFINE(can_Rx_Thread_Stack, CAN_RX_THREAD_STACK_SIZE);
static struct k_thread can_Rx_Thread_Data;

CAN_MSGQ_DEFINE(can_rx_msgq, 2);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static void can_TxCallback(const struct device *dev, int error, void *arg);

static char *can_Poll_State2Str(enum can_state state);
static void can_State_thread(void *unused1, void *unused2, void *unused3);

static void can_State_Change_Work_Handler(struct k_work *work);
static void can_State_Change_Callback(const struct device *dev, enum can_state state, struct can_bus_err_cnt err_cnt, void *user_data);

static int can_Init_ConfiguredRxFrames(void);
static void can_RxIndication(void);
static void can_Rx_Thread(void *arg1, void *arg2, void *arg3);

void can_send_frame(uint32_t frame_id, const uint8_t *data, uint8_t dlc) {
    struct can_frame frame;
    int retVal;

    // Prepare the CAN frame
    frame.flags = 0;
    frame.id = frame_id;       // Set the frame ID
    frame.dlc = dlc;           // Set the DLC (Data Length Code)
    memcpy(frame.data, data, dlc); // Copy the exact amount of data specified by DLC

    // Send the CAN frame
    retVal = can_send(can_dev, &frame, K_NO_WAIT, can_TxCallback, "");
    if (retVal != 0) {
        DEBUG_ERROR("Error sending CAN frame: %d", retVal);
    }
}



int can_Init(void)
{
    int retVal;

    DEBUG_INFO("Initializing CAN driver");

    if (!device_is_ready(can_dev))
    {
        DEBUG_ERROR("CAN: Device %s not ready.", can_dev->name);
        return -1;
    }

    retVal = can_set_mode(can_dev, CAN_CFG_BUSTYPE);
    if (retVal != 0)
    {
        DEBUG_ERROR("Error setting CAN mode [%d]", retVal);
        return -1;
    }

    retVal = can_start(can_dev);
    if (retVal != 0)
    {
        DEBUG_ERROR("Error starting CAN controller [%d]", retVal);
        return -1;
    }

    k_work_init(&can_State_Change_Work, can_State_Change_Work_Handler);

    // Thread for the Can Driver State
    if (!k_thread_create(&can_Poll_State_Thread_Data,
                         can_Poll_State_Stack,
                         K_THREAD_STACK_SIZEOF(can_Poll_State_Stack),
                         can_State_thread, NULL, NULL, NULL,
                         CAN_STATE_POLL_THREAD_PRIORITY, 0,
                         K_NO_WAIT))
    {
        DEBUG_ERROR("ERROR spawning can_State_thread");
    }

    can_set_state_change_callback(can_dev, can_State_Change_Callback, &can_State_Change_Work);

    if (-1 == can_Init_ConfiguredRxFrames())
    {
        DEBUG_ERROR("Error initializing RX frames");
        return -1;
    }

    DEBUG_INFO("Finished init.");

    return 0;
}

void can_Start(void)
{
    DEBUG_INFO("Initializing CAN");

    if (!k_thread_create(&can_Rx_Thread_Data, can_Rx_Thread_Stack,
                         K_THREAD_STACK_SIZEOF(can_Rx_Thread_Stack),
                         can_Rx_Thread, NULL, NULL, NULL,
                         CAN_RX_THREAD_PRIORITY, 0, K_NO_WAIT))
    {
        DEBUG_ERROR("ERROR spawning rx thread");
    }
}

uint8_t can_isRunning(void)
{
    return device_is_ready(can_dev);
}

static void can_TxCallback(const struct device *dev, int error, void *arg)
{
    char *sender = (char *)arg;

    ARG_UNUSED(dev);

    if (error != 0)
    {
        DEBUG_ERROR("Callback! error-code: %d\nSender: %s", error, sender);
    }
}

static void can_RxCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    DEBUG_INFO("Rx Callback msg : %d", user_data);
}

static int can_Init_ConfiguredRxFrames(void)
{
    // Filter to accept all CAN IDs ending with xxxxxxx1 last bit shall be one e.g. 0x21 , 0x51
    struct can_filter filter_id_range = {
        .flags = 0u,
        .id = 0x01,              // Base ID of the range 
        .mask = 0x01             // Match only the the last bit
    };

    // Add the filter to the CAN receiver message queue
    int filter_id = can_add_rx_filter_msgq(can_dev, &can_rx_msgq, &filter_id_range);

    // Check if the filter was added successfully
    if (filter_id < 0) {
        DEBUG_ERROR("Error adding filter for CAN ID range 0x10 to 0x99");
        return -1;
    }

    return 0;
}


static void can_Rx_Thread(void *arg1, void *arg2, void *arg3) {
    struct can_frame frame_received;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    DEBUG_INFO("Start can_Rx_Thread");

    static uint16_t steeringAngle_raw_previous = 0;

    while (1) {
        k_msgq_get(&can_rx_msgq, &frame_received, K_FOREVER);



        
        // Check for RTR frames (Remote Transmission Request), if enabled
        if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) && (frame_received.flags & CAN_FRAME_RTR) != 0U) {
            continue;
        }

        // handle only id's from 0x11 to 0x99    
        if (frame_received.id >= 0x11 && frame_received.id <= 0x99) {
            DEBUG_INFO("Received CAN message with ID: 0x%X", frame_received.id);

            // 🔹 Log raw CAN data as hex
            char hex_str[3 * MAX_CAN_DLC + 1] = {0}; // Buffer for hex representation
            char *ptr = hex_str;
            for (uint8_t i = 0; i < frame_received.dlc; i++) {
                snprintf(ptr, 4, "%02X ", frame_received.data[i]);
                ptr += 3; // Move pointer forward (2 hex chars + space)
            }
            DEBUG_INFO("CAN Data: [%s]", hex_str);  // Log raw CAN data in hex format

            // 🔹 Pass raw CAN data to UDP sender
            udp_send_can_data(frame_received.id, frame_received.data, frame_received.dlc);
        } else {
            DEBUG_WARNING("Received unexpected CAN message with ID: 0x%X", frame_received.id);
        }


    }
}


static char *can_Poll_State2Str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

static void can_State_thread(void *unused1, void *unused2, void *unused3)
{
	struct can_bus_err_cnt err_cnt = {0, 0};
	struct can_bus_err_cnt err_cnt_prev = {0, 0};
	enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;
	enum can_state state;
	int err;

	while (1) 
	{
		err = can_get_state(can_dev, &state, &err_cnt);
		zone_can_state = state;
		if (err != 0) 
		{
			DEBUG_ERROR("Failed to get CAN controller state: %d", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		if ((err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt) || (err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt) ||(state_prev != state))
		{
			err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
			err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
			state_prev = state;
			DEBUG_INFO("state: %s\nrx error count: %d\ntx error count: %d",  can_Poll_State2Str(state), err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
		} 
		else 
		{
			if (zone_can_state>0){
				DEBUG_ERROR("Could not send CAN Messages. CAN state: %s",can_Poll_State2Str(zone_can_state));
				DEBUG_ERROR("Stop CAN");
				int retVal = can_stop(can_dev);
				if (retVal != 0) 
				{
					DEBUG_ERROR("Error starting CAN controller [%d]", retVal);
					return -1;
				}
				k_sleep(K_MSEC(500));
				DEBUG_ERROR("Start CAN");
				retVal = can_start(can_dev);
				if (retVal != 0) 
				{
					DEBUG_ERROR("Error starting CAN controller [%d]", retVal);
					return -1;
				}
				k_sleep(K_MSEC(2000));
				

			}
			k_sleep(K_MSEC(100));

		}

	}
}

static void can_State_Change_Work_Handler(struct k_work *work)
{
	DEBUG_INFO("State Change ISR\nstate: %s\n"
	       "rx error count: %d\n"
	       "tx error count: %d",
		can_Poll_State2Str(zone_can_state),
		can_Err_Cnt.rx_err_cnt, can_Err_Cnt.tx_err_cnt);
}

static void can_State_Change_Callback(const struct device *dev, enum can_state state, struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct k_work *work = (struct k_work *)user_data;

	ARG_UNUSED(dev);

	zone_can_state = state;
	can_Err_Cnt = err_cnt;
	k_work_submit(work);
}


