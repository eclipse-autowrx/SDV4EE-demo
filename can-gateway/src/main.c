/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited. 
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "can.h"
#include "debug.h"


int main(void)
{
	
	k_sleep(K_MSEC(5000));

	if (can_Init() != 0) 
	{
		DEBUG_ERROR("Failed to initialize CAN driver");
		return -1;
	}
	else 
	{
		DEBUG_ERROR("CAN initialized");
	}

	can_Start();
	DEBUG_ERROR("CAN started");
	k_sleep(K_MSEC(5000));
	if (IS_ENABLED(CONFIG_NET_UDP)) {
		start_udp();
		DEBUG_ERROR("UDP started");
	}
	DEBUG_ERROR("Zone gateway started!");
	while (1) 
	{
		k_sleep(K_MSEC(10000));
	}

}
