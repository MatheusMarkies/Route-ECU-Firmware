/*
 * communication_serial_protocol.c
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#include "communication_serial_protocol.h"
#include "main.h"

#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

void PROTOCOL_RX_Callback() {
	PROTOCOL_RX_Buffer[PROTOCOL_Stream_Index++] = PROTOCOL_RX_Stream_Data;
	last_rx_tick = HAL_GetTick();
	protocol_status = RX;
	HAL_UART_Receive_IT(&huart_instance, &PROTOCOL_RX_Stream_Data, 1);
}

void PROTOCOL_TX_Callback() {
	if (protocol_status == FREE) {
		protocol_status = WAITING;
		send_tick = HAL_GetTick();
	}
}

void resetBuffers() {
	PROTOCOL_RX_Stream_Data = 0;
	PROTOCOL_Stream_Index = 0;
	memset(PROTOCOL_RX_Buffer, 0, sizeof(PROTOCOL_RX_Buffer));
}

uint8_t sendCommand(char command[], char answer[], uint32_t timeout) {
	uint8_t ATisOK = 0;
	send_tick = HAL_GetTick();

	resetBuffers();

	HAL_UART_Receive_IT(&huart_instance, &PROTOCOL_RX_Stream_Data, 1);

	uint8_t commandBuffer[200] = { 0 };
	memcpy(commandBuffer, (uint8_t*) command, strlen(command) + 1);

	printf("Sending AT Command: \r\n");
	printf(command);

	uint32_t previousTick = HAL_GetTick();
	while (!ATisOK && previousTick + timeout > HAL_GetTick()) {

		if (HAL_GetTick() - last_send_at_timestamp >= SEND_AT_INTERVAL_MS) {
			last_send_at_timestamp = HAL_GetTick();

			if (protocol_status == FREE) {
				HAL_UART_Transmit_IT(&huart_instance, commandBuffer,
						sizeof(commandBuffer));
			}

			if (protocol_status >= WAITING) {
				if (protocol_status == RX) {
					if (strstr((char*) PROTOCOL_RX_Buffer, answer)) {
						ATisOK = 1;
					}
				}
			}

		}
		//HAL_Delay(50);
	}

	protocol_status = FREE;

	if (!ATisOK) {
		DEBUG_PRINT((char*) PROTOCOL_RX_Buffer);
		DEBUG_PRINT("CMD Timeout...\r\n");
	}
	return ATisOK;
}

void directTransmit(char *cmd) {
	resetBuffers();
	HAL_UART_Transmit(&huart_instance, (uint8_t*) cmd, strlen(cmd), 1000);
	HAL_UART_Receive(&huart_instance, PROTOCOL_RX_Buffer, sizeof(PROTOCOL_RX_Buffer), 1000);

	protocol_status = FREE;
}
