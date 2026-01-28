/*
 * communication_serial_protocol.c
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#include "communication_serial_protocol.h"
#include "main.h"

uint16_t send_tick = 0;

uint8_t PROTOCOL_RX_Buffer[500];
uint8_t PROTOCOL_RX_Stream_Data = 0;
int PROTOCOL_Stream_Index = 0;

uint16_t last_rx_tick = 0;

PROTOCOLStatus protocol_status = FREE;

UART_HandleTypeDef huart_instance;

const uint32_t PROTOCOL_TIMEOUT = 1 * 1000;
const uint32_t SEND_ATTEMPT_INTERVAL_MS = 500;
const uint32_t READ_SERIAL_INTERVAL_MS = 20;
const uint32_t SEND_AT_INTERVAL_MS = 50;

volatile uint32_t last_read_serial_timestamp = 0;
volatile uint32_t last_send_at_timestamp = 0;

volatile uint8_t is_connected = 0;

void PROTOCOL_RX_Callback(void) {
	PROTOCOL_RX_Buffer[PROTOCOL_Stream_Index++] = PROTOCOL_RX_Stream_Data;
	last_rx_tick = HAL_GetTick();
	protocol_status = RX;
	HAL_UART_Receive_IT(&huart_instance, &PROTOCOL_RX_Stream_Data, 1);
}

void PROTOCOL_TX_Callback(void) {
	if (protocol_status == FREE) {
		protocol_status = WAITING;
		send_tick = HAL_GetTick();
	}
}

void SERIAL_ResetBuffers(void) {
	PROTOCOL_RX_Stream_Data = 0;
	PROTOCOL_Stream_Index = 0;
	memset(PROTOCOL_RX_Buffer, 0, sizeof(PROTOCOL_RX_Buffer));
}

uint8_t SERIAL_SendCommand(char command[], char answer[], uint32_t timeout) {
	uint8_t ATisOK = 0;
	if(!is_connected)
	{
		ATisOK = 0;
		return ATisOK;
	}

	send_tick = HAL_GetTick();

	serial_ResetBuffers();

	HAL_UART_Receive_IT(&huart_instance, &PROTOCOL_RX_Stream_Data, 1);

	uint8_t commandBuffer[1024] = { 0 };
	memcpy(commandBuffer, (uint8_t*) command, strlen(command) + 1);

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
		printf("\r\n");
		printf("CMD Timeout... \r\n");
	}
	return ATisOK;
}

uint8_t SERIAL_CheckConnection(void){
	if(serial_SendCommand("AT","OK",1000))
		is_connected = 1;
	else{
		is_connected = 0;
		return 0;
	}
	return 1;
}

void SERIAL_DirectTransmit(char *cmd) {
	serial_ResetBuffers();
	HAL_UART_Transmit(&huart_instance, (uint8_t*) cmd, strlen(cmd), 1000);
	HAL_UART_Receive(&huart_instance, PROTOCOL_RX_Buffer, sizeof(PROTOCOL_RX_Buffer), 1000);

	protocol_status = FREE;
}
