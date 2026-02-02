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

Protocol_Status_t protocol_status = FREE;

UART_HandleTypeDef *huart_instance;

const uint32_t PROTOCOL_TIMEOUT = 1 * 1000;
const uint32_t SEND_ATTEMPT_INTERVAL_MS = 500;
const uint32_t READ_SERIAL_INTERVAL_MS = 20;
const uint32_t SEND_AT_INTERVAL_MS = 50;

volatile uint32_t last_read_serial_timestamp = 0;
volatile uint32_t last_send_at_timestamp = 0;

volatile uint8_t is_connected = 0;

Command_Context_t current_command = { 0 };

void PROTOCOL_RX_Callback(void) {
	PROTOCOL_RX_Buffer[PROTOCOL_Stream_Index++] = PROTOCOL_RX_Stream_Data;

	last_rx_tick = HAL_GetTick();
	printf("s\r\n");
	HAL_UART_Receive_IT(huart_instance, &PROTOCOL_RX_Stream_Data, 1);
}

void PROTOCOL_TX_Callback(void) {
	protocol_status = WAITING;
}

void SERIAL_ResetBuffers(void) {
	PROTOCOL_RX_Stream_Data = 0;
	PROTOCOL_Stream_Index = 0;

	//protocol_status = FREE;

    if (current_command.cmd != NULL) {
        free(current_command.cmd);
        current_command.cmd = NULL;
    }

    if (current_command.expected_answer != NULL) {
        free(current_command.expected_answer);
        current_command.expected_answer = NULL;
    }

    current_command.active = 0;

	memset(PROTOCOL_RX_Buffer, 0, sizeof(PROTOCOL_RX_Buffer));
}

void SERIAL_SendCommand(char command[], char answer[], uint32_t timeout, CommandCallback_t callback) {
	if (huart_instance == NULL) {
		return;
	}

	if (protocol_status == WAITING) {
		// return;
	}

	SERIAL_ResetBuffers();

	size_t cmd_len = strlen(command) + 2;
	current_command.cmd = (char*) malloc(cmd_len * sizeof(char));
    if (current_command.cmd != NULL) {
	    snprintf(current_command.cmd, cmd_len, "%s\r", command);
    }

    if (answer != NULL) {
        size_t ans_len = strlen(answer) + 1;
        current_command.expected_answer = (char*) malloc(ans_len * sizeof(char));
        if (current_command.expected_answer != NULL) {
            strcpy(current_command.expected_answer, answer);
        }
    } else {
        current_command.expected_answer = NULL;
    }

	current_command.timeout = timeout;
	current_command.start_tick = HAL_GetTick();
	current_command.send_attempt_tick = HAL_GetTick();
	current_command.callback = callback;
	current_command.result = CMD_RESULT_PENDING;
	current_command.active = 1;

	HAL_UART_Receive_IT(huart_instance, &PROTOCOL_RX_Stream_Data, 1);

    if (current_command.cmd != NULL) {
	    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(huart_instance,
			    (uint8_t*) current_command.cmd, strlen(current_command.cmd));

        if (status == HAL_OK) {
            protocol_status = WAITING;
        } else {
            protocol_status = FREE;
            free(current_command.cmd);
            if(current_command.expected_answer) free(current_command.expected_answer);

            if (current_command.callback) {
                current_command.callback(CMD_RESULT_ERROR, NULL);
            }
        }
    }
}

void SERIAL_CheckRXCommand(void) {
	static char *last_response_received;
	static Command_State_t command_state = CMD_STATUS_WAITING;
	static uint32_t last_attempt_tick = 0;

	if (command_state == CMD_STATUS_WAITING) {
		if (strstr((char*) PROTOCOL_RX_Buffer, "\r")) {
			last_response_received = (char*) PROTOCOL_RX_Buffer;
			printf(last_response_received);
			printf("\r\n");
			last_attempt_tick = 0;
			command_state = CMD_STATUS_PROCESSING;
		} else {
			if (protocol_status == WAITING
					&& (HAL_GetTick() - current_command.send_attempt_tick)
							< current_command.timeout) {
				if (HAL_GetTick() - last_attempt_tick >= SEND_AT_INTERVAL_MS) {
					last_attempt_tick = HAL_GetTick();

					HAL_StatusTypeDef status = HAL_UART_Transmit_IT(
							huart_instance, (uint8_t*) current_command.cmd,
							strlen(current_command.cmd));
					if (status == HAL_OK) {
						protocol_status = WAITING;
					} else
						SERIAL_ResetBuffers();

				}
			} else if ((HAL_GetTick() - current_command.send_attempt_tick)
					>= current_command.timeout) {
				last_attempt_tick = 0;

				if (current_command.callback)
					current_command.callback(CMD_RESULT_TIMEOUT, NULL);
				current_command.result = CMD_RESULT_TIMEOUT;

				SERIAL_ResetBuffers();
			}
		}
	} else if (command_state == CMD_STATUS_PROCESSING) {
		if (protocol_status == WAITING
				&& strstr(last_response_received,
						current_command.expected_answer)) {
			if (current_command.callback)
				current_command.callback(CMD_RESULT_SUCCESS,
						last_response_received);
			current_command.result = CMD_RESULT_SUCCESS;

			SERIAL_ResetBuffers();
		}

        if (protocol_status == FREE) {
            if (strstr(last_response_received, "AT") &&
                !strstr(last_response_received, "AT+")) {
                printf("AT comando detectado, respondendo OK\r\n");
                SERIAL_DirectTransmit("OK\r");
            }

            else if (strstr(last_response_received, "AT+WHO")) {
                printf("AT+WHO detectado, respondendo ID\r\n");
                SERIAL_DirectTransmit("ID:ROUTE_ECU_V1\r");
            }

            SERIAL_ResetBuffers();
        }

		command_state = CMD_STATUS_WAITING;
	}

}

void SERIAL_CheckConnection(Command_Result_t result, char *response) {
	if (result == CMD_RESULT_SUCCESS)
		is_connected = 1;
	else {
		is_connected = 0;
		return;
	}
}

void SERIAL_DirectTransmit(char *cmd) {
    printf(">> TX direto: %s", cmd);
    HAL_UART_Transmit_IT(huart_instance, (uint8_t*) cmd, strlen(cmd));
    if (protocol_status != WAITING) {
        protocol_status = FREE;
    }
}
