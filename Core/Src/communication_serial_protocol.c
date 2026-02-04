/*
 * communication_serial_protocol.c
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#include "communication_serial_protocol.h"
#include "main.h"

uint8_t PROTOCOL_RX_Buffer[PROTOCOL_RX_BUFFER_SIZE];
uint8_t PROTOCOL_RX_Stream_Data = 0;
volatile int PROTOCOL_Stream_Index = 0;
volatile uint16_t last_rx_tick = 0;

Protocol_Status_t protocol_status = FREE;
UART_HandleTypeDef *huart_instance = NULL;

const uint32_t PROTOCOL_TIMEOUT = 1000;
const uint32_t SEND_ATTEMPT_INTERVAL_MS = 500;
const uint32_t READ_SERIAL_INTERVAL_MS = 20;
const uint32_t SEND_AT_INTERVAL_MS = 50;

volatile uint32_t last_read_serial_timestamp = 0;
volatile uint32_t last_send_at_timestamp = 0;

volatile uint8_t is_connected = 0;

Command_Queue_t command_queue = {0};
Command_Context_t current_command = {0};

void SERIAL_Init(UART_HandleTypeDef *huart) {
    huart_instance = huart;

    memset(&command_queue, 0, sizeof(Command_Queue_t));
    command_queue.head = 0;
    command_queue.tail = 0;
    command_queue.count = 0;

    memset(&current_command, 0, sizeof(Command_Context_t));
    current_command.active = 0;

    memset(PROTOCOL_RX_Buffer, 0, sizeof(PROTOCOL_RX_Buffer));
    PROTOCOL_Stream_Index = 0;

    if (huart_instance != NULL) {
        HAL_UART_Receive_IT(huart_instance, &PROTOCOL_RX_Stream_Data, 1);
    }

    protocol_status = FREE;
}

uint8_t SERIAL_QueueCommand(const char *command, const char *answer,
                             uint32_t timeout, CommandCallback_t callback) {
    if (command_queue.count >= CMD_QUEUE_SIZE) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: Command queue full!\r\n");
#endif
        return 0;
    }

    if (command == NULL || strlen(command) >= MAX_CMD_LENGTH) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: Invalid command!\r\n");
#endif
        return 0;
    }

    __disable_irq();

    Command_Item_t *item = &command_queue.buffer[command_queue.head];

    snprintf(item->cmd, MAX_CMD_LENGTH, "%s\r", command);

    if (answer != NULL) {
        strncpy(item->expected_answer, answer, MAX_ANSWER_LENGTH - 1);
        item->expected_answer[MAX_ANSWER_LENGTH - 1] = '\0';
    } else {
        item->expected_answer[0] = '\0';
    }

    item->timeout = timeout;
    item->callback = callback;
    item->active = 1;

    command_queue.head = (command_queue.head + 1) % CMD_QUEUE_SIZE;
    command_queue.count++;

    __enable_irq();

    return 1;
}

uint8_t SERIAL_IsQueueFull(void) {
    return (command_queue.count >= CMD_QUEUE_SIZE);
}

uint8_t SERIAL_GetQueueCount(void) {
    return command_queue.count;
}

void SERIAL_ClearQueue(void) {
    __disable_irq();
    command_queue.head = 0;
    command_queue.tail = 0;
    command_queue.count = 0;
    memset(&command_queue.buffer, 0, sizeof(command_queue.buffer));
    __enable_irq();
}

static uint8_t SERIAL_GetNextCommand(Command_Context_t *cmd_out) {
    if (command_queue.count == 0) {
        return 0;
    }

    __disable_irq();

    Command_Item_t *item = &command_queue.buffer[command_queue.tail];

    strncpy(cmd_out->cmd, item->cmd, MAX_CMD_LENGTH - 1);
    cmd_out->cmd[MAX_CMD_LENGTH - 1] = '\0';

    strncpy(cmd_out->expected_answer, item->expected_answer, MAX_ANSWER_LENGTH - 1);
    cmd_out->expected_answer[MAX_ANSWER_LENGTH - 1] = '\0';

    cmd_out->timeout = item->timeout;
    cmd_out->callback = item->callback;
    cmd_out->start_tick = HAL_GetTick();
    cmd_out->send_attempt_tick = HAL_GetTick();
    cmd_out->result = CMD_RESULT_PENDING;
    cmd_out->active = 1;

    command_queue.tail = (command_queue.tail + 1) % CMD_QUEUE_SIZE;
    command_queue.count--;

    __enable_irq();

    return 1;
}

void SERIAL_ProcessQueue(void) {
    if (current_command.active) {
        return;
    }

    if (protocol_status != FREE) {
        return;
    }
    if (SERIAL_GetNextCommand(&current_command)) {
        SERIAL_ResetBuffers();

        HAL_StatusTypeDef status = HAL_UART_Transmit_IT(huart_instance,
                (uint8_t*) current_command.cmd, strlen(current_command.cmd));

        if (status == HAL_OK) {
            protocol_status = WAITING;
#ifdef DEBUG_SERIAL_PROTOCOL
            printf("[SERIAL PROTOCOL] Command sent: %s \r\n", current_command.cmd);
#endif
        } else {
#ifdef DEBUG_SERIAL_PROTOCOL
            printf("[SERIAL PROTOCOL] Error sending command!\r\n");
#endif

            if (current_command.callback) {
                current_command.callback(CMD_RESULT_ERROR, NULL);
            }

            current_command.active = 0;
            protocol_status = FREE;
        }
    }
}

void SERIAL_SendCommand(char command[], char answer[], uint32_t timeout,
                        CommandCallback_t callback) {
    if (huart_instance == NULL) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: UART not initialized!\r\n");
#endif
        return;
    }

    if (!SERIAL_QueueCommand(command, answer, timeout, callback)) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: Unable to add command to queue!\r\n");
#endif
        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
    }
}

void SERIAL_SendJSON(char *json_string, char answer[], uint32_t timeout,
                     CommandCallback_t callback) {
    if (json_string == NULL) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERRO: JSON string é NULL!\r\n");
#endif

        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
        return;
    }

    if (huart_instance == NULL) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: UART not initialized!\r\n");
#endif
        free(json_string);

        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
        return;
    }

    size_t json_len = strlen(json_string);
    size_t total_len = json_len + 2;

    if (total_len > MAX_CMD_LENGTH) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("ERROR: JSON too large!\r\n");
#endif
        free(json_string);

        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
        return;
    }

    char *json_with_cr = (char*)malloc(total_len);
    if (json_with_cr == NULL) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: Failed to allocate memory for JSON!\r\n");
#endif
        free(json_string);

        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
        return;
    }

    snprintf(json_with_cr, total_len, "%s\r", json_string);
    free(json_string);
    uint8_t result = SERIAL_QueueCommand(json_with_cr, answer, timeout, callback);

    free(json_with_cr);

    if (!result) {
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] ERROR: Unable to add JSON to the queue!\r\n");
#endif
        if (callback) {
            callback(CMD_RESULT_ERROR, NULL);
        }
    }
}

void SERIAL_DirectTransmit(char *cmd) {
    if (huart_instance == NULL || cmd == NULL) {
        return;
    }

    printf(">> TX direto: %s", cmd);

    HAL_UART_Transmit(huart_instance, (uint8_t*) cmd, strlen(cmd), 100);
}

void SERIAL_CheckRXCommand(void) {
    static char *last_response_received = NULL;
    static Command_State_t command_state = CMD_STATUS_WAITING;
    static uint32_t last_attempt_tick = 0;

    if (command_state == CMD_STATUS_WAITING) {
        if (strstr((char*) PROTOCOL_RX_Buffer, "\r")) {
            last_response_received = (char*) PROTOCOL_RX_Buffer;
#ifdef DEBUG_SERIAL_PROTOCOL
            printf("[SERIAL PROTOCOL] Received: %s \r\n", last_response_received);
#endif
            last_attempt_tick = 0;
            command_state = CMD_STATUS_PROCESSING;
        }

    } else if (command_state == CMD_STATUS_PROCESSING) {

        if (current_command.active && protocol_status == WAITING) {
            if (strstr(last_response_received, current_command.expected_answer)) {
                if (current_command.callback) {
                    current_command.callback(CMD_RESULT_SUCCESS, last_response_received);
                }

                current_command.result = CMD_RESULT_SUCCESS;
                current_command.active = 0;
                SERIAL_ResetBuffers();
                protocol_status = FREE;
            }
        }

        if (!current_command.active && protocol_status == FREE) {
            if (strstr(last_response_received, "AT") &&
                !strstr(last_response_received, "AT+")) {
#ifdef DEBUG_SERIAL_PROTOCOL
                printf("[SERIAL PROTOCOL] Responding: OK\r\n");
#endif
                SERIAL_DirectTransmit("OK\r");
            }
            else if (strstr(last_response_received, "AT+WHO")) {
#ifdef DEBUG_SERIAL_PROTOCOL
                printf("[SERIAL PROTOCOL] Responding: ID:ROUTE_ECU_V1\r\n");
#endif
                SERIAL_DirectTransmit("ID:ROUTE_ECU_V1\r");
            }

            SERIAL_ResetBuffers();
        }

        command_state = CMD_STATUS_WAITING;
    }
}

void SERIAL_ResetBuffers(void) {
    PROTOCOL_RX_Stream_Data = 0;
    PROTOCOL_Stream_Index = 0;

    memset(PROTOCOL_RX_Buffer, 0, sizeof(PROTOCOL_RX_Buffer));

    if (huart_instance != NULL) {
        HAL_UART_Receive_IT(huart_instance, &PROTOCOL_RX_Stream_Data, 1);
    }
}

void PROTOCOL_RX_Callback(void) {
    if (PROTOCOL_Stream_Index < PROTOCOL_RX_BUFFER_SIZE - 1) {
        PROTOCOL_RX_Buffer[PROTOCOL_Stream_Index++] = PROTOCOL_RX_Stream_Data;
        last_rx_tick = HAL_GetTick();
    } else {
        // Buffer cheio
#ifdef DEBUG_SERIAL_PROTOCOL
        printf("[SERIAL PROTOCOL] WARNING: RX buffer overflow!\r\n");
#endif
        SERIAL_ResetBuffers();
    }

    HAL_UART_Receive_IT(huart_instance, &PROTOCOL_RX_Stream_Data, 1);
}

void PROTOCOL_TX_Callback(void) {
    // Transmissão completa
    if (current_command.active) {
        protocol_status = WAITING;
    } else {
        protocol_status = FREE;
    }
}

void SERIAL_CheckConnection(Command_Result_t result) {
    if (result == CMD_RESULT_SUCCESS) {
        is_connected = 1;
    } else {
        is_connected = 0;
    }
}
