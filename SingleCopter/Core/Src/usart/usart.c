/*
 * usart.c
 *
 *  Created on: Jun 1, 2025
 *      Author: nicor
 */


#include "usart/usart.h"
#include "main.h"
#include <string.h>  // para memcpy

#define TRANSMITED_BYTES 9*4
extern uint8_t rx_buffer[TRANSMITED_BYTES];
extern float received_values[9];


typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
} PIDState;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART2)
        {
            // Immediately prepare next reception
            HAL_UART_Receive_IT(huart, rx_buffer, TRANSMITED_BYTES);
            // Convert raw bytes into floats
			memcpy(received_values, rx_buffer, TRANSMITED_BYTES);
        }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    __NOP();
}
