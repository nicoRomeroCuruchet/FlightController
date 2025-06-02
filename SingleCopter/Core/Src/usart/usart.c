/*
 * usart.c
 *
 *  Created on: Jun 1, 2025
 *      Author: nicor
 */


#include "usart/usart.h"
#include "main.h"
#include <string.h>  // para memcpy

#define TRANSMITED_BYTES 12

extern UART_HandleTypeDef huart2;

extern uint8_t rx_buffer[TRANSMITED_BYTES];
extern float received_values[3];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        memcpy(received_values, rx_buffer, TRANSMITED_BYTES);

        float p = received_values[0];
        float i = received_values[1];
        float d = received_values[2];

        HAL_UART_Receive_IT(huart, rx_buffer, TRANSMITED_BYTES);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    __NOP();
}
