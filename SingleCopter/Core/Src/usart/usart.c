/*
 * usart.c
 *
 *  Created on: Jun 1, 2025
 *      Author: nicor
 */


#include "usart/usart.h"
#include "main.h"
#include <string.h>  // para memcpy

#define TRANSMITED_BYTES 10*4
extern uint8_t rx_buffer[TRANSMITED_BYTES];
extern float received_values[10];
extern PIDController pid_roll, pid_pitch, pid_yaw;

// SAMPLE_TIME_S y PID_LIM_* definidos en PID/PID.h (compartidos con main.c)


static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL
                            :  crc >> 1;
    }
    return ~crc;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART2)
        {
            // Immediately prepare next reception
            HAL_UART_Receive_IT(huart, rx_buffer, TRANSMITED_BYTES);
            memcpy(received_values, rx_buffer, TRANSMITED_BYTES);

            uint32_t check_sum_c = crc32_update(0xFFFFFFFF, rx_buffer, TRANSMITED_BYTES-4);
            uint32_t check_sum_r;
            memcpy(&check_sum_r, &rx_buffer[36], 4);

            if (check_sum_r == check_sum_c)
            {
            	initializePID(&pid_roll, received_values[0],
            			                 received_values[1],
										 received_values[2],
										 SAMPLE_TIME_S,
										 PID_LIM_MIN_INT_ROLL, PID_LIM_MAX_INT_ROLL,
										 PID_LIM_MIN_ROLL, PID_LIM_MAX_ROLL);

            	initializePID(&pid_pitch, received_values[3],
            			                  received_values[4],
										  received_values[5],
										 SAMPLE_TIME_S,
										 PID_LIM_MIN_INT_PITCH, PID_LIM_MAX_INT_PITCH,
										 PID_LIM_MIN_PITCH, PID_LIM_MAX_PITCH);

            	initializePID(&pid_yaw, received_values[6],
            			                 received_values[7],
										 received_values[8],
										 SAMPLE_TIME_S,
										 PID_LIM_MIN_INT_YAW, PID_LIM_MAX_INT_YAW,
										 PID_LIM_MIN_YAW, PID_LIM_MAX_YAW);

            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            }
            else
            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        }

        resetPID(&pid_roll);
        resetPID(&pid_pitch);
        resetPID(&pid_yaw);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    __NOP();
}
