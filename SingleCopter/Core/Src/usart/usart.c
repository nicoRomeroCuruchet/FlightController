/*
 * usart.c
 *
 *  Created on: Jun 1, 2025
 *      Author: nicor
 *
 * Protocolo UART (44 bytes):
 *   [0..35]  : 9 floats (Kp/Ki/Kd para roll, pitch, yaw)
 *   [36..39] : 1 float comp_filter_beta (peso del DMP en el filtro complementario)
 *   [40..43] : CRC32 sobre los 40 primeros bytes
 *
 * Aplica los nuevos PIDs sólo si el quad está disarmed (start != 0).
 * En vuelo se ignoran los paquetes.
 */


#include "usart/usart.h"
#include "main.h"
#include <string.h>

extern uint8_t rx_buffer[TRANSMITED_BYTES];
extern float   received_values[NUM_FLOATS_RX];
extern PIDController pid_roll, pid_pitch, pid_yaw;
extern uint8_t start;   // 0 = armed, 1 = disarmed (definido en main.c)
extern float comp_filter_beta;   // peso del DMP en el complementario, 0..1


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
    if (huart->Instance != USART2) return;

    // Re-armar la recepción cuanto antes
    HAL_UART_Receive_IT(huart, rx_buffer, TRANSMITED_BYTES);

    // Sólo aplicar cambios si el quad está disarmed (start != 0).
    // En vuelo (start == 0) ignoramos el paquete para no tocar gains ni
    // resetear integrales con el motor activo.
    if (start == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        return;
    }

    // Copiar solo los 10 floats al array de trabajo (no el CRC)
    memcpy(received_values, rx_buffer, sizeof(received_values));

    uint32_t check_sum_c = crc32_update(0xFFFFFFFF, rx_buffer, CRC_OFFSET);
    uint32_t check_sum_r;
    memcpy(&check_sum_r, &rx_buffer[CRC_OFFSET], 4);

    if (check_sum_r != check_sum_c) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        return;
    }

    initializePID(&pid_roll,  received_values[0], received_values[1], received_values[2],
                  SAMPLE_TIME_S,
                  PID_LIM_MIN_INT_ROLL, PID_LIM_MAX_INT_ROLL,
                  PID_LIM_MIN_ROLL,     PID_LIM_MAX_ROLL);

    initializePID(&pid_pitch, received_values[3], received_values[4], received_values[5],
                  SAMPLE_TIME_S,
                  PID_LIM_MIN_INT_PITCH, PID_LIM_MAX_INT_PITCH,
                  PID_LIM_MIN_PITCH,     PID_LIM_MAX_PITCH);

    initializePID(&pid_yaw,   received_values[6], received_values[7], received_values[8],
                  SAMPLE_TIME_S,
                  PID_LIM_MIN_INT_YAW, PID_LIM_MAX_INT_YAW,
                  PID_LIM_MIN_YAW,     PID_LIM_MAX_YAW);

    /* Bound check del beta del filtro complementario antes de aplicarlo.
     * beta=0 → puro gyro (drift sin corregir). beta=1 → puro DMP (ruido total).
     * Rango razonable: [0.001, 0.5]. */
    float beta = received_values[9];
    if (beta >= 0.001f && beta <= 0.5f) {
        comp_filter_beta = beta;
    }

    resetPID(&pid_roll);
    resetPID(&pid_pitch);
    resetPID(&pid_yaw);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    __NOP();
}

/* Si la UART entra en estado de error (framing/overrun/noise/parity), HAL
 * detiene la recepción y nunca más dispara HAL_UART_RxCpltCallback.
 * Limpiamos los flags y re-armamos la recepción para que la próxima trama
 * vuelva a entrar normalmente. Sin este callback, basta un solo bit corrupto
 * para dejar la UART muda hasta el próximo reset. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    volatile uint32_t dummy = huart->Instance->DR;
    (void)dummy;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState   = HAL_UART_STATE_READY;
    huart->gState    = HAL_UART_STATE_READY;

    HAL_UART_Receive_IT(huart, rx_buffer, TRANSMITED_BYTES);

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
}
