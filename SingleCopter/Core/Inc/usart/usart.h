#ifndef USART_CALLBACKS_H
#define USART_CALLBACKS_H

#include "stm32f4xx_hal.h"

/* Protocolo UART (40 bytes):
 *   [0..35]  : 9 floats little-endian (Kp/Ki/Kd para roll, pitch, yaw)
 *   [36..39] : CRC32 sobre los 36 primeros bytes
 *              poly 0xEDB88320, init+xor-out 0xFFFFFFFF (CRC-32/ISO-HDLC)
 */
#define TRANSMITED_BYTES 40

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif // USART_CALLBACKS_H
