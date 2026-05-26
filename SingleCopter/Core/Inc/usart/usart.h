#ifndef USART_CALLBACKS_H
#define USART_CALLBACKS_H

#include "stm32f4xx_hal.h"

/* Protocolo UART (44 bytes):
 *   [0..35]  : 9 floats little-endian (Kp/Ki/Kd para roll, pitch, yaw)
 *   [36..39] : 1 float comp_filter_beta (peso del DMP en el filtro complementario)
 *   [40..43] : CRC32 sobre los 40 primeros bytes
 *              poly 0xEDB88320, init+xor-out 0xFFFFFFFF (CRC-32/ISO-HDLC)
 */
#define TRANSMITED_BYTES 44
#define NUM_FLOATS_RX    10
#define CRC_OFFSET       (NUM_FLOATS_RX * 4)   // = 40

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif // USART_CALLBACKS_H
