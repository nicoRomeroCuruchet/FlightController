#ifndef USART_CALLBACKS_H
#define USART_CALLBACKS_H

#include "stm32f4xx_hal.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);

#endif // USART_CALLBACKS_H
