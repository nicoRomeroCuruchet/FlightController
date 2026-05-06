#ifndef INC_RC_RC_H_
#define INC_RC_RC_H_

#include "stm32f4xx_hal.h"

#define TIMCLOCK   84000000
#define PRESCALAR  83
#define AUTORELOAD 20000

#define FAILSAFE_MS 100   // ms sin pulso válido antes de activar failsafe

extern volatile uint32_t last_radio_update_ms;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_RC_RC_H_ */
