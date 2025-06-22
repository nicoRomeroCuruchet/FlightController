#ifndef INC_RC_RC_H_
#define INC_RC_RC_H_

#include "stm32f4xx_hal.h"

#define TIMCLOCK   84000000
#define PRESCALAR  83
#define AUTORELOAD 20000

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_RC_RC_H_ */
