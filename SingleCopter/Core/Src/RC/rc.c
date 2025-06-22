#include "RC/rc.h"

//CH1
volatile uint32_t rising_ch1 =0;
volatile uint32_t pulse_ch1 = 0;
volatile uint8_t  state_ch1 = 0;
//CH2
volatile uint32_t rising_ch2 =0;
volatile uint32_t pulse_ch2 = 0;
volatile uint8_t  state_ch2 = 0;
//CH3
volatile uint32_t rising_ch3 =0;
volatile uint32_t pulse_ch3 = 0;
volatile uint8_t  state_ch3 = 0;
//CH4
volatile uint32_t rising_ch4 =0;
volatile uint32_t pulse_ch4 = 0;
volatile uint8_t  state_ch4 = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	        uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        if (state_ch1 == 0) {
	            rising_ch1 = now;
	            state_ch1 = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	        } else {
	            // Falling edge
	            uint32_t diff = (now >= rising_ch1)
	                ? (now - rising_ch1)
	                : (now + (htim->Instance->ARR - rising_ch1));

	            float refClock = TIMCLOCK / (float)PRESCALAR;
	            uint32_t tmp = diff * (1000000.0f / refClock);
	            if (990 < tmp && tmp < 2000 ) pulse_ch1 = tmp;
	            // Reset for next rising edge
	            state_ch1 = 0;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	    }

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	    if (state_ch2 == 0) {
	        rising_ch2 = now;
	        state_ch2 = 1;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
	    } else {
	        // Falling edge
	        uint32_t diff = (now >= rising_ch2)
	            ? (now - rising_ch2)
	            : (now + (htim->Instance->ARR - rising_ch2));

	        float refClock = TIMCLOCK / (float)PRESCALAR;
	        uint32_t tmp = diff * (1000000.0f / refClock);
			if (990 < tmp && tmp < 2000 ) pulse_ch2 = tmp;

	        // Reset for next rising edge
	        state_ch2 = 0;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
	    }
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
	    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

	    if (state_ch3 == 0) {
	        rising_ch3 = now;
	        state_ch3 = 1;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	    } else {
	        // Falling edge
	        uint32_t diff = (now >= rising_ch3)
	            ? (now - rising_ch3)
	            : (now + (htim->Instance->ARR - rising_ch3));

	        float refClock = TIMCLOCK / (float)PRESCALAR;
	        uint32_t tmp = diff * (1000000.0f / refClock);
			if (990 < tmp && tmp < 2000 ) pulse_ch3 = tmp;

	        // Reset for next rising edge
	        state_ch3 = 0;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	    }
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
	    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

	    if (state_ch4 == 0) {
	        rising_ch4 = now;
	        state_ch4 = 1;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
	    } else {
	        // Falling edge
	        uint32_t diff = (now >= rising_ch4)
	            ? (now - rising_ch4)
	            : (now + (htim->Instance->ARR - rising_ch4));

	        float refClock = TIMCLOCK / (float)PRESCALAR;
	        uint32_t tmp = diff * (1000000.0f / refClock);
			if (990 < tmp && tmp < 2000 ) pulse_ch4 = tmp;

	        // Reset for next rising edge
	        state_ch4 = 0;
	        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
	    }
	}
}
