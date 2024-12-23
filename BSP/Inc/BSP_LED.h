//
// Created by CYK on 2024/11/21.
//

#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "RTOS.h"

void LED_R_On(void);
void LED_G_On(void);
void LED_R_Off(void);
void LED_G_Off(void);

#endif //BSP_LED_H
