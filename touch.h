#ifndef __TOUCH_H
#define __TOUCH_H
#include "stm32f4xx.h"

/* Private macro */


#define Touch_released (GPIOC->IDR & GPIO_Pin_7)
#define Touch_pressed (!(GPIOC->IDR & GPIO_Pin_7))

/* function prototypes */
void Touch_Init(void);
void Touch_RawUpdate (void);
void Touch_Update (void);
void Touch_Calibrate (void);
void Touch_FilterUpdate (void);
void Touch_UpdateAll (void);

s32 Touch_DifFilterUpdate(s32 input_x, s32 input_y);
#endif
