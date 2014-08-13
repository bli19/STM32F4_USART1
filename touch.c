#include "stm32f4xx.h"
#include "Delay.h"
#include "stm32f4xx_gpio.h"
#include "stm322xg_eval_lcd.h"
#include "stm32f4xx_spi.h"
#include "stdio.h"
#include "touch.h"

s32 RawTouch_x,RawTouch_y,RawTouch_z1,RawTouch_z2;
s32 FilterTouch_x,FilterTouch_y;
s32 Touch_x,Touch_y,Touch_z;
//s32 Touch_z_threshold = 9000;	// ~12000..6500 for stylus, down to 3000 for finger
volatile s32 A,B,C,D,E,F,k;

#define BUFFER_LENGTH 3
s32 buffer_x[BUFFER_LENGTH];
s32 buffer_y[BUFFER_LENGTH];
s32 FilterTouch_x_old, FilterTouch_y_old;
s32 Touch_x_old, Touch_y_old;
s32 temp;



void Touch_Init(void)
{
	//Touch_Calibrate data
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	A = 402336;
	B = -7584;
	C = -1517124408;
	D = -5376;
	E = 536960;
	F = -2098038304;
	k = -5859309;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC, ENABLE);



	//PA5 SCK, PA6 MISO, PA7 MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, 5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, 6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, 7, GPIO_AF_SPI1);

	// PA4 CS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits (GPIOA, GPIO_Pin_4);

	// PC7 PENIRQ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//SPI1
	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //MIN 8
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

	//EXTI

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI Line4 to PC4 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);

	/* Configure EXTI Line4 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	//EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line4 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);
}


void Touch_RawUpdate (void)
{
	GPIO_ResetBits (GPIOA, GPIO_Pin_4);//cs

	SPI1->DR = 0x9000;	//y
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY));
	RawTouch_y = (SPI1->DR) << 5;

	SPI1->DR = 0xd000;	//x
	//Delay_us(100);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY));
	temp = (SPI1->DR);
	RawTouch_y |= temp >> 11;
	RawTouch_x = (temp&0x7f) << 5;

	SPI1->DR = 0x0;	//
	//SPI1->DR = 0b1011000000000000;	//z1
	//Delay_us(100);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY));
	temp = (SPI1->DR);
	RawTouch_x |= temp >> 11;
/*
	RawTouch_z1 = (temp&0b1111111) << 5;

	SPI1->DR = 0b1100000000000000;	//z2
	Delay_us(15*2);
	temp = (SPI1->DR);
	RawTouch_z1 |= temp >> 11;
	RawTouch_z2 = (temp&0b1111111) << 5;

	SPI1->DR = 0b0000000000000000;
	Delay_us(15*2);
	RawTouch_z2 |= (SPI1->DR) >> 11;
*/
	GPIO_SetBits (GPIOA, GPIO_Pin_4);


}

void Touch_Update (void)
{
	Touch_z = (RawTouch_x * RawTouch_z2)/RawTouch_z1 - RawTouch_x;
	//if(Touch_z < Touch_z_threshold)
	{
		Touch_x = ((A * RawTouch_x) + (B * RawTouch_y) + C)/k;
		Touch_y = ((D * RawTouch_x) + (E * RawTouch_y) + F)/k;
	}

}


s32 Touch_DifFilterUpdate(s32 input_x, s32 input_y)
{
    u32 i;
    s32 acc = 0;
    s32 dif = 1;

    for (i = 0; i < BUFFER_LENGTH; i++)
    {
    	if( ((input_x - buffer_x[i]) <= dif) && ((input_y - buffer_y[i]) <= dif) && ((buffer_x[i] - input_x) < dif) && ((buffer_y[i] - input_y) < dif) )
    	{
    		acc++;
    	}
    }

    buffer_y[0] = input_y;
    buffer_x[0] = input_x;


    /* 蔓躅?*/
    FilterTouch_x = input_x;
    FilterTouch_y = input_y;

    /* 鸯妁噱?玎溴疰囗 耔沩嚯 */
    for (i = BUFFER_LENGTH; i > 0; i--)
    {
    	buffer_y[i] = buffer_y[i - 1];
        buffer_x[i] = buffer_x[i - 1];
    }

    return acc;
}


void Touch_UpdateAll (void)
{
	s32 acc;
	do
	{
		FilterTouch_x_old = FilterTouch_x;
		FilterTouch_y_old = FilterTouch_y;
		Touch_x_old = Touch_x;
		Touch_y_old = Touch_y;

		Touch_RawUpdate();
		Touch_Update();

		acc = Touch_DifFilterUpdate(Touch_x,Touch_y);
	}while(acc <=(BUFFER_LENGTH-1));
}

void Touch_Calibrate (void)
{
	u32 x1,x2,x0,y1,y2,y0,x1d,x2d,x0d,y1d,y2d,y0d;

	//X = C + K*Q
	LCD_Clear(0);

	x0d=24;
	y0d=32;
	x1d=120;
	y1d=288;
	x2d=216;
	y2d=160;


	LCD_DrawCircle(x0d,y0d,5,GREEN);
	while(Touch_released);
	while(!(GPIOC->IDR & GPIO_Pin_4))
	{
		Touch_RawUpdate();
		//Touch_FilterUpdate();
		x0 = RawTouch_x;
		y0 = RawTouch_y;
		Delay_Ms(10);
	}

	Delay_Ms(100);
	LCD_Clear(0);

	LCD_DrawCircle(x1d,y1d,5,GREEN);
	while(Touch_released);
	while(!(GPIOC->IDR & GPIO_Pin_4))
	{
		Touch_RawUpdate();
		//Touch_FilterUpdate();
		x1 = RawTouch_x;
		y1 = RawTouch_y;
		Delay_Ms(10);
	}

	Delay_Ms(100);
	LCD_Clear(0);


	LCD_DrawCircle(x2d,y2d,5,GREEN);
	while(Touch_released);
	while(!(GPIOC->IDR & GPIO_Pin_4))
	{
		Touch_RawUpdate();
		//Touch_FilterUpdate();
		x2 = RawTouch_x;
		y2 = RawTouch_y;
		Delay_Ms(10);
	}


	Delay_Ms(100);
	//LCD_Clear(0);

	k = (x0-x2)*(y1-y2)-(x1-x2)*(y0-y2);
	A = (x0d-x2d)*(y1-y2)-(x1d-x2d)*(y0-y2);
	B = (x0-x2)*(x1d-x2d)-(x0d-x2d)*(x1-x2);
	C = y0*(x2*x1d - x1*x2d)+y1*(x0*x2d-x2*x0d)+y2*(x1*x0d-x0*x1d);
	D = (y0d-y2d)*(y1-y2)-(y1d-y2d)*(y0-y2);
	E = (x0-x2)*(y1d-y2d)-(y0d-y2d)*(x1-x2);
	F = y0*(x2*y1d - x1*y2d)+y1*(x0*y2d-x2*y0d)+y2*(x1*y0d-x0*y1d);

}

void EXTI9_5_IRQHandler(void)
 {

	if (EXTI_GetITStatus(EXTI_Line7) == SET)
	{
		/* Clear the EXTI line 4 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line7);

		if(Touch_pressed)
		{
			//asm("nop");
		}
		else
		{
			//asm("nop");
		}
	}

}


