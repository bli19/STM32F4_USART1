#include "stm32f4xx.h"
//#include "main.h"
#include "touch.h"
#include "stm322xg_eval_lcd.h"
#include "lidar.h"
#include <math.h>

lidar_data_t lidarBuf[360];
lidar_data_t lidarBufOld[360];

lidar_data_t lidarBuf[360];
lidar_data_t lidarBufOld[360];
u32 lidarSpeed;
u8 lidarScale;	//larger = farther

u32 rotation = 270;

s32 sin_table[720];
s32 cos_table[720];

//wallsearch
u32 point_stack[50];//TODO think of size
u32 ptr;
u32 start;
float line_treshold = 0;
u32 line_treshold_k = 20;
line_t lineBuf[50];
line_t lineBufOld[50];
u32 lineIndex;
u8 stringBuf[40];


extern uint32_t fps_time;
extern uint32_t fps_update;

void StartLidarDemo(void)
{

	u32 i;
	u32 j;
	static u8 gridnum;
	lidarScale = 25;
	gridnum = 3;

	while(1)
	{
		fps_time = 0;

		//switch lidarScale
		if(Touch_pressed)
		{
			switch(lidarScale)//8916 874 07 98
			{
				case 13:
				{
					lidarScale = 25;
					gridnum = 3;
					break;
				}
				case 25:
				{
					lidarScale = 50;
					gridnum = 6;
					break;
				}
				case 50:
				{
					lidarScale = 13;
					gridnum = 1;
					break;
				}
			}
			LCD_Clear(BLACK);
			Delay_Ms(200);
		}
		//draw onscreen info
		LCD_DrawCircle(120,160,3,YELLOW);

		LCD_DisplayStringLine(2," # AN DIHN LIDAR DEMO");

		//lidar motor spinning rate in 1/10 of Hz
		sprintf(stringBuf," # SPIN %.2u", lidarSpeed);
		LCD_DisplayStringLine(220,stringBuf);



		//draw grid circles

		for(j=1;j<=gridnum;j++)
		{
			LCD_DrawCircle(120,160,j*1000/lidarScale,RGB(0x33,0x33,0x33));
		}

		//draw point cloud
		for(i=0;i<360;i++)
		{
			if((lidarBuf[i].dist != lidarBufOld[i].dist))
			{
				PutPixel(120-(lidarBufOld[i].x / lidarScale),160+(lidarBufOld[i].y / lidarScale),BLACK);
				//if(!lidarBuf[i].error)
				{

					if(!lidarBuf[i].warning)
					{
						PutPixel(120-(lidarBuf[i].x / lidarScale),160+(lidarBuf[i].y / lidarScale),GREEN);
					}
					else
					{
						PutPixel(120-(lidarBuf[i].x / lidarScale),160+(lidarBuf[i].y / lidarScale),YELLOW);
					}

					lidarBufOld[i].dist = lidarBuf[i].dist;
					lidarBufOld[i].x = lidarBuf[i].x;
					lidarBufOld[i].y = lidarBuf[i].y;
				}
			}
		}

		SearchWalls();
	//	DrawWalls();

		//display refresh rate, in frames per second
		if (fps_update > 20000)
		{
			sprintf(stringBuf, " # FPS %.3u", 100000/fps_time);
			LCD_DisplayStringLine(207, stringBuf);
			fps_update = 0;
		}
		//LCD_Clear(BLACK);
	}
}


void SearchWalls(void)
{
	u32 a, dist_max = 0 , a_max = 0;
	float d, d_max;
	s32 i=0, i_max, num_good, num_bad;

	lineIndex = 0;
	ptr = 0;


	// init first 3 endpoints
	point_stack[ptr] = 359;
	while(lidarBuf[point_stack[ptr]].error)
	{
		point_stack[ptr]--;
		if(point_stack[ptr] == 0) return;
	}

	
	for(a = 90; a < 180; a++)
	{
		if((!lidarBuf[a].error) && (lidarBuf[a].dist > dist_max))
		{
			dist_max = lidarBuf[a].dist;
			a_max = a;
		}
	}
	point_stack[++ptr] = a_max;
	point_stack[++ptr] = 0;
	while(lidarBuf[point_stack[ptr]].error)
	{
		point_stack[ptr]++;
		if(point_stack[ptr] == point_stack[ptr-1]) return;
	}


	// Split!
	while(ptr)
	{
		// find the line from start point to first end point
		line_t line;
		TwoPointsLine(point_stack[ptr], point_stack[ptr - 1], &line);


		// find point with max distance from the line
		d = 0;
		d_max = 0;
		i_max = 0;
		num_good = 0;
		num_bad = 0; // counter for sequental missing points

		for(i = point_stack[ptr]+1; i < point_stack[ptr-1]; i++)
		{
			if(!lidarBuf[i].error)
			{
				num_good++;
				num_bad = 0;
				d = line.A * lidarBuf[i].x + line.B * lidarBuf[i].y + line.C;
				if (d < 0) d=(-1)*d;
				if (d > d_max)
				{
					d_max = d;
					i_max = i;
				}
			}
		}

		//compare d_max with treshold
		if(d_max > line_treshold + (lidarBuf[i_max].dist / line_treshold_k))
		{
			// add new segment end point
			point_stack[ptr+1] = point_stack[ptr];
			point_stack[ptr] = i_max;
			ptr++;
		}
		else
		{

			if(num_good > 7)
			{
				// least square fit line ?
				TwoPointsLine_LSQF(point_stack[ptr], point_stack[ptr - 1], &line);
				// save the line
				line.Start = point_stack[ptr];
				line.StartX = lidarBuf[line.Start].x;
				line.StartY = lidarBuf[line.Start].y;
				line.End = point_stack[ptr-1];
				line.EndX = lidarBuf[line.End].x;
				line.EndY = lidarBuf[line.End].y;

				lineBuf[lineIndex++] = line;
			}

			// renew the start point
			ptr--;
		}
	}
	// Merge!

//	for(i = 0; i < lineIndex; i++)
//	{
//
//	}

}

void DrawWalls(void)
{
	u32 i;
	static u32 lineIndexOld;

	if(lineIndex == 0) return;


	for(i = 0; i < lineIndexOld; i++)
	{
		LCD_DrawUniLine(120-lineBufOld[i].StartX/lidarScale,
						160+lineBufOld[i].StartY/lidarScale,
						120-lineBufOld[i].EndX/lidarScale,
						160+lineBufOld[i].EndY/lidarScale, BLACK);
		LCD_DrawCircle(120-lineBufOld[i].StartX/lidarScale,
						160+lineBufOld[i].StartY/lidarScale, 3, BLACK);
		LCD_DrawCircle(120-lineBufOld[i].EndX/lidarScale,
						160+lineBufOld[i].EndY/lidarScale, 3, BLACK);
	}

	for(i = 0; i < lineIndex; i++)
	{
//		line_t line2 = lineBuf[i], line;
//		TwoPointsLine_LSQF(line2.Start, line.End, &line);
//		// normalize line parameters
//		float norm = sqrtf(line.A * line.A + line.B * line.B);
//		line.A = line.A / norm;
//		line.B = line.B / norm;
//		line.C = line.C / norm;

		LCD_DrawUniLine(120-lineBuf[i].StartX/lidarScale,
						160+lineBuf[i].StartY/lidarScale,
						120-lineBuf[i].EndX/lidarScale,
						160+lineBuf[i].EndY/lidarScale, CYAN);
		LCD_DrawCircle(120-lineBuf[i].StartX/lidarScale,
						160+lineBuf[i].StartY/lidarScale, 3, MAGENTA);
		LCD_DrawCircle(120-lineBuf[i].EndX/lidarScale,
						160+lineBuf[i].EndY/lidarScale, 3, MAGENTA);

		lineBufOld[i] = lineBuf[i];
	}
	lineIndexOld = lineIndex;
}

void TwoPointsLine (u32 st, u32 end, line_t* Line)
{
	float norm;
	float sx = (lidarBuf[st].x + lidarBuf[end].x);
	float sy = (lidarBuf[st].y + lidarBuf[end].y);
	float sx2 = (lidarBuf[st].x*lidarBuf[st].x + lidarBuf[end].x*lidarBuf[end].x);
	float sy2 = (lidarBuf[st].y*lidarBuf[st].y + lidarBuf[end].y*lidarBuf[end].y);
	float sxy = (lidarBuf[st].x*lidarBuf[st].y + lidarBuf[end].x*lidarBuf[end].y);

	Line->A = sx * sy2 - sy * sxy;
	Line->B = sy * sx2 - sx * sxy;
	Line->C = sxy * sxy - sx2 * sy2;

	// normalize line parameters
	norm = sqrtf(Line->A * Line->A + Line->B * Line->B);
	Line->A = Line->A / norm;
	Line->B = Line->B / norm;
	Line->C = Line->C / norm;
}

/****
最小二乘法来描述线段
****/
void TwoPointsLine_LSQF (u32 st, u32 end, line_t* Line)
{
	float sx=0, sy=0, sx2=0, sy2=0, sxy=0;
	float norm;
	u32 i;
	for(i = st + 1; i < end; i++)
	{
		if(!lidarBuf[i].error)
		{
			sx += lidarBuf[i].x;
			sy += lidarBuf[i].y;
			sx2 += lidarBuf[i].x * lidarBuf[i].x;
			sy2 += lidarBuf[i].y * lidarBuf[i].y;
			sxy += lidarBuf[i].x * lidarBuf[i].y;
		}
	}
	sx += lidarBuf[end].x;
	sy += lidarBuf[end].y;
	sx2 += lidarBuf[end].x * lidarBuf[end].x;
	sy2 += lidarBuf[end].y * lidarBuf[end].y;
	sxy += lidarBuf[end].x * lidarBuf[end].y;

	Line->A = sx * sy2 - sy * sxy;
	Line->B = sy * sx2 - sx * sxy;
	Line->C = sxy * sxy - sx2 * sy2;

	// normalize line parameters
	norm = sqrtf(Line->A * Line->A + Line->B * Line->B);
	Line->A = Line->A / norm;
	Line->B = Line->B / norm;
	Line->C = Line->C / norm;
}

void TrigTable_Init(void)
{
	u32 i;
	for(i=0;i<720;i++)
	{
		sin_table[i] = (s32) (1000 * sinf(((float)i)/180*M_PI));
		cos_table[i] = (s32) (1000 * cosf(((float)i)/180*M_PI));
	}
}

void PutToLidarBuffer(u8 c)
{
	static u8 packet_count = 0;
	static u8 tempBuf[23];
	u32 angle;

	if(packet_count)
	{
		tempBuf[packet_count++] = c;

		if(packet_count == 22)
		{
			lidarSpeed = ((((u32)tempBuf[3]) * 0x100) + (u32)tempBuf[2])/(64*6);//hz

			angle = ((tempBuf[1]-0xA0)*4);

			lidarBuf[angle].warning = (tempBuf[5] & 0x40) >> 6;
			lidarBuf[angle].error = (tempBuf[5] & 0x80) >> 7;
			if (lidarBuf[angle].error) lidarBuf[angle].dist = 0;
			else lidarBuf[angle].dist = ((tempBuf[5] & 0x3f)*0x100) + tempBuf[4];
			lidarBuf[angle].brightness = ((tempBuf[7])*0x100) + tempBuf[6];

			lidarBuf[angle].x = lidarBuf[angle].dist * cos_table[angle+rotation] / 1000;
			lidarBuf[angle].y = lidarBuf[angle].dist * sin_table[angle+rotation] / 1000;

			lidarBuf[angle+1].warning = (tempBuf[9] & 0x40) >> 6;
			lidarBuf[angle+1].error = (tempBuf[9] & 0x80) >> 7;
			if (lidarBuf[angle+1].error) lidarBuf[angle+1].dist = 0;
			else lidarBuf[angle+1].dist = ((tempBuf[9] & 0x3f)*0x100) + tempBuf[8];
			lidarBuf[angle+1].brightness = ((tempBuf[11])*0x100) + tempBuf[10];

			lidarBuf[angle+1].x = lidarBuf[angle+1].dist * cos_table[angle+1+rotation] / 1000;
			lidarBuf[angle+1].y = lidarBuf[angle+1].dist * sin_table[angle+1+rotation] / 1000;

			lidarBuf[angle+2].warning = (tempBuf[13] & 0x40) >> 6;
			lidarBuf[angle+2].error = (tempBuf[13] & 0x80) >> 7;
			if (lidarBuf[angle+2].error) lidarBuf[angle+2].dist = 0;
			else lidarBuf[angle+2].dist = ((tempBuf[13] & 0x3f)*0x100) + tempBuf[12];
			lidarBuf[angle+2].brightness = ((tempBuf[15])*0x100) + tempBuf[14];

			lidarBuf[angle+2].x = lidarBuf[angle+2].dist * cos_table[angle+2+rotation] / 1000;
			lidarBuf[angle+2].y = lidarBuf[angle+2].dist * sin_table[angle+2+rotation] / 1000;

			lidarBuf[angle+3].warning = (tempBuf[17] & 0x40) >> 6;
			lidarBuf[angle+3].error = (tempBuf[17] & 0x80) >> 7;
			if (lidarBuf[angle+3].error) lidarBuf[angle+3].dist = 0;
			else lidarBuf[angle+3].dist = ((tempBuf[17] & 0x3f)*0x100) + tempBuf[16];
			lidarBuf[angle+3].brightness = ((tempBuf[19])*0x100) + tempBuf[18];

			lidarBuf[angle+3].x = lidarBuf[angle+3].dist * cos_table[angle+3+rotation] / 1000;
			lidarBuf[angle+3].y = lidarBuf[angle+3].dist * sin_table[angle+3+rotation] / 1000;

			packet_count = 0;
		}
	}
	else if(c == 0xFA)
	{
		packet_count = 1;	// start collecting the packet
	}
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		u8 c;

		c = USART_ReceiveData(USART1);

		PutToLidarBuffer(c);
	}

	if (USART_GetITStatus(USART1, USART_IT_ERR) == SET)
	{
		LCD_DisplayStringLine(60,"Uart Error");
	}
}




void Uart_Init(void)
{
	//RCC init
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// GPIO PB7 Init
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //RX

	// USART properties
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl
			= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	/* Configure the USART1 */
	USART_OverSampling8Cmd(USART1, ENABLE);
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	/* Configure USART interrupts */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART2 */
	USART_Cmd(USART1, ENABLE);
}



