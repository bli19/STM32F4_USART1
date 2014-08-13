#ifndef _LIDAR_H_
#define _LIDAR_H_
#include "stm32f4xx.h"

#define M_PI		3.14159265358979323846

typedef struct line_s
{
	float A;
	float B;
	float C;
	u32 Start;
	s32 StartX;
	s32 StartY;
	u32 End;
	s32 EndX;
	s32 EndY;
}line_t;


typedef struct lid_dat
{
	s32 dist;	// in mm
	u32 brightness;
	s32 x;
	s32 y;
	u8 warning;
	u8 error;
}lidar_data_t;


void Uart_Init(void);
void StartLidarDemo(void);
void SearchWalls(void);
void DrawWalls(void);
void TwoPointsLine (u32 st, u32 end, line_t* Line);
void TwoPointsLine_LSQF (u32 st, u32 end, line_t* Line);
void TrigTable_Init(void);


#endif
