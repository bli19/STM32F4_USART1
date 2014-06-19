 #include "stm32f4xx_usart.h"
#include "misc.h" 
void NVIC_Config(void);

void USART_Config(void);

void delay(volatile unsigned int timeCount);

void delay_second(void);
	

/**********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
**********************************************************************************************************/
int main(void)
{
	// bsp_Init();

 
	NVIC_Config();
    USART_Config();
    while(1)
    {
        while(RESET == USART_GetFlagStatus(USART1,USART_FLAG_TXE));
        USART_SendData(USART1,'b');
        while(RESET == USART_GetFlagStatus(USART1,USART_FLAG_TXE));
        USART_SendData(USART1,'a');
			 //bsp_LedToggle(1);
        delay_second();
    }
}

/**********************************************************************************************************/
void NVIC_Config()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannel  = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);
}

void USART_Gpio_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB  , ENABLE);
    
    //PB6->TX  PB7->Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
}

void USART_Config(void)
{
	  USART_InitTypeDef USART_InitStructure; 
    USART_Gpio_Config();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1,&USART_InitStructure);   
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    
    USART_Cmd(USART1,ENABLE);
    
}

void USART1_IRQHandler(void)
{
    char c;
    if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
    {
        c = USART_ReceiveData(USART1);
        USART_SendData(USART1,c);
    }
        //while(1);
}

void delay(volatile unsigned int timeCount)
{
    while(timeCount --);
}

void delay_second(void)
{
    delay(1606596);
}










