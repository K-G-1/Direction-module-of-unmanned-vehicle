#include "timer.h"
#include "led.h"
#include "usart.h"
#include "IMU.h"
#include <stdio.h>
#include "stmflash.h"


uint16_t Time2;
u8 Txbuff[32];
 /**
  * @file   TIM2_Config
  * @brief  调用函数库，初始化定时器2的配置
  * @param  无
  * @retval 无
  */
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	//使能TIM2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//定时器定时时间T计算公式：T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK
	TIM_TimeBaseStructure.TIM_Period = (30-1);//自动重装载值20--定时时间(10*4800/48M)s 
	TIM_TimeBaseStructure.TIM_Prescaler =(4800-1);//预分频值，+1为分频系数 
	TIM_TimeBaseStructure.TIM_ClockDivision =0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	//使能TIM2中断源
	TIM_Cmd(TIM2, ENABLE);  	//使能TIMx外设						 	
}

 /**
  * @file   TIM2_IRQHandler
  * @brief  定时器2中断处理函数
  * @param  无
  * @retval 无
  */
void TIM2_IRQHandler(void)
{	
	static int times_count = 0;
    if ( TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET ) //是否发生中断
    {	
        
		Get_Attitude();
		times_count ++;
		
		if(times_count %100 == 0)
		{
			sprintf((char*)Txbuff,"put:%2.2f,%2.2f,%3.2f\r\n",(angle.roll),(angle.pitch),(angle.yaw));
			UART_PutStr(USART2,Txbuff);
//			USART_SendData(USART1, 0x02);  
//			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
//			times_count = 0;
		}
//		if(times_count %250 ==0){
//			Acc_Offest_Read();
//		}
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    //清除中断待处理位
    }		 
}

 /**
  * @file   TIM2_IRQHandler
  * @brief  产生毫秒中断
  * @param  无
  * @retval 无
  */
void TIM2_delay(uint16_t s)
{
  Time2=s;
  while(Time2);
}
