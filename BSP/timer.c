/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��Timer.c
 * ����    ����ʼ��ͨ�ö�ʱ��TIM2��ʵ��TIM2��ʱ���� 
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-12-20
 * Ӳ������: ��
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/

#include "timer.h"
#include "led.h"
#include "usart.h"
#include "IMU.h"
#include <stdio.h>
uint16_t Time2;
u8 Txbuff[48];
 /**
  * @file   TIM2_Config
  * @brief  ���ú����⣬��ʼ����ʱ��2������
  * @param  ��
  * @retval ��
  */
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	//ʹ��TIM2ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	//��ʱ����ʱʱ��T���㹫ʽ��T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK
	TIM_TimeBaseStructure.TIM_Period = (30-1);//�Զ���װ��ֵ20--��ʱʱ��(10*4800/48M)s 
	TIM_TimeBaseStructure.TIM_Prescaler =(4800-1);//Ԥ��Ƶֵ��+1Ϊ��Ƶϵ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision =0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	//ʹ��TIM2�ж�Դ
	TIM_Cmd(TIM2, ENABLE);  	//ʹ��TIMx����						 	
}

 /**
  * @file   TIM2_IRQHandler
  * @brief  ��ʱ��2�жϴ�����
  * @param  ��
  * @retval ��
  */
void TIM2_IRQHandler(void)
{	
	static int times_count = 0;
    if ( TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET ) //�Ƿ����ж�
    {	
        
		Get_Attitude();
		times_count ++;
		
		if(times_count %10 == 0)
		{
			sprintf((char*)Txbuff,"put:%2.2f,%2.2f,%3.2f\r\n",(angle.roll),(angle.pitch),(angle.yaw));
			UART_PutStr(USART1,Txbuff);
//			USART_SendData(USART1, 0x02);  
//			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
		}
		
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    //����жϴ�����λ
    }		 
}

 /**
  * @file   TIM2_IRQHandler
  * @brief  ���������ж�
  * @param  ��
  * @retval ��
  */
void TIM2_delay(uint16_t s)
{
  Time2=s;
  while(Time2);
}
