//ͷ�ļ�
#include "systick.h"
extern uint16_t nTime;
 /**
  * @file   SYSTICK_Init
  * @brief  ��ʼ��SYSTICK��1Ms�ж�1��
  * @param  ��
  * @retval ��
  */
void SYSTICK_Init(void)
{		/*SystemCoreClock/1000000��1us�ж�1�Σ�SystemCoreClock/ 1000��1ms�ж�һ��*/
	while (SysTick_Config(SystemCoreClock/1000));
}


 /**
  * @file   delay_ms
  * @brief  ������ʱ
  * @param  ��ʱʱ��
  * @retval ��
  */
void delay_ms(uint16_t nms)
{
  nTime=nms;
  while(nTime);
}



