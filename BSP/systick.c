/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��SYSTICK.c
 * ����    ����શ�ʱ��ʵ��ms��ʱ
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-12-20
 * Ӳ������  :��
 * ���Է�ʽ��J-Link-OB
********************************************************************************/

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



