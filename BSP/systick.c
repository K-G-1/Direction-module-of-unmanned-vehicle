//头文件
#include "systick.h"
extern uint16_t nTime;
 /**
  * @file   SYSTICK_Init
  * @brief  初始化SYSTICK，1Ms中断1次
  * @param  无
  * @retval 无
  */
void SYSTICK_Init(void)
{		/*SystemCoreClock/1000000：1us中断1次；SystemCoreClock/ 1000：1ms中断一次*/
	while (SysTick_Config(SystemCoreClock/1000));
}


 /**
  * @file   delay_ms
  * @brief  毫秒延时
  * @param  延时时间
  * @retval 无
  */
void delay_ms(uint16_t nms)
{
  nTime=nms;
  while(nTime);
}



