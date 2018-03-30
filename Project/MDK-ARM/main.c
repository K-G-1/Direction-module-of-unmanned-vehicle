/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：main.c
 * 描述    ：通过串口1打印输出自己输入的字符串；（串口信息为：9600，N,8，1）  
 * 库版本  ：V1.3.0
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2015-12-20
 * 调试方式：J-Link-OB
**********************************************************************************/	

//头文件
#include "stm32f0xx.h"
#include "led.h"
#include "usart.h"
#include "i2c.h"
#include "systick.h"
#include "MPU6050.h"
#include "IMU.h"
#include "HMC5883L.h"
#include "timer.h"


/**
  * @file   main
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	
	SYSTICK_Init();
	I2C_Configuration();
	LED_GPIO_Config();
    USART1_Config();//串口参数配置初始化
	
    while(MPU_Init()){
		UART_PutStr(USART1,"faild\r\n");
		delay_ms(1000);
		}
	MPU_Gyro_Offset();
	MPU_Acc_Offset();
	
	Init_HMC5883L();
//	HMC5883L_Start_Calib();
	TIM2_Config();	
		
    while (1)
    {

//		USART_SendData(USART1, 0x02);  
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
//		delay_ms(1000);
    }
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/





