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





