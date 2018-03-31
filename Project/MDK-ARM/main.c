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
#include "stmflash.h"

/**
  * @file   main
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	
	SYSTICK_Init();				
	I2C_Configuration();		//IIC初始化 SCL ：PB4   SDA： PB5
	LED_GPIO_Config();		 	//led初始化，led1为PA5
    USART1_Config();			//串口参数配置初始化
	USART2_Config();
	
    while(MPU_Init()){
		UART_PutStr(USART2,"faild\r\n");
		delay_ms(1000);
	}
	Acc_Offest_Read();
	Gyro_Offest_Read();
	
	Init_HMC5883L();
	Mag_Offest_Read();

	TIM2_Config();				//TIM2定时器初始化 中断间隔 3ms
		
    while (1)
    {

//		UART_PutStr(USART2,"test\r\n");
		delay_ms(1000);
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





