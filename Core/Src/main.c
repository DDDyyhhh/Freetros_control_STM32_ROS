/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"
#include "motor.h"
#include "pid.h"
#include "Serial.h"
#include "Control.h"
#include "Key.h"
#include "Encoder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Self_Check_Motors();
//void RosTask(void *argument);
//void ControlTask(void *argument);
//void LedTask(void *argument); // 强烈推荐的调试任务

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_char ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart3, &rx_char, 1);
	Key_Init();
	Serial_Init(); // <<< 确保调试串口被初始化
	Motor_Init();
	Control_Init();
	setup();
	

	HAL_TIM_Base_Start_IT(&htim6);
	


//   ===【关键】设定一个目标位置 ===
//   让小车前进 50000 个脉冲的距离
//   Control_Set_Target_Position_Left(200.0f); // 你需要去 Control.c/h 添加这个函数
//	 Control_Set_Target_Position_Right(200.0f);
//	 HAL_Delay(5000);
//	 Control_Set_Target_Position_Left(1000.0f); // 你需要去 Control.c/h 添加这个函数
//	 Control_Set_Target_Position_Right(1000.0f);
	//*******************实验测试轮子编码器值和实际运动的值************************************
//	while(Key_GetNum() != KEY0_PRES)
//  {
//      // 等待...
//      HAL_Delay(10);
//  }
//  
//  Serial_Printf("Calibration Test Started. Robot will move forward for 5 seconds.\r\n");
//  HAL_Delay(500); // 延时一下，给你拿开手的时间

//  // 2. 在开始运动前，清零编码器计数
//  Encoder_Get_Left(); 
//  Encoder_Get_Right();

//  // 3. 让左右轮以一个较低的、恒定的速度同向转动
//  int16_t test_pwm = 25; // 使用一个较小的 PWM 值，例如 25%
//  Motor_SetSpeed_Left(test_pwm);
//  Motor_SetSpeed_Right(test_pwm); // 确保两个轮子都正转
//  
//  // 4. 持续前进 5 秒
//  HAL_Delay(5000); 
//  
//  // 5. 停止电机
//  Motor_SetSpeed_Left(0);
//  Motor_SetSpeed_Right(0);
//  
//  // 6. 读取这 5 秒内累积的总脉冲数
//  int32_t total_ticks_left = Encoder_Get_Left();
//  int32_t total_ticks_right = Encoder_Get_Right();
//  
//  // 7. 通过串口打印出最终结果
//  Serial_Printf("========== Calibration Complete! ==========\r\n");
//  Serial_Printf("Please measure the distance traveled (in cm).\r\n");
//  Serial_Printf("Total Ticks Left: %ld\r\n", total_ticks_left);
//  Serial_Printf("Total Ticks Right: %ld\r\n", total_ticks_right);
//	
	
  // 按键(PE4)未被按下时，HAL_GPIO_ReadPin 返回 GPIO_PIN_SET (高电平)
//  while(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) != GPIO_PIN_RESET)
//  {
//    // 让 LED0(PF9) 闪烁，表示正在等待
//    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//    HAL_Delay(200);
//  }
//  
//  // 按键被按下(低电平)后，跳出循环
//  // 让 LED0(PF9) 常亮，表示程序已开始运行 (低电平点亮)
//  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); 
////	
//	setup();
		//uint8_t message[] = "1\r\n"; // 加上换行符
	
	 
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		loop();
//		HAL_UART_Transmit(&huart3, message, sizeof(message) - 1, 100); // 100ms 超时
//    HAL_Delay(500); // 每秒发送两次
//	    uint8_t key = Key_GetNum(); 
//			if (key == KEY0_PRES)
//			{
//					Control_Set_Target_Position_Left(500.0f);
//					Control_Increase_Target_Speed_Ticks(50.0f); // 目标速度增加 10
//			}
//				HAL_Delay(20);


//		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//    HAL_Delay(500);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//  if (huart->Instance == USART3) {
//    // 如果收到的字符是 'a'
//    if (rx_char == 'a') {
//      // 翻转 LED0 (PF9)
//      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//    }
//    // 再次启动下一次接收
//    HAL_UART_Receive_IT(&huart3, &rx_char, 1);
//  }
//}




void Self_Check_Motors(void)
{
    int16_t test_speed = 30; // 使用一个较低的速度进行测试，例如 30% 占空比
    uint32_t delay_ms = 1000; // 每个动作持续 1 秒

    // 1. 提示开始自检 (LED 快闪)
    Serial_Printf("Starting Motor Self-Check...\r\n");
    for(int i=0; i<5; i++)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET); // 先熄灭 LED

    // 2. 测试右轮正转
    Serial_Printf("Testing Right Wheel Forward...\r\n");
    Motor_SetSpeed_Right(test_speed);
    HAL_Delay(delay_ms);
    Motor_SetSpeed_Right(0); // 测试完后立即停止
    HAL_Delay(500); // 短暂延时

    // 3. 测试右轮反转
    Serial_Printf("Testing Right Wheel Backward...\r\n");
    Motor_SetSpeed_Right(-test_speed);
    HAL_Delay(delay_ms);
    Motor_SetSpeed_Right(0);
    HAL_Delay(500);

    // 4. 测试左轮正转
    Serial_Printf("Testing Left Wheel Forward...\r\n");
    Motor_SetSpeed_Left(test_speed);
    HAL_Delay(delay_ms);
    Motor_SetSpeed_Left(0);
    HAL_Delay(500);

    // 5. 测试左轮反转
    Serial_Printf("Testing Left Wheel Backward...\r\n");
    Motor_SetSpeed_Left(-test_speed);
    HAL_Delay(delay_ms);
    Motor_SetSpeed_Left(0);

    // 6. 自检结束，LED 常亮 (假设低电平点亮)
    Serial_Printf("Self-Check Complete!\r\n");
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000); // 延时1秒，让你能看到常亮状态
}




/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
