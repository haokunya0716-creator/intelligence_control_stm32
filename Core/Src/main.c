/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_button.h"
#include "app_encoder.h"
#include "app_usart.h"
#include "task.h"
#include "drv8701.h"
#include "gx_delay.h"

#include "app_servo.h"
#include "app_motor.h"
#include "app_speed.h"
#include "JY901S.h"
#include "rescue_usart2.h"
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
uint8_t rx_byte_imu;   // 串口6接收陀螺仪数据
/*
 *调试用变量
 */
float duty_l = 0;
float duty_r = 0;
float angle_servo = 180.0f;
int key1_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 所有定时器和 GPIO 初始化完成后，启动 DRV8701E 的两路 EN PWM。
   * Motor_Init() 会先写入停止值，避免上电初始化过程中产生
   * 意外的电机动作。
   */
  Motor_Init();
  Motor_Cmd(1);

  /* 协议服务按收到的命令选择速度闭环或偏航角闭环。 */
  App_Speed_Init();
  App_Motor_Init();

  App_Servo_Init();//初始化舵机
  HAL_UART_Receive_IT(&huart3, &rx_byte_imu, 1);//陀螺仪接收初始化


  /*
   * TIM1 以 1 MHz 计数并开启更新中断，供 gx_GetUs() 提供
   * 64 位微秒时间戳。TIM1 的自动重装载值为 65535，因此每次
   * 更新中断对应 65536 us 的软件累计。
   */
  /*
   * 启动前显式清零计数器和更新标志，避免个别复位状态下遗留的
   * UIF 标志被 gx_GetUs() 当成一次真实的 16 位计数器溢出。
   */

  HAL_TIM_Base_Start_IT(&htim1);
  if (Rescue_Usart2_Start() == 0U) {
    Error_Handler();
  }
  App_USART1_Printf("start!\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (key1_flag == 1) {
      /*
       * 这里来放开始的时候要执行的代码
       */
      key1_flag = 0;
    }
    //App_Usart_USB_Process();
    PERIODIC_START(KEY_DETECT,20)
    App_Button_Proc();
    PERIODIC_END

    PERIODIC_START(PEINT_USART,100)
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
    PERIODIC_END


    /* 处理 USART2 命令，并运行当前命令选中的速度环或角度环。 */
    Rescue_Usart2_Process();
    // PERIODIC_START(PEINT_USART,15)
    // App_USART1_Printf("%.3f,%.3f\n",
    //                   angle_yaw_omega_ref,
    //                   angle_yaw_omega);

    // Motor_Set_L(duty_l);
    // Motor_Set_R(duty_l);
    // App_USART1_Printf("%.3f\n",
    //                  gyroZRate);

    // App_USART1_Printf("%.3f,%.3f\n",
    //                   encoder_l_pos,
    //                   encoder_r_pos);
    //PERIODIC_END
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    IMU_PutByte(rx_byte_imu);
    HAL_UART_Receive_IT(&huart3, &rx_byte_imu, 1);
  } else if (huart->Instance == USART2) {
    /* USART2 的组帧和缓存由协议模块负责；这里不执行电机或舵机动作。 */
    extern void Rescue_Usart2_RxCallback(void);
    Rescue_Usart2_RxCallback();
  }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
