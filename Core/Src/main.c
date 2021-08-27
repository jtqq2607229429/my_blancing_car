/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "oled.h"
#include "hc05.h"
#include "math.h"
#include "mpu6050.h"
#include "oled.h"
#include "pid.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float kP = 0, kI = 0, kD = 0;
bool start_flag = false;  //����
int motor, dir1, dir2 = 0;
extern long rxIndex;
extern int out;
extern int t;
extern int halstate;
TxPack txpack;
RxPack rxpack;  //����ʹ�ýṹ��

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int __io_putchar(int ch) {
//    uint8_t temp[1] = {ch};
//    HAL_UART_Transmit(&huart1, temp, 1, 0xff);
//    return (ch);
//}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 1, "hollo!", 12);
    OLED_ShowString(0, 2, "kP:", 12);
    OLED_ShowString(0, 3, "kD:", 12);
    OLED_ShowString(0, 4, "kI:", 12);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  //�����ʼ��
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  //
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 5000);
    __HAL_TIM_SET_COUNTER(&htim4, 5000);  // ��������ʼ��

    PID_Init(&BC_A_PID, 420, 0, 1.2);
    PID_Init(&BC_P_PID, 550, 2.75, 1.1);                //pid��ʼ��
    PID_Init(&BC_T_PID, -9, 0, -0.8);

    MPU6050_initialize();
    DMP_Init();                                                   //DMP6050��ʼ��

    HAL_Delay(500);                                         //�ȴ�6050��ʼ��

    HAL_UART_Receive_IT(&huart1, (uint8_t *) &uartByte, 1);  //������ʼ��

    HAL_Delay(200);

    HAL_TIM_Base_Start_IT(&htim1);
   //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        if (readValuePack(&rxpack)) {          //��������������
            kP = (float) (rxpack.floats[0]);
            kI = (float) (rxpack.floats[1]);               //����ǵڶ����ٶ�PID��kPֵ.
            kD = (float) (rxpack.floats[2]);               //���pid�趨ֵ
            motor = rxpack.integers[0];                    //����ٶ�ֵ

            dir1 = rxpack.integers[1];

            if (rxpack.bools[0]) {                            //����������flash
                save();
            }
            if (rxpack.bytes[0]) {                            //����
                start_flag = true;
                PID_Init(&BC_A_PID, kP, kI, kD);
//                PID_Init(&BC_P_PID, kI, kI / 200.0, kD);
//                PID_Init(&BC_T_PID, kP, 0, kD);
            } else {
                start_flag = false;
            }
        }
        //����������
        txpack.floats[0] = Pitch + 2;
        txpack.floats[1] = -2;
        txpack.floats[2] = Yaw;
        sendValuePack(&txpack);         //����������Ϣ

        OLED_ShowNum(36, 2, kP, 12);
        OLED_ShowNum(36, 3, kD, 12);
        OLED_ShowNum(36, 4, kI, 12);
        OLED_ShowNum(36, 5, Pitch, 12);
        OLED_ShowNum(36, 6, rxIndex, 12);
        OLED_ShowNum(36, 7, t, 12);//��Ļ��ʾ��Ϣ
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
#pragma clang diagnostic pop
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
    while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
