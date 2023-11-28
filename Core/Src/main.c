/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define A1 70  //测量范围下限
#define A2 700 //测量范围上限
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint16_t ADC_LOG[100]= {0};
uint16_t LOG_AVG_Data = 0;
static unsigned char pre_status = 0;
_Bool mode = 1;
float dis_set = 90;
uint16_t tick = 0;
char send_str[30] = {0x00,};
typedef struct 
{
    /* data */
    uint8_t mode;
    uint8_t control;
    float dis_set;
    float dis_cur;
}Hydr_TypeDef;

Hydr_TypeDef hydr_dev ={
    0,
    0,
    0.0,
    0.0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float distance_calc(float a1,float a2,float ad_value);
int solenoid_valve_control(unsigned char new_status);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  //MX_IWDG_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_TIM_Base_Start(&htim3);
  //HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_LOG,100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    tick++;
    /*计算当前距离，并显示*/
    //hydr_dev.dis_cur = distance_calc(A1,A2,LOG_AVG_Data);
    //添加距离显示（打印函数）
    sprintf((char *)send_str,"wset dis_cur.valf %d.%d\r\n",((uint32_t)(hydr_dev.dis_cur*100))/100,((uint32_t)(hydr_dev.dis_cur*100))%100);
    HAL_UART_Transmit_IT(&huart1,(uint8_t *)send_str,strlen(send_str));
    /*如果是自动模式模式，根据当前距离和设定距离，计算出控制方法  0：stop 1：- 2：+*/
    if(hydr_dev.mode == 1){

        //检测距离小于预设距离，电池阀B得电，A断电，伸缩杆缩回
        if( hydr_dev.dis_set -  hydr_dev.dis_cur > 0.1)
            hydr_dev.control = 2;
        //检测距离大于预设距离，电池阀A得电，B断电，伸缩杆伸出
        else if(hydr_dev.dis_set - hydr_dev.dis_cur < -0.1)
            hydr_dev.control = 1;
        else
            hydr_dev.control = 0;
    }
    /*根据结构体中control属性，控制继电器通断，从而控制电机和相应电磁阀工作*/
    solenoid_valve_control(hydr_dev.control);
   
    HAL_Delay(20);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief ADC IRQ Callback
 * 
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint32_t i, sum = 0;
	if (hadc->Instance == hadc1.Instance)
	{
    for(i=0;i<100;++i)
      sum += ADC_LOG[i];
    LOG_AVG_Data = sum / 100;
		
	}
}
/**
 * @brief 根据adc值 计算实时距离
 * 
 */
float distance_calc(float a1,float a2,float ad_value){
    /*
    * a1- 70 ,a2 - 1000,dis = (a2 - a1) / (2440 - 10) * (ad_value - 10) + a1
    *     60        2450
    */
   return((a2-a1)/2430 * (ad_value - 10)+ a1); 
}
/**
 * @brief  电池阀控制
 *  0 - 电池阀A断电 ，电池阀B断电
 *  1 - 电池阀A得电，电池阀B断电
 *  2 - 电池阀A断电，电池阀B得电
 */

int solenoid_valve_control(unsigned char new_status){
    if(new_status == pre_status)
        return 0;
    else{
        switch (new_status)
        {
            case 0 :
            {
                HAL_GPIO_WritePin(Relay_A_GPIO_Port, Relay_A_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Relay_B_GPIO_Port, Relay_B_Pin, GPIO_PIN_RESET);
            }
             break;
            case 1 :
            {
                HAL_GPIO_WritePin(Relay_A_GPIO_Port, Relay_A_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Relay_B_GPIO_Port, Relay_B_Pin, GPIO_PIN_RESET);
            }
             break;
            case 2 :
            {
                HAL_GPIO_WritePin(Relay_A_GPIO_Port, Relay_A_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Relay_B_GPIO_Port, Relay_B_Pin, GPIO_PIN_SET);
            }
             break;
            default:
                return -1;
                break;
        }
    }
    pre_status = new_status;
    return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

		if(huart->Instance == USART1){
			memcpy(cmd_data,Uart_ReadCache,Size);
            cmd_size  = Size;
            char *p = NULL;
            //解析串口接收数据
            if(cmd_size > 0)
            {
                if(strstr(cmd_data,"mode:auto\r\n") != NULL)
                {
                    hydr_dev.mode  = 1;
                }
                else if(strstr(cmd_data,"mode:manual\r\n") != NULL)
                {
                    hydr_dev.mode  = 0;
                }
                else if((p = strstr(cmd_data,"dis_set:")) != NULL){
                    float setdis = atof(p+8);
                    //判断输入数据是否在检测范围内
                    if(( setdis >= 70) &&( setdis <= 1000))
                      hydr_dev.dis_set  = setdis;
                }
                else if(strstr(cmd_data,"dis:+\r\n") != NULL){
                    hydr_dev.control = 2;
                }
                else if(strstr(cmd_data,"dis:-\r\n") != NULL){
                    hydr_dev.control = 1;
                }
                else if (strstr(cmd_data,"stop\r\n") != NULL)
                {
                    hydr_dev.control = 0;
                }
                else
                ;
                memset(cmd_data,0x00,cmd_size);
                cmd_size = 0;
            }
			memset(Uart_ReadCache,0x00,300);
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, Uart_ReadCache, 300);
		}
    else	if(huart->Instance == USART2){
			memcpy(dis_data,Uart2_ReadCache,Size);
            dis_size  = Size;
            char *p = NULL;
            //解析串口接收数据
            if(dis_size > 0)
            {
                if(strstr(dis_data,",") != NULL)
                {
                    hydr_dev.dis_cur = strtod(dis_data,NULL);
                }
                else
                ;
                memset(dis_data,0x00,dis_size);
                dis_size = 0;
            }
			memset(Uart2_ReadCache,0x00,100);
			HAL_UARTEx_ReceiveToIdle_IT(&huart2, Uart2_ReadCache, 100);
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
