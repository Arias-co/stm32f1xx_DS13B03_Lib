/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "Print.h"
#include "usbd_cdc_if.h"

#include "OneWire.h"
#include "DS18B20.h"

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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

class Usb: public Print
{
public:

    Usb();
    virtual void write( uint8_t * text );
};

Usb::Usb()
{

}

void Usb::write( uint8_t * text )
{

    CDC_Transmit_FS( text, strlen( (char*) text ) );
}

Usb usb;

OneWire_t OW;

/* Array for DS18B20 ROM number */
uint8_t DS_ROM[8];

/* Temperature variable */
float temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
static void MX_GPIO_Init( void );
static void MX_TIM2_Init( void );
/* USER CODE BEGIN PFP */

void CDC_ReceiveCallBack( uint8_t * Buf, uint16_t Len );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void )
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
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    /* the high temperature indicator LED starts off */
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );

    usb.print( "starting... \r\n" );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    OneWire_Init( &OW, Onwire_GPIO_Port, Onwire_Pin, htim2 );
    if ( OneWire_First( &OW ) )
    {
        /* Set LED GREEN */

        /* Read ROM number */
        OneWire_GetFullROM( &OW, DS_ROM );
    }
    else
    {

    }

    /* Start temp conversion */
    if ( DS18B20_Is( DS_ROM ) )
    {
        /* Set resolution */
        DS18B20_SetResolution( &OW, DS_ROM, DS18B20_Resolution_11bits );

        /* Set high and low alarms */
        DS18B20_SetAlarmHighTemperature( &OW, DS_ROM, 30 );
        DS18B20_SetAlarmLowTemperature( &OW, DS_ROM, 10 );

        /* Start conversion on all sensors */
        DS18B20_StartAll( &OW );
    }
    while ( 1 )
    {
        HAL_Delay( 700 );
        /* Check if connected device is DS18B20 */
        if ( DS18B20_Is( DS_ROM ) )
        {
            /* Everything is done */
            if ( DS18B20_AllDone( &OW ) )
            {
                /* Read temperature from device */
                if ( DS18B20_Read( &OW, DS_ROM, &temp ) )
                {
                    usb.print( "Temperature = %.3f \r\n", temp );
                    /* Start again on all sensors */
                    DS18B20_StartAll( &OW );

                    /* Check temperature */
                    if ( temp > 30 )
                    {
                        /* the high temperature indicator LED turns on */
                        HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
                    }
                    else
                    {
                        /* turns off the high temperature indicator led */
                        HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );
                    }
                }
                else
                {
                    /* CRC failed, hardware problems on data line */
                }
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit =
    { 0 };

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
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ) != HAL_OK )
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init( void )
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig =
    { 0 };
    TIM_MasterConfigTypeDef sMasterConfig =
    { 0 };

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if ( HAL_TIM_Base_Init( &htim2 ) != HAL_OK )
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if ( HAL_TIM_ConfigClockSource( &htim2, &sClockSourceConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if ( HAL_TIMEx_MasterConfigSynchronization( &htim2, &sMasterConfig )
            != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOD_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOA_CLK_ENABLE( );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( Onwire_GPIO_Port, Onwire_Pin, GPIO_PIN_RESET );

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    /*Configure GPIO pin : Onwire_Pin */
    GPIO_InitStruct.Pin = Onwire_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( Onwire_GPIO_Port, &GPIO_InitStruct );

}

/* USER CODE BEGIN 4 */
void CDC_ReceiveCallBack( uint8_t * Buf, uint16_t Len )
{
    HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );
    usb.print( Buf );
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
