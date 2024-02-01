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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define Function_Set 0b00110000 //0x30 
 #define Display_On 0b00001100 //0x0C can also try 0x0f 
 #define Display_Clear 0b00000001 //0x01 
 #define Entry_Mode_Set 0b00000110 //0x06 
 #define RETURN_HOME 0b00000010 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
 
 void LCD_Init(void);
 void LCD_Sel_Row_Col(uint8_t row, uint8_t col);
 void LCD_SendCommand (uint8_t cmd);
 void LCD_SendData (uint8_t value);
 void LCD_Clear(void);
 void LCD_sendString(char* in);
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
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
 
 char *usr_msg = " ECE 5530 Minhaz Shahrier Sriram Karthik ";
 HAL_SPI_Init(&hspi4);
 LCD_Init();
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1) 
 { 
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */
  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */
  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */
  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
 void LCD_Init(void) 
{ 
 //8 bit interface: 
 
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // Make sure low
HAL_Delay(100);
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, GPIO_PIN_SET); //Low to high 
HAL_Delay(50); // greater than 40ms
LCD_SendCommand(Function_Set);
HAL_Delay(10); // greater than 100us
LCD_SendCommand(Function_Set);
HAL_Delay(10); // greater than 37us
LCD_SendCommand(Display_On);
HAL_Delay(10); // greater than 100us
LCD_SendCommand(Display_Clear);
HAL_Delay(15); // greater than 10ms 
LCD_SendCommand(Entry_Mode_Set);
HAL_Delay(1); // Init sequence end 
 } 
void LCD_Clear(void) 
{ 
 LCD_SendCommand(Display_Clear);
 HAL_Delay(2); // greater than 2ms 
} 
uint8_t data [3];
uint8_t data2 [3];
void LCD_SendCommand(uint8_t cmd) 
{ 
 
 data2[0]=0xF8;
 data2[1]= (cmd & 0xF0);
 data2[2]=((cmd << 4) & 0xF0);
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
 HAL_SPI_Transmit(&hspi4, data2, 3, 1);
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
} 
void LCD_SendData (uint8_t value){ 
 
 data[0]=0xFA;
 data[1]= (value & 0xF0);
 data[2]=((value << 4) & 0xF0);
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); 
 HAL_SPI_Transmit(&hspi4, data, 3, 1);
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
} 
void LCD_Sel_Row_Col(uint8_t row, uint8_t col) 
{ 
 switch(row) 
 { 
 
 //Selecting Row and collum "Instruction Set" 
 case 0: 
 col /= 2; //cursor is 2 spaces wide
 col |= 0x80; // 0b10000000 
 break; 
 case 1: 
 col /= 2; //cursor is 2 spaces wide
 col |= 0x90; // 0b10010000 
 break; 
 case 2: 
 col /= 2; //cursor is 2 spaces wide 
 col |= 0x88; // 0b10001000 
 break; 
 case 3: 
 col /= 2; //cursor is 2 spaces wide
 col |= 0x98; // 0b10011000 
 break; 
 default: 
 col |= 0x80; // 0b10000000 
 break; 
 } 
 LCD_SendCommand(col); 
 HAL_Delay(2); 
 } 
void LCD_sendString(char* in) { 
 char padded_str[64];
 uint8_t len = strlen(in);
 
 // copy input string to padded_str and add spaces at the end 
 strncpy(padded_str, in, len);
 for(uint8_t i = len; i < 64; i++)
 padded_str[i] = ' ';
 padded_str[64] = '\0'; // terminate the string
 
 LCD_Sel_Row_Col(0, 0); 

 for(uint8_t i = 0; i < 16; i++) 
 LCD_SendData(padded_str[i]); 
 //Select third line and send from 16 to 31 
 LCD_Sel_Row_Col(1, 0); 
 for(uint8_t i = 16; i < 32; i++) 
 LCD_SendData(padded_str[i]); 
 LCD_Sel_Row_Col(2, 0); 
 for(uint8_t i = 32; i < 48; i++) 
 LCD_SendData(padded_str[i]); 
 
 //Select fourth line and send continuous underscores 
 LCD_Sel_Row_Col(3, 0); 
 for(uint8_t i = 48; i < 64; i++) 
 LCD_SendData(padded_str[i]); 
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