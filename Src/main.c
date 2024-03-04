/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2024 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CHANNEL_B 2
#define CHANNEL_C 3
#define CHANNEL_D 4
#define MIN_VOLTAGE 1.61 // 1.61mV
// 5.1; //2.45;

#define MAX_VOLTAGE 3.00 // 3.00mV

#define VSLSB 0.0025 // 0.0025mV
#define CLSB 0.025 // 0.025mA
#define VLSB 1.25 // 1.25mV
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

float GetVoltage(uint16_t voltageHex);
float GetShuntVoltage(uint16_t voltageHex);
float GetCurrent(uint16_t currentHex);
uint16_t GetShuntVoltageReg(void);
uint16_t GetBusVoltageReg(void);
uint16_t GetCurrentReg(void);
void SetDACOut(uint16_t value, uint8_t channel);
void PrintTransmitMeasuredVC(int delay);
void PrintTransmitApproximatedVC(int delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t spiData[3];
uint16_t outFine = 0;
uint16_t out;
// uint16_t outDec;
uint16_t outCoarse = 1;
// uint16_t outFineDec = 0;
// uint16_t outCoarseDec = 0;

// float k = 0.84422;// for 30omh
// float b = 0.04587;// for 30omh
// float b_ = 0.16735;// for 30omh

// float k2 = 0.000388057; // for 30omh
// float b2 = -0.07494; // for 30omh
// float b3 = 0.05910838 -0.05912935787;
// float b3 = 0.00002092; // for 30omh

// for 6 Ohm
float k2 = 0.000474392; // approximate
float b2 = -0.205391382;

// float k1 = 0.84629863; // correct INA
// float b1 = 0.06857670;

// float approxCoefficient = 103.5414; // 30 omh
float approxCoefficient = 424.6; // 30 omh

uint8_t I2C_Data[3];
uint8_t recData[3];
uint8_t devAddr = 0x80;
uint16_t configWord;
uint16_t calibWord;
uint16_t receive;
// uint16_t volHex;
uint8_t sendOK;
float voltage;
float current;
// float shuntVoltage;
uint16_t g_counter = 0;
int g_delay = 300;

// uint8_t data[7]; //= "Hello\r\n";
char data2[40];

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //0 100 000 100 100 111 // use 4 averages, 1.1 ms conversion time
  //0 100 010 100 100 111 // use 16 averages
	//configWord = 0x327; 
	//configWord = 0x4927;  128 avg
	configWord = 0x4527;// 16 avg
	I2C_Data[0] = 0x00;
	I2C_Data[1] = (configWord & 0xFF00) >> 8;
	I2C_Data[2] = (configWord & 0x00FF);
	HAL_I2C_Master_Transmit(&hi2c1, devAddr, I2C_Data, 3, 20);
	receive = HAL_I2C_Master_Receive(&hi2c1, devAddr, &recData[1],2,10);
	
	// 2. Set Calibration register
	calibWord = 0x800;
	I2C_Data[0] = 0x05;
	I2C_Data[1] = (calibWord & 0xFF00) >> 8;
	I2C_Data[2] = (calibWord & 0x00FF);
	sendOK = 	HAL_I2C_Master_Transmit(&hi2c1, devAddr, I2C_Data, 3, 10);
	receive = HAL_I2C_Master_Receive(&hi2c1, devAddr, &recData[1],2,10);
	HAL_Delay(100);
	
  // 1. Set up the DAC

  // doing reset

  out = 0x0001;
  spiData[0] = 0x28;   //00101000
  spiData[1] = (out & 0xFF00) >> 8;
  spiData[2] = (out & 0x00FF);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); 
  HAL_SPI_Transmit(&hspi1, spiData, 3, 10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  // using internal reference
  out = 0x0001;
  spiData[0] = 0x38;   //00111000 = 0x38
  spiData[1] = (out & 0xFF00) >> 8;
  spiData[2] = (out & 0x00FF);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  if(!HAL_SPI_Transmit(&hspi1, spiData, 3, 10))
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  //HAL_Delay(1);

  //configuring LDAC
  HAL_Delay(10);
  out = 0x000C;
  spiData[0] = 0x30;   //00110000 = 0x30
  spiData[1] = (out & 0xFF00) >> 8;
  spiData[2] = (out & 0x00FF);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spiData, 3, 10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  /*
  //set D to zero
  //HAL_Delay(10);
  out = 0x000;
  spiData[0] = 0x13;   //00010011
  spiData[1] = (out & 0xFF0) >> 4;
  spiData[2] = (out & 0x00F)<<4;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spiData, 3, 10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  out = 0x000;*/

  outCoarse = 0; //1.41 mV
  outFine = 4000;
  SetDACOut(outFine, CHANNEL_B);
  SetDACOut(outCoarse, CHANNEL_C);
	SetDACOut(0, CHANNEL_D);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // outCoarse = 4090; //1.41 mV
    // outFine = 0; //
    // SetDACOut(0, CHANNEL_B);
    // SetDACOut(outCoarse, CHANNEL_C);
    // SetDACOut(outFine, CHANNEL_D);

    PrintTransmitMeasuredVC(g_delay);
    
    // PrintTransmitApproximatedVC(g_delay);

    /* 
    while(1)
    {
      PrintTransmitMeasuredVC(1000);
      g_counter++;
    }
    */

    /*
    while(voltage < MIN_VOLTAGE){
        // PrintTransmitApproximatedVC(g_delay);
        PrintTransmitMeasuredVC(g_delay);
        g_counter++;
      
        for(outFine = 1; outFine < 100; outFine++) //increment by 10 
        {
          SetDACOut(outCoarse, CHANNEL_C);
          SetDACOut(outFine, CHANNEL_D);

          // PrintTransmitApproximatedVC(g_delay);
          PrintTransmitMeasuredVC(g_delay);
          g_counter++;
        }
        outCoarse += 1;
        // outFine = 0;
    }
    
    while(1){
      // outDec++;
      out += 5;
      SetDACOut(out, CHANNEL_C);
      SetDACOut(outFine, CHANNEL_D);
      if(out >= 4095)
          out = 1;

      // PrintTransmitApproximatedVC(g_delay);
      PrintTransmitMeasuredVC(g_delay);
      g_counter++;
    }
    */

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

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void SetDACOut(uint16_t value, uint8_t channel) 
{
   spiData[1] = (value & 0xFF0) >> 4;
   spiData[2] = (value & 0x00F) << 4;
   if(channel == CHANNEL_B)
      spiData[0] = 0x11;
   else if(channel == CHANNEL_C)
      spiData[0] = 0x12;
   else if(channel == CHANNEL_D)
      spiData[0] = 0x13;
   //00010010
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi1, spiData, 3, 10);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

/**
  * @brief  Measures values of Voltage and Current then Prints and Transmits them. 
  * @param  delay - optional delay after transmitting data in ms.
  * @retval None
  */
void PrintTransmitMeasuredVC(int delay)
{
  voltage = GetVoltage(GetBusVoltageReg())/1000;
  current = GetCurrent(GetCurrentReg());
  snprintf(data2, sizeof data2, " %1.3f %1.3f %d %d %d \n\r", voltage, current, outCoarse, outFine, g_counter);
  HAL_UART_Transmit(&huart2, data2, sizeof data2, 100);
  HAL_Delay(delay);
}

/**
  * @brief  Calculates Approximate values of Voltage and Current then Prints and Transmits them. 
  * @param  delay - optional delay after transmitting data in ms.
  * @retval None
  */
void PrintTransmitApproximatedVC(int delay)
{
  voltage = GetVoltage(GetBusVoltageReg())/1000;
  current = (outCoarse * approxCoefficient + outFine) * k2 + b2;
  snprintf(data2, sizeof data2, " %1.3f %1.3f %d %d %d \n\r", voltage, current, outCoarse, outFine, g_counter);
  HAL_UART_Transmit(&huart2, data2, sizeof data2, 100);
  HAL_Delay(delay);
}

uint16_t GetShuntVoltageReg(void){	
	I2C_Data[0] = 0x01;
	sendOK = 	HAL_I2C_Master_Transmit(&hi2c1, devAddr, I2C_Data, 1, 10);
	receive = HAL_I2C_Master_Receive(&hi2c1, devAddr, &recData[1],2,10);
	return ((uint16_t)recData[1]<<8 | recData[2]);
}

uint16_t GetBusVoltageReg(void){
	I2C_Data[0] = 0x02;
	sendOK = 	HAL_I2C_Master_Transmit(&hi2c1, devAddr, I2C_Data, 1, 10);
	receive = HAL_I2C_Master_Receive(&hi2c1, devAddr, &recData[1],2,10);
	return ((uint16_t)recData[1]<<8 | recData[2]);
}

uint16_t GetCurrentReg(void){
	I2C_Data[0] = 0x04;
	sendOK = 	HAL_I2C_Master_Transmit(&hi2c1, devAddr, I2C_Data, 1, 10);
	receive = HAL_I2C_Master_Receive(&hi2c1, devAddr, &recData[1],2,10);
	return ((uint16_t)recData[1]<<8 | recData[2]);
}

float GetVoltage(uint16_t voltageHex){
	uint16_t temp;
	if(voltageHex>>15)
	{
		temp=~voltageHex;
		return (((temp*(-1)) + 0x01)*VLSB);
	}
	else
	return voltageHex*VLSB;
}

float GetShuntVoltage(uint16_t voltageHex){
	uint16_t temp;
	if(voltageHex>>15)
	{
		temp=~voltageHex;
		return (((temp*(-1)) + 0x01)*VSLSB);
	}
	else
	  return voltageHex*VSLSB;
}

float GetCurrent(uint16_t currentHex){
  /*	
  uint16_t temp;
	if((currentHex>>15) & (currentHex<=0xFFF9))
		return 0;
	if((currentHex>>15) & (currentHex>=0xFFFA))
  {
		temp = ~currentHex;
	  return (((temp*(-1))+ 0x06)*CLSB);
	}
  else
	  return (currentHex+5)*CLSB;
  */
	uint16_t temp;
	if(currentHex>>15)
	{
		temp=~currentHex;
		return (((temp*(-1))+ 0x01)*CLSB);
	}
	else
	  return currentHex*CLSB;
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
