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
#include "ltc6811.h"
#include "global_definitions.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define NUM_OF_SLAVES 1
#define NUM_OF_CELLS 6
#define EVEN_SLAVE_CELLS 6 //12
#define ODD_SLAVE_CELLS 6 //11
#define NUM_OF_MUX_CHANNELS 8
#define NUM_OF_THERMISTORS (NUM_OF_SLAVES * 36)
#define MOVING_AVERAGE_SAMPLES 10 //number of samples to take exponential moving average over
#define VOLTAGE_SAG_PERCENT_TOLERANCE 10 //percentage cells are allowed to deviate from moving average without being considered fusable link pop

#define BATTERY_CAPCITY 10000
#define CURRENT_SENSE_RATIO 12.5 //12.5mv / A
#define CURRENT_SENSE_OFFSET 2500 //at 0A, current sensor voltage is 2.5V



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */


double SOC; //state of charge, value between 0-100 representing charge
uint64_t last_SOC_update_ms; //last time SOC was updated, measured in ms since boot
uint16_t adc_val[1];
double current_sense_voltage;
double current;
int64_t time_since_last_update;

float cell_voltage[NUM_OF_CELLS];      // index 0 = lowest potential
float cell_voltage_ma[NUM_OF_CELLS];   // index 0 = lowest potential
ltc6811 ltc6811_arr[NUM_OF_SLAVES]; // index 0 = lowest potential
float thermistor_temps[NUM_OF_THERMISTORS];

//analog signals being passed to ACU LV for logging on CAN
double acu_measure;
double TS_measure;

float voltage_sag_allowed_float = 1 - (float)VOLTAGE_SAG_PERCENT_TOLERANCE / 100.0;


int fuse_pop; //FOR TESTING
int AMS_OK = 0; //digital signal controlling AMS relay (OUTPUT)
int IMD_OK; //digital signal reading IMD fault status (INPUT)
int IMD_M; //PWM signal reading IMD measurement (INPUT)
int RPM_SIGNAL; //PWM signal for reading fan speed (INPUT)
int PWM_SIGNAL; //0-100 value for fan rpm (OUTPUT)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//FIGURE OUT WHICH BUS IS LOGGING AND WHICH IS COMMANDS
void send_can1();


void send_can2();


//NEED TO UPDATE TO HAL
uint32_t vcu_adc_read_millivolts(ADC_HandleTypeDef* hadc, uint32_t channel)
{

	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_0;

	sConfig.Rank = 1;

	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;

	HAL_ADC_ConfigChannel(hadc, &sConfig);

	HAL_ADC_Start(hadc);

	HAL_ADC_PollForConversion(hadc, 1000);

	uint16_t adc_val = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	double val_mv = adc_val * 2500 / 4096;

	return val_mv;

}





//take voltage from current sensor and calculate current
int64_t current_sense_voltage_to_current (double current_sense_voltage)
{
return (current_sense_voltage - CURRENT_SENSE_OFFSET) / CURRENT_SENSE_RATIO * -1;  //multiplied by -1 because current sensor is backwards
}




//once battery is charged, reset SOC value
//use lookup table / formula to measure pack voltage and estimate SOC
void reset_SOC()
{
last_SOC_update_ms = HAL_GetTick();
SOC = 100;
}



//take current and time since last caulcations to update SOC
//NEED TO UPDATE TO HAL
void update_SOC ()
{

//double current_sense_voltage = vcu_adc_read_millivolts(&hadc1, ADC_CHANNEL_4);
current_sense_voltage = adc_val[0] * 2500 / 4096;
current = current_sense_voltage_to_current(current_sense_voltage);

uint64_t current_ms = HAL_GetTick();

time_since_last_update = current_ms - last_SOC_update_ms;

last_SOC_update_ms = current_ms;

SOC -= current * time_since_last_update / BATTERY_CAPCITY;

}

void update_moving_average(float *current_avg, float new_sample)
{

	*current_avg = ((MOVING_AVERAGE_SAMPLES - 1) * *current_avg + new_sample) / MOVING_AVERAGE_SAMPLES;
}

int check_fusable_link(float ma, float cell_voltage)
{

	if(cell_voltage < ma * voltage_sag_allowed_float)
	{
		return 1; //ADD SDC?
	}
	return 0;
}

void set_moving_average(float *cell_voltage_ma, float *cell_voltage)
{
	for (int cell = 0; cell < NUM_OF_CELLS; cell++)
	{
		cell_voltage_ma[cell] = cell_voltage[cell];
	}
}



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
  MX_RTC_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA (&hadc1, (uint32_t*)adc_val, 1);
  HAL_ADC_Start(&hadc1);

  HAL_Delay(100); // 100ms should allow all relevant power circuitry to stabilize

  //reset_SOC();

  init_LTC6811();

  wake_sleep(); // wake LTC6811 from sleep

  // Configuration for all LTC6811s
  struct ltc6811_config ltc6811_config;
  ltc6811_config.gpio_pulldowns = GPIO1_NO_PULLDOWN | GPIO2_NO_PULLDOWN | GPIO3_NO_PULLDOWN | GPIO4_NO_PULLDOWN | GPIO5_NO_PULLDOWN;
  ltc6811_config.refon = REFON_STAY_POWERED;
  ltc6811_config.adcopt = ADCOPT_MODE_0;
  ltc6811_config.vuv = 0;
  ltc6811_config.vov = 0;
  ltc6811_config.dcto = 0;
  update_config(&ltc6811_config);


  // configure LTC6811 structs to match real life setup
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
      ltc6811_arr[i].address = i;
      ltc6811_arr[i].cell_count = (i % 2 == 0) ? EVEN_SLAVE_CELLS : ODD_SLAVE_CELLS;
  }

  reset_SOC();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	//set AMS_OK

  /* USER CODE END 2 */

  wake_sleep(); // wake LTC6811 from sleep
  // send command to read cell voltages
  broadcast_command(ADCV(MD_27k_14k, DCP_NOT_PERMITTED, CH_ALL));

  HAL_Delay(1); // reading all cell voltages @ "26Hz" should take 210ms

  // read cell voltage registers from all slaves on the bus
  read_all_voltages(ltc6811_arr, NUM_OF_SLAVES);


  // calculate actual voltage values
  extract_all_voltages(ltc6811_arr, cell_voltage, NUM_OF_SLAVES);

  set_moving_average(cell_voltage_ma, cell_voltage);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      wake_sleep(); // wake LTC6811 from sleep
      // send command to read cell voltages
      broadcast_command(ADCV(MD_27k_14k, DCP_NOT_PERMITTED, CH_ALL));

      HAL_Delay(1); // reading all cell voltages @ "27kHz" should take 1ms

      // read cell voltage registers from all slaves on the bus
      read_all_voltages(ltc6811_arr, NUM_OF_SLAVES);

      // calculate actual voltage values
      extract_all_voltages(ltc6811_arr, cell_voltage, NUM_OF_SLAVES);
      float sum_voltage = 0;
      for(int i = 0; i < NUM_OF_CELLS; i++)
      {
      update_moving_average(&cell_voltage_ma[i], cell_voltage[i]);
      sum_voltage += cell_voltage_ma[i];
      }
      float average_voltage = sum_voltage / NUM_OF_CELLS;

      for(int i = 0; i < NUM_OF_CELLS; i++)
      {
      if(check_fusable_link(average_voltage, cell_voltage[i]))
      {
    	  fuse_pop = 1;
      }
      }


	  if(read_all_temps(ltc6811_arr, thermistor_temps, NUM_OF_MUX_CHANNELS, NUM_OF_SLAVES)) //0 = no fault, 1 = fault
	  {
		  AMS_OK = 1;
	  }

	  if (AMS_OK || fuse_pop)
	  {
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

//		  HAL_Delay(100);
	  }

//	  update_SOC();




	  //HAL_Delay(1000);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//   __NOP();
//}

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
