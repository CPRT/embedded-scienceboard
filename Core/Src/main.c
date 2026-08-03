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
/* CAN MESSAGES
 * Base ID: 30
 *
 * SET_MOTOR, Id: 0 (Jetson -> Science)
 * uint8_t pin
 * uint8_t duty_cycle (percent)
 * uint16_t duration
 * uint16_t ramp
 *
 * SET_SERVO, Id: 1 (Jetson -> Science)
 * uint8_t pin
 * uint16_t us
 *
 * POLAR_SCAN, Id: 2 (Jetson -> Science) (no data)
 *
 * ADC_DATA, Id: 3 (Science -> Jetson)
 * uint16_t adc1
 * uint16_t adc2
 * uint16_t adc3
 *
 * TEMP_DATA, Id: 4 (Science -> Jetson)
 * float temperature
 * float humidity
 *
 * CO2_DATA, Id: 5 (Science -> Jetson)
 * uint16_t ppm
 *
 * POLAR_DATA, Id: 6 (Science ->  Jetson)
 * uint8_t index
 * uint16_t[3] data
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
#define CAN_ID 30

#define SET_MOTOR  0x00
#define SET_SERVO  0x01
#define POLAR_SCAN 0x02
#define ADC_DATA   0x03
#define TEMP_DATA  0x04
#define CO2_DATA   0x05
#define POLAR_DATA 0x06

#define SCAN_STEPS 48
#define MICROSTEPS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Flags and tracking variables
uint8_t start_polarimeter_scan = 0;
uint32_t last_dht22_read = 0;
uint32_t last_adc_read = 0;

uint8_t co2_buffer[16] = {0};
int co2_good = 0;

uint16_t sample_buf[SCAN_STEPS];

uint8_t polar_step = 0;
int polar_running = 0;
uint32_t polar_step_time = 0;

ADC_ChannelConfTypeDef polarConfig = {0};

#define NUM_DC_MOTORS 7
#define NUM_SERVOS    4

#define POLAR_MIN 730
#define POLAR_MAX 2170
#define POLAR_STEP 30
#define POLAR_SERVO 3

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PwmMap_t;

static const PwmMap_t dc_motor_map[NUM_DC_MOTORS] = {
    { &htim1, TIM_CHANNEL_1 },  // Motor 1
    { &htim1, TIM_CHANNEL_2 },  // Motor 2
    { &htim1, TIM_CHANNEL_3 },  // Motor 3
    { &htim2, TIM_CHANNEL_1 },  // Motor 4
    { &htim2, TIM_CHANNEL_2 },  // Motor 5
    { &htim2, TIM_CHANNEL_3 },  // Motor 6
    { &htim2, TIM_CHANNEL_4 }   // Heater
};

static const PwmMap_t servo_map[NUM_SERVOS] = {
    { &htim3, TIM_CHANNEL_1 },
    { &htim3, TIM_CHANNEL_2 },
    { &htim3, TIM_CHANNEL_3 },
    { &htim3, TIM_CHANNEL_4 }
};

typedef struct {
    uint32_t target_duty;
    uint32_t start_duty;
    uint32_t current_duty;
    uint32_t start_time;
    uint32_t ramp_end_time;
    uint32_t turn_off_time;
} DC_Motor_t;

DC_Motor_t dc_motors[NUM_DC_MOTORS] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void Read_DHT22(float *temperature, float *humidity);
uint16_t Read_Analog_Input(uint32_t channel);
void Delay_us(uint16_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay_us(uint16_t us) {
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim4);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start) < us);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef   RxHeader;
  uint8_t               RxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
  if ((RxHeader.StdId >> 5) == CAN_ID)
  {
	switch (RxHeader.StdId & 0x1f) {
	  case SET_MOTOR:
		uint8_t pin = RxData[0];
		if (pin < NUM_DC_MOTORS)
		{
		  uint32_t now     = HAL_GetTick();
		  uint32_t rampMs  = (uint32_t)((RxData[4] << 8) | RxData[5]) * 100u;
		  uint32_t holdMs  = (uint32_t)((RxData[2] << 8) | RxData[3]) * 100u;

		  dc_motors[pin].start_duty    = dc_motors[pin].current_duty;
		  dc_motors[pin].target_duty   = RxData[1];
		  dc_motors[pin].start_time    = now;
		  dc_motors[pin].ramp_end_time = now + rampMs;
		  dc_motors[pin].turn_off_time = now + holdMs;
		}
		break;

	  case SET_SERVO:
		if (RxData[0] < NUM_SERVOS)
		{
		  __HAL_TIM_SET_COMPARE(servo_map[RxData[0]].htim, servo_map[RxData[0]].channel, (RxData[1] << 8) | RxData[2]);
		}
		break;

	  case POLAR_SCAN:
		polar_step = 0;
		polar_running = 1;
		polar_step_time = HAL_GetTick();
		break;
	}
  }
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    co2_good = 1;
    HAL_UART_Receive_IT(&huart3, co2_buffer, 16);
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  for (int i = 0; i < NUM_DC_MOTORS; i++) {
      HAL_TIM_PWM_Start(dc_motor_map[i].htim, dc_motor_map[i].channel);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
      HAL_TIM_PWM_Start(servo_map[i].htim, servo_map[i].channel);
  }

  // Free-running 1MHz counter on TIM4, used by Delay_us() and Read_DHT22()'s timeouts
  HAL_TIM_Base_Start(&htim4);

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = CAN_ID<<10;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x3f<<10;
  canfilterconfig.FilterMaskIdLow = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  polarConfig.Channel = ADC_CHANNEL_5;
  polarConfig.Rank = ADC_REGULAR_RANK_1;
  polarConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  last_dht22_read = HAL_GetTick();
  
  HAL_UART_Receive_IT(&huart3, co2_buffer, 16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // DC Motor ramping + auto-off (duty computed directly from elapsed time)
    for (int i = 0; i < NUM_DC_MOTORS; i++) {
        DC_Motor_t *m = &dc_motors[i];

        if (m->turn_off_time == 0) continue;

        if ((int32_t)(current_time - m->turn_off_time) >= 0) {
            __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, 0);
            m->current_duty  = 0;
            m->target_duty   = 0;
            m->turn_off_time = 0;
            continue;
        }

        if ((int32_t)(current_time - m->ramp_end_time) >= 0) {
            if (m->current_duty != m->target_duty) {
                m->current_duty = m->target_duty;
                __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, m->current_duty);
            }
            continue;
        }

        uint32_t elapsed  = current_time - m->start_time;
        uint32_t rampSpan = m->ramp_end_time - m->start_time;
        int32_t  delta    = (int32_t)m->target_duty - (int32_t)m->start_duty;
        int32_t  scaled   = m->start_duty + (delta * (int32_t)elapsed) / (int32_t)rampSpan;

        if (delta >= 0) {
            if (scaled > (int32_t)m->target_duty) scaled = m->target_duty;
        } else {
            if (scaled < (int32_t)m->target_duty) scaled = m->target_duty;
        }

        m->current_duty = (uint32_t)scaled;
        __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, m->current_duty);
    }

    // Polarimeter
    if (polar_running) {
        if (HAL_GetTick() - polar_step_time >= 100) {
        	if (polar_step == 0) {
        		__HAL_TIM_SET_COMPARE(servo_map[POLAR_SERVO].htim, servo_map[POLAR_SERVO].channel, POLAR_MIN);
        		HAL_Delay(1500);
        	}
        	HAL_ADC_ConfigChannel(&hadc1, &polarConfig);

        	HAL_ADC_Start(&hadc1);
        	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        	    sample_buf[polar_step] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        	} else {
        	    sample_buf[polar_step] = 0xFFFF;
        	}
        	HAL_ADC_Stop(&hadc1);

        	/*for (uint16_t i = 0; i < MICROSTEPS; i++) {
        	    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
        	    HAL_Delay(1);
        	    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
        	    HAL_Delay(1);
        	}*/
        	__HAL_TIM_SET_COMPARE(servo_map[POLAR_SERVO].htim, servo_map[POLAR_SERVO].channel, POLAR_MIN + POLAR_STEP * polar_step);
        	polar_step_time = HAL_GetTick();

        	if ((polar_step + 1) % 3 == 0) {
        		CAN_TxHeaderTypeDef   TxHeader;
        		uint8_t               TxData[8];
        		uint32_t              TxMailbox;

        		TxHeader.IDE = CAN_ID_STD;
        		TxHeader.StdId = (CAN_ID << 5) | POLAR_DATA;
        		TxHeader.RTR = CAN_RTR_DATA;
        		TxHeader.DLC = 7;

        		TxData[0] = polar_step - 2;
        		TxData[1] = (sample_buf[polar_step - 2] >> 8) & 0xff;
        		TxData[2] = sample_buf[polar_step - 2] & 0xff;
        		TxData[3] = (sample_buf[polar_step - 1] >> 8) & 0xff;
        		TxData[4] = sample_buf[polar_step - 1] & 0xff;
        		TxData[5] = (sample_buf[polar_step - 0] >> 8) & 0xff;
        		TxData[6] = sample_buf[polar_step - 0] & 0xff;

        		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        		{
        		  //Error_Handler();
        		  // Dropping is better than crashing
        		}
        	}

        	polar_step++;
        	if (polar_step == SCAN_STEPS) {
        		polar_running = 0;
        		for (int i = 0; i < SCAN_STEPS; i++) {
        			sample_buf[i] = 0;
        		}
        	}
        }
    }

    // Sensor Readings
    if (current_time - last_dht22_read >= 2000) {
        last_dht22_read = current_time;

        float temperature = 0.0f, moisture = 0.0f;
        uint32_t temp_bytes, moisture_bytes;

        Read_DHT22(&temperature, &moisture);

        memcpy(&temp_bytes, &temperature, 4);
        memcpy(&moisture_bytes, &moisture, 4);

        CAN_TxHeaderTypeDef   TxHeader;
        uint8_t               TxData[8];
        uint32_t              TxMailbox;

        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = (CAN_ID << 5) | TEMP_DATA;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = 8;

        TxData[0] = (temp_bytes >> 24) & 0xff;
        TxData[1] = (temp_bytes >> 16) & 0xff;
        TxData[2] = (temp_bytes >> 8) & 0xff;
        TxData[3] = temp_bytes & 0xff;
        TxData[4] = (moisture_bytes >> 24) & 0xff;
        TxData[5] = (moisture_bytes >> 16) & 0xff;
        TxData[6] = (moisture_bytes >> 8) & 0xff;
        TxData[7] = moisture_bytes & 0xff;

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
          //Error_Handler();
        }
    }

    // CO2
    if (co2_good) {
    	co2_good = 0;
        uint16_t co2 = 0xFFFF;
        if (co2_buffer[0] == 0x42 && co2_buffer[1] == 0x4D)
        {
            // 2. Calculate the 8-bit additive checksum over bytes 0 to 14
            uint8_t calculated_checksum = 0;
            for (int i = 0; i < 15; i++)
            {
                calculated_checksum += co2_buffer[i];
            }

            // 3. Verify the computed checksum matches the packet's trailing byte
            if (calculated_checksum == co2_buffer[15])
            {
                // 4. Extract CO2 PPM (BYTE 6 is High Byte, BYTE 7 is Low Byte)
                co2 = ((uint16_t)co2_buffer[6] << 8) | co2_buffer[7];
            }
        }

        CAN_TxHeaderTypeDef   TxHeader;
        uint8_t               TxData[8];
        uint32_t              TxMailbox;

        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = (CAN_ID << 5) | CO2_DATA;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = 2;

        TxData[0] = (co2 >> 8) & 0xff;
        TxData[1] = co2 & 0xff;

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
          //Error_Handler();
        }
    }

    if (current_time - last_adc_read >= 100) {
        last_adc_read = current_time;

        uint16_t adc1 = Read_Analog_Input(ADC_CHANNEL_5);
        uint16_t adc2 = Read_Analog_Input(ADC_CHANNEL_1);
        uint16_t adc3 = Read_Analog_Input(ADC_CHANNEL_4);

        CAN_TxHeaderTypeDef   TxHeader;
        uint8_t               TxData[8];
        uint32_t              TxMailbox;

        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = (CAN_ID << 5) | ADC_DATA;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = 6;

        TxData[0] = (adc1 >> 8) & 0xff;
        TxData[1] = adc1 & 0xff;
        TxData[2] = (adc2 >> 8) & 0xff;
        TxData[3] = adc2 & 0xff;
        TxData[4] = (adc3 >> 8) & 0xff;
        TxData[5] = adc3 & 0xff;

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
          //Error_Handler();
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 237;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 237;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 48-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|DIR_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Read_DHT22(float *temperature, float *humidity){
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t i, j;
    uint16_t start_time;
    const uint16_t timeout_us = 200; // 200us safety limit for individual signal phases

    TIM4->CNT = 0;

    // Host Start Signal (PB6 = TIM4_CH1 pin, used here in bit-banged GPIO mode)
    Set_Pin_Output(GPIOB, GPIO_PIN_6);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(18); // Pull low for 18ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    Delay_us(30);  // Pull high for 30us

    // Switch to Input to read DHT response
    Set_Pin_Input(GPIOB, GPIO_PIN_6);

    // Wait for DHT response (low 80us then high 80us)
    Delay_us(40);
    if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)) return;

    // Safety Timeout for initial low phase
    start_time = __HAL_TIM_GET_COUNTER(&htim4);
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {
        if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start_time) > timeout_us) return;
    }

    // Safety Timeout for initial high phase
    start_time = __HAL_TIM_GET_COUNTER(&htim4);
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
        if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start_time) > timeout_us) return;
    }

    // Read 40 bits = 5 bytes
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 8; i++) {
            // Safety Timeout waiting for low bit preamble to end (pin goes high)
            start_time = __HAL_TIM_GET_COUNTER(&htim4);
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {
                if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start_time) > timeout_us) return;
            }

            Delay_us(40); // Check pin state after 40 microseconds

            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
                data[j] |= (1 << (7 - i)); // It's a '1'

                // Safety Timeout waiting for high data transmission to end
                start_time = __HAL_TIM_GET_COUNTER(&htim4);
                while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
                    if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start_time) > timeout_us) return;
                }
            }
        }
    }

    // Check Checksum and Parse
    uint8_t check = data[0] + data[1] + data[2] + data[3];
    if (check == data[4]) {
        short raw_humidity = (data[0] << 8) | data[1];
        short raw_temperature = (data[2] << 8) | data[3];

        *humidity = (float)raw_humidity / 10.0f;
        *temperature = (float)raw_temperature / 10.0f;
    }
}

uint16_t Read_Analog_Input(uint32_t channel) {
    uint16_t adc_val = 0xFFFF;
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return adc_val;
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
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
