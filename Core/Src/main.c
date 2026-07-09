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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAGIC0 0xAA
#define MAGIC1 0x55

#define TYPE_DC     0x00
#define TYPE_SERVO  0x01
#define TYPE_POLAR  0x02

#define TYPE_SENSORS 0x01

#pragma pack(push,1)

typedef struct
{
    uint8_t pin;
    uint8_t type;
    uint16_t duty_cycle;
    uint16_t duration;
    uint16_t ramp;   
} PwmCommand;

typedef struct
{
    uint16_t methane;      // ADC_CHANNEL_0 
    uint16_t co2;
    float temperature;
    float moisture;
    uint16_t adc2;         // ADC_CHANNEL_1 
    uint16_t adc3;         // ADC_CHANNEL_4 
} SensorReadings;

#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
uint16_t scan_steps = 0;
uint32_t last_dht22_read = 0;
uint32_t last_sensor_send = 0;

#define NUM_DC_MOTORS 6
#define NUM_SERVOS    4

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PwmMap_t;

// timer, channel for DC motors (Motor 1-3 on TIM1, Motor 4-6 on TIM2)
static const PwmMap_t dc_motor_map[NUM_DC_MOTORS] = {
    { &htim1, TIM_CHANNEL_1 },  // Motor 1
    { &htim1, TIM_CHANNEL_2 },  // Motor 2
    { &htim1, TIM_CHANNEL_3 },  // Motor 3
    { &htim2, TIM_CHANNEL_1 },  // Motor 4
    { &htim2, TIM_CHANNEL_2 },  // Motor 5
    { &htim2, TIM_CHANNEL_3 },  // Motor 6
};

// timer, channel for servos
static const PwmMap_t servo_map[NUM_SERVOS] = {
    { &htim3, TIM_CHANNEL_1 },
    { &htim3, TIM_CHANNEL_2 },
    { &htim3, TIM_CHANNEL_3 },
    { &htim3, TIM_CHANNEL_4 },
};

// Motor tracking structure
typedef struct {
    uint32_t target_duty;
    uint32_t current_duty;
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
void Process_USB_Command(uint8_t* buffer, uint32_t length);
void Run_Polarimeter_Scan(uint16_t steps);
uint16_t Read_CO2_Sensor(void);
void Read_DHT22(float *temperature, float *humidity);
uint16_t Read_Methane(void);
uint16_t Read_Analog_Input(uint32_t channel);
void SendFramedPacket(uint8_t type, const uint8_t *payload, uint16_t payload_len);
void Delay_us(uint16_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t CalcChecksum(const uint8_t *data, uint32_t len)
{
    uint8_t checksum = 0;

    for(uint32_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // Start PWM output on every mapped DC motor channel
  for (int i = 0; i < NUM_DC_MOTORS; i++) {
      HAL_TIM_PWM_Start(dc_motor_map[i].htim, dc_motor_map[i].channel);
  }

  // Start PWM output on every mapped servo channel
  for (int i = 0; i < NUM_SERVOS; i++) {
      HAL_TIM_PWM_Start(servo_map[i].htim, servo_map[i].channel);
  }

  // Free-running 1MHz counter on TIM4, used by Delay_us() and Read_DHT22()'s timeouts
  HAL_TIM_Base_Start(&htim4);

  // Reset timers
  last_dht22_read = HAL_GetTick();
  last_sensor_send = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();

    // Auto-off for DC motors
    for (int i = 0; i < NUM_DC_MOTORS; i++) {
        if (dc_motors[i].turn_off_time != 0 && current_time >= dc_motors[i].turn_off_time) {
            __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, 0);
            dc_motors[i].current_duty = 0;
            dc_motors[i].target_duty = 0;
            dc_motors[i].turn_off_time = 0;
        }
    }

    // Linear Motor Ramping
    static uint32_t last_ramp_time = 0;
    if (current_time - last_ramp_time >= 10) {
        last_ramp_time = current_time;

        for (int i = 0; i < NUM_DC_MOTORS; i++) {
            if (dc_motors[i].current_duty < dc_motors[i].target_duty) {
                dc_motors[i].current_duty++;
                __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, dc_motors[i].current_duty);
            } else if (dc_motors[i].current_duty > dc_motors[i].target_duty) {
                dc_motors[i].current_duty--;
                __HAL_TIM_SET_COMPARE(dc_motor_map[i].htim, dc_motor_map[i].channel, dc_motors[i].current_duty);
            }
        }
    }

    // Polarimeter
    if (start_polarimeter_scan) {
        Run_Polarimeter_Scan(scan_steps);
        start_polarimeter_scan = 0; // Clear flag when finished
    }

    // Sensor Readings
    if (current_time - last_dht22_read >= 2000) {
      last_dht22_read = current_time;

      SensorReadings pkt;

      pkt.methane = Read_Methane();
      pkt.co2 = Read_CO2_Sensor();

      // analog inputs
      pkt.adc2 = Read_Analog_Input(ADC_CHANNEL_1);
      pkt.adc3 = Read_Analog_Input(ADC_CHANNEL_4);

      Read_DHT22(&pkt.temperature, &pkt.moisture);

      SendFramedPacket(
          TYPE_SENSORS,
          (uint8_t*)&pkt,
          sizeof(pkt)
      );
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief microsecond helper function
  * @retval uint16_t for delay amount
  */
void Delay_us(uint16_t us) {
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim4);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) - start) < us);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
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
  htim3.Init.Prescaler = 48;
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
  htim4.Init.Prescaler = 71; // 72MHz / 72 = 1MHz -> needed for Delay_us()/Read_DHT22() timing
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Configure Stepper Pulse Pin: PA1 as Output */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SendFramedPacket(uint8_t type,
                      const uint8_t *payload,
                      uint16_t payload_len)
{
    uint8_t packet[64];

    if(payload_len + 4 > sizeof(packet))
        return;

    packet[0] = MAGIC0;
    packet[1] = MAGIC1;
    packet[2] = type;

    memcpy(&packet[3], payload, payload_len);

    packet[3 + payload_len] =
        CalcChecksum(payload, payload_len);

    CDC_Transmit_FS(packet, payload_len + 4);
}

void Process_USB_Command(uint8_t *buffer, uint32_t length)
{
    if (length < sizeof(PwmCommand) + 3)
        return;

    // Verify header
    if (buffer[0] != MAGIC0 || buffer[1] != MAGIC1)
        return;

    // Verify checksum
    uint8_t checksum = CalcChecksum(&buffer[2], sizeof(PwmCommand));

    if (checksum != buffer[2 + sizeof(PwmCommand)])
        return;

    // Extract command
    PwmCommand cmd;
    memcpy(&cmd, &buffer[2], sizeof(PwmCommand));

    switch (cmd.type)
    {
        case TYPE_DC:
            if (cmd.pin < NUM_DC_MOTORS)
            {
                dc_motors[cmd.pin].target_duty = cmd.duty_cycle;
                dc_motors[cmd.pin].turn_off_time = HAL_GetTick() + (cmd.duration * 100);
            }
            break;

        case TYPE_SERVO:
            if (cmd.pin < NUM_SERVOS)
            {
                __HAL_TIM_SET_COMPARE(servo_map[cmd.pin].htim, servo_map[cmd.pin].channel, cmd.duty_cycle);
            }
            break;

        case TYPE_POLAR:
            scan_steps = cmd.duration;
            start_polarimeter_scan = 1;
            break;
    }
}

void Run_Polarimeter_Scan(uint16_t steps) {
    ADC_ChannelConfTypeDef sConfig = {0};

    for (uint16_t i = 0; i < steps; i++) {
        // Pulse stepper pin
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(5);

        // Dynamically switch ADC to Channel 5 (Polarimeter Input)
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        // Read ADC Sample
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
            // TODO: store/send adc_val
        }
        HAL_ADC_Stop(&hadc1);
    }
}

uint16_t Read_CO2_Sensor(void)
{
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uint8_t resp[9] = {0};

    HAL_UART_Transmit(&huart3, cmd, 9, 100);

    if (HAL_UART_Receive(&huart3, resp, 9, 200) == HAL_OK)
    {
        return (resp[2] << 8) | resp[3];
    }

    return 0;
}

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
    if ((data[0] + data[1] + data[2] + data[3]) == data[4]) {
        short raw_humidity = (data[0] << 8) | data[1];
        short raw_temperature = (data[2] << 8) | data[3];

        *humidity = (float)raw_humidity / 10.0f;
        *temperature = (float)raw_temperature / 10.0f;
    }
}

uint16_t Read_Methane(void) {
    return Read_Analog_Input(ADC_CHANNEL_0);
}

/**
  * @brief  Reads a single ADC1 channel on demand. Used for methane and the
  *         other misc analog inputs (ADC2/ADC3 per pin doc).
  * @param  channel: STM32 HAL channel definition (e.g. ADC_CHANNEL_1)
  * @retval 12-bit raw analog conversion value
  */
uint16_t Read_Analog_Input(uint32_t channel) {
    uint16_t adc_val = 0;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
