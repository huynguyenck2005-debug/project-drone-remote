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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "mpu6050.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "pid.h"
#include "i2c_lcd.h"

#define FAILSAFE_MS  300
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Sửa ở cả remote và drone
typedef struct __attribute__((packed)) {
    uint16_t throttle;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
} RemoteFrame;
typedef struct __attribute__((packed)) {
    uint8_t  battery_percent;
    uint16_t vbat_mv;
    uint16_t motor1_ccr;
    uint16_t motor2_ccr;
    uint16_t motor3_ccr;
    uint16_t motor4_ccr;
} TelemetryFrame;
/* Map setpoint remote (-300~+300) → góc độ (-30°~+30°) */
#define SP_TO_DEG(x)   ((float)(x) / 300.0f * 30.0f)
#define MOTOR_CLAMP(x) ((x) > 999 ? 999 : ((x) < 0 ? 0 : (x)))

PID_t pid_roll, pid_pitch, pid_yaw;

uint32_t last_tick    = 0;
uint32_t last_rx_tick = 0;
uint8_t  armed        = 0;

I2C_LCD_HandleTypeDef lcd1;
RemoteFrame rx_frame  = {0};

float out_roll = 0, out_pitch = 0, out_yaw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ADC_Read_Single_Channel(uint32_t channel) {
      ADC_ChannelConfTypeDef sConfig = {0};

      sConfig.Channel = channel;
      sConfig.Rank = ADC_REGULAR_RANK_1;
      sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
          Error_Handler();
      }

      HAL_ADC_Start(&hadc1);

      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
          uint32_t value = HAL_ADC_GetValue(&hadc1);
          HAL_ADC_Stop(&hadc1);
          return value;
      }

      return 0;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* PWM channels cho LED */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* Tắt hết LED ban đầu */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

  /* MPU6050 */
  if (MPU6050_Init(&hi2c1) != HAL_OK) Error_Handler();

  /* NRF24 */
  csn_high();
  HAL_Delay(100);
  nrf24_defaults();
  ce_low();
  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_2mbps);
  nrf24_set_channel(78);
  nrf24_set_crc(en_crc, _1byte);

  uint8_t addr[5] = {0x10, 0x21, 0x32, 0x43, 0x54};
  nrf24_open_tx_pipe(addr);
  nrf24_open_rx_pipe(0, addr);
  nrf24_set_addr_width(5);
  nrf24_auto_ack(0, enable);        /* Bật auto-ack pipe 0 */
  nrf24_auto_retr_limit(3);         /* Retry 3 lần — cần cho ACK hoạt động */
  nrf24_auto_retr_delay(1);         /* 500µs giữa retry */
  nrf24_dpl(enable);                /* Dynamic payload length */
  nrf24_set_rx_dpl(0, enable);      /* DPL pipe 0 */
  nrf24_en_ack_pld(enable);         /* Bật ACK payload */
  nrf24_pipe_pld_size(0, sizeof(RemoteFrame));
  HAL_Delay(5);
  nrf24_listen();

  TelemetryFrame telem_init = {0};
  nrf24_flush_rx();
  nrf24_flush_tx();
  nrf24_transmit_rx_ack_pld(0, (uint8_t*)&telem_init, sizeof(TelemetryFrame));

  /* LCD */
//  lcd1.hi2c    = &hi2c1;
//  lcd1.address = 0x27 << 1;
//  lcd_init(&lcd1);
//  lcd_gotoxy(&lcd1, 0, 0);
//  lcd_puts(&lcd1, "Calibrating...");

  /* MPU6050 calibrate */
  MPU6050_Calibrate();
  MPU6050_InitAngles();
//  lcd_clear(&lcd1);

  /* PID init — bắt đầu với Kp nhỏ, Ki=0, Kd=0 để test */
  PID_Init(&pid_roll,  1.5f, 0.01f, 0.0f, 100.0f, 300.0f);
  PID_Init(&pid_pitch, 1.5f, 0.01f, 0.0f, 100.0f, 300.0f);
  PID_Init(&pid_yaw,   0.0f, 0.0f, 0.0f, 30.0f, 200.0f);

  last_tick    = HAL_GetTick();
  last_rx_tick = HAL_GetTick();
  int32_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* ── 1. Tính dt ── */
		  HAL_Delay(10);
		  uint32_t now = HAL_GetTick();
		  float dt = (now - last_tick) / 1000.0f;
		  last_tick = now;

	    /* ── 2. Cập nhật góc ── */
	    MPU6050_Update(dt);
	    /* Chuẩn bị telemetry */
	    TelemetryFrame telem;
	    telem.battery_percent = armed;
	    telem.vbat_mv         = (uint16_t)(HAL_GetTick() - last_rx_tick);
	    telem.motor1_ccr      = (uint16_t)MOTOR_CLAMP(m1);
	    telem.motor2_ccr      = (uint16_t)MOTOR_CLAMP(m2);
	    telem.motor3_ccr      = (uint16_t)MOTOR_CLAMP(m3);
	    telem.motor4_ccr      = (uint16_t)MOTOR_CLAMP(m4);

	    /* ── 3. Nhận NRF24 ── */
	    if (nrf24_data_available()) {
	        nrf24_receive((uint8_t*)&rx_frame, sizeof(RemoteFrame));
	        last_rx_tick = HAL_GetTick();
	        armed = 1;
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	        nrf24_clear_max_rt();
	        if (!(nrf24_r_reg(FIFO_STATUS, 1) & (1 << TX_FULL_FIFO))) {
	            nrf24_transmit_rx_ack_pld(0, (uint8_t*)&telem, sizeof(TelemetryFrame));
	        }
	    } else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);

	    /* ── 4. Failsafe ── */
	    if (armed && (HAL_GetTick() - last_rx_tick) > FAILSAFE_MS) {
	        armed = 0;
	        PID_Reset(&pid_roll);
	        PID_Reset(&pid_pitch);
	        PID_Reset(&pid_yaw);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

//	        while (rx_frame.throttle > 0) {
//	             rx_frame.throttle = (rx_frame.throttle < 25) ? 0 : (rx_frame.throttle - 25);
//
//	             /* Vẫn cập nhật MPU và giữ drone ổn định trong lúc hạ */
//	             uint32_t now = HAL_GetTick();
//	             float dt = (now - last_tick) / 1000.0f;
//	             last_tick = now;
//	             MPU6050_Update(dt);
//
//	             out_roll  = PID_Update(&pid_roll,  0, MPU6050.angle_y, dt);
//	             out_pitch = PID_Update(&pid_pitch, 0, MPU6050.angle_x, dt);
//
//	             int32_t thr = rx_frame.throttle;
//	             int32_t m1 = thr - (int32_t)out_roll - (int32_t)out_pitch;
//	             int32_t m2 = thr + (int32_t)out_roll - (int32_t)out_pitch;
//	             int32_t m3 = thr - (int32_t)out_roll + (int32_t)out_pitch;
//	             int32_t m4 = thr + (int32_t)out_roll + (int32_t)out_pitch;
//
//	             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)MOTOR_CLAMP(m1));
//	             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)MOTOR_CLAMP(m2));
//	             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)MOTOR_CLAMP(m3));
//	             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)MOTOR_CLAMP(m4));
//
//	             HAL_Delay(100);
//	        }

	    } else if (armed) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	    }

	    /* ── 5. PID + Motor mix ── */
	    m1 = 0; m2 = 0; m3 = 0; m4 = 0;
	    if (armed) {
	        float sp_roll  = SP_TO_DEG(rx_frame.roll);
	        float sp_pitch = SP_TO_DEG(rx_frame.pitch);

	        out_roll  = PID_Update(&pid_roll,  sp_roll,  MPU6050.angle_y, dt);
	        out_pitch = PID_Update(&pid_pitch, sp_pitch, MPU6050.angle_x, dt);
	        out_yaw   = 0;  /* Chưa có yaw */

	        /* Throttle 0–19999, PID output -300~+300 */
	        int32_t thr = rx_frame.throttle;

	        /* X-frame mix:
	         *   M1=front-left   M2=front-right
	         *   M3=rear-left    M4=rear-right   */
	        m1 = thr - (int32_t)out_roll + (int32_t)out_pitch;
	        m2 = thr + (int32_t)out_roll + (int32_t)out_pitch;
	        m3 = thr - (int32_t)out_roll - (int32_t)out_pitch;
	        m4 = thr + (int32_t)out_roll - (int32_t)out_pitch;

	        /* Throttle thấp → tắt (tránh integral windup) */
	        if (thr < 500) {
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	            PID_Reset(&pid_roll);
	            PID_Reset(&pid_pitch);
	        } else {
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_CLAMP(m1));
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_CLAMP(m2));
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_CLAMP(m3));
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_CLAMP(m4));
	        }
	    } else {
	        /* Không có tín hiệu → tắt hết */
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	    }



//	    /* ── 6. LCD debug — cập nhật mỗi 200ms ── */
//	    static uint32_t lcd_tick = 0;
//	    if ((HAL_GetTick() - lcd_tick) >= 200) {
//	        lcd_tick = HAL_GetTick();
//
//	        char msg[24];
//
//	        /* Dòng 0: M1 và M2 */
//	        lcd_gotoxy(&lcd1, 0, 0);
//	        sprintf(msg, "1:%-6d2:%-6d", (int)MOTOR_CLAMP(m1), (int)MOTOR_CLAMP(m2));
//	        lcd_puts(&lcd1, msg);
//
//	        /* Dòng 1: M3 và M4 */
//	        lcd_gotoxy(&lcd1, 0, 1);
//	        sprintf(msg, "3:%-6d4:%-6d", (int)MOTOR_CLAMP(m3), (int)MOTOR_CLAMP(m4));
//	        lcd_puts(&lcd1, msg);
//	    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
