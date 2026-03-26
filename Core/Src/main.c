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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "norm_params.h"
#include "threshold_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} ICM20948_Raw_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_SAMPLE_PERIOD_MS        100U    /* 0.1s interval */
#define IMU_WINDOW_SIZE             10U     /* Window size (1 seconds) */
#define IMU_NUM_FEATURES            6U      /* ax, ay, az, gx, gy, gz */

#define IMU_CS_PORT                 GPIOA
#define IMU_CS_PIN                  GPIO_PIN_4

/* USER BANK 0 */
#define ICM20948_WHO_AM_I           0x00
#define ICM20948_PWR_MGMT_1         0x06
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_REG_BANK_SEL       0x7F

/* USER BANK 2 */
#define ICM20948_GYRO_SMPLRT_DIV    0x00
#define ICM20948_GYRO_CONFIG_1      0x01
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG       0x14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
uint8_t imu_ok = 0U;

/* ── Ring buffer ── */
static float ring_buf[IMU_WINDOW_SIZE][IMU_NUM_FEATURES];
static uint16_t ring_write_idx = 0U;
static uint32_t total_samples = 0U;

/* ── Timing ── */
static uint32_t sample_tick = 0U;

/* ── Inference I/O buffers ── */
static float ai_in_buf[IMU_WINDOW_SIZE * IMU_NUM_FEATURES];
static float ai_out_buf[IMU_WINDOW_SIZE * IMU_NUM_FEATURES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_SendString(const char *str);

static void UART_SendChar(char c);
static void UART_SendInt(int32_t value);
static void UART_SendFloat(float value);
static void UART_SendINFO(const char *str);
static void UART_SendERROR(const char *str);

static void IMU_CS_Low(void);
static void IMU_CS_High(void);

static void ICM20948_WriteReg(uint8_t reg, uint8_t value);
static uint8_t ICM20948_ReadReg(uint8_t reg);
static void ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
static void ICM20948_SelectBank(uint8_t bank);
static uint8_t ICM20948_Init_Minimal(void);
static uint8_t ICM20948_Config_Basic(void);
static uint8_t ICM20948_ReadRaw(ICM20948_Raw_t *raw);

static void Normalize_Sample(const ICM20948_Raw_t *raw, float out[IMU_NUM_FEATURES]);

/*/* Inference function defined in app_x-cube-ai.c */
extern int AI_Run_Model(const float *in_data, float *out_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================================
 *  UART Helper Functions
 * ================================================================ */
void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void UART_SendChar(char c)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&c, 1U, HAL_MAX_DELAY);
}

static void UART_SendInt(int32_t value)
{
  char buf[12];
  uint32_t i = 0U;
  uint32_t mag;

  if (value < 0)
  {
    UART_SendChar('-');
    mag = (uint32_t)(-value);
  }
  else
  {
    mag = (uint32_t)value;
  }

  do
  {
    buf[i++] = (char)('0' + (mag % 10U));
    mag /= 10U;
  } while (mag > 0U);

  while (i > 0U)
  {
    UART_SendChar(buf[--i]);
  }
}

static void UART_SendFloat(float value)
{
  if (value < 0.0f)
  {
    UART_SendChar('-');
    value = -value;
  }
  uint32_t int_part = (uint32_t)value;
  UART_SendInt((int32_t)int_part);
  UART_SendChar('.');
  float frac = value - (float)int_part;
  for (int i = 0; i < 8; i++)
  {
    frac *= 10.0f;
    uint8_t digit = (uint8_t)frac;
    UART_SendChar('0' + digit);
    frac -= (float)digit;
  }
}

static void UART_SendINFO(const char *str)
{
  UART_SendString("INFO, ");
  UART_SendString(str);
  UART_SendString("\r\n");
}

static void UART_SendERROR(const char *str)
{
  UART_SendString("ERROR, ");
  UART_SendString(str);
  UART_SendString("\r\n");
}

/* ================================================================
 *  ICM-20948 SPI Driver (unchanged)
 * ================================================================ */
static void IMU_CS_Low(void)
{
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
}

static void IMU_CS_High(void)
{
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

static void ICM20948_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  tx[0] = reg & 0x7FU;
  tx[1] = value;

  IMU_CS_Low();
  HAL_SPI_Transmit(&hspi1, tx, 2U, HAL_MAX_DELAY);
  IMU_CS_High();
}

static uint8_t ICM20948_ReadReg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80U;
  tx[1] = 0xFFU;

  rx[0] = 0U;
  rx[1] = 0U;

  IMU_CS_Low();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, HAL_MAX_DELAY);
  IMU_CS_High();

  return rx[1];
}

static void ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t addr;
  uint8_t tx_dummy = 0xFFU;
  uint8_t i;

  addr = reg | 0x80U;

  IMU_CS_Low();
  HAL_SPI_Transmit(&hspi1, &addr, 1U, HAL_MAX_DELAY);

  for (i = 0U; i < len; i++)
  {
    HAL_SPI_TransmitReceive(&hspi1, &tx_dummy, &buf[i], 1U, HAL_MAX_DELAY);
  }

  IMU_CS_High();
}

static void ICM20948_SelectBank(uint8_t bank)
{
  ICM20948_WriteReg(ICM20948_REG_BANK_SEL, (uint8_t)(bank << 4));
}

static uint8_t ICM20948_Init_Minimal(void)
{
  uint8_t who_am_i;

  IMU_CS_High();
  HAL_Delay(100);

  /* 1) First, verify communication */
  who_am_i = ICM20948_ReadReg(ICM20948_WHO_AM_I);

  /*
  UART_SendString("WHO_AM_I=");
  UART_SendInt((int32_t)who_am_i);
  UART_SendString("\r\n");
  */

  if (who_am_i != 0xEAU)
  {
    return 0U;
  }

  /* 2) Release sleep mode in Bank 0 */
  ICM20948_SelectBank(0U);

  /* CLKSEL=1, SLEEP=0 */
  ICM20948_WriteReg(ICM20948_PWR_MGMT_1, 0x01U);

  /* Enable all accel/gyro axes */
  ICM20948_WriteReg(ICM20948_PWR_MGMT_2, 0x00U);

  /* Wait for sensor stabilization after waking up from sleep */
  HAL_Delay(20);

  return 1U;
}

static uint8_t ICM20948_Config_Basic(void)
{
  ICM20948_SelectBank(2U);

  /* Gyro: low range, DLPF enabled */
  ICM20948_WriteReg(ICM20948_GYRO_SMPLRT_DIV, 10U);
  ICM20948_WriteReg(ICM20948_GYRO_CONFIG_1, 0x01U);

  /* Accel: low range, DLPF enabled */
  ICM20948_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00U);
  ICM20948_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 10U);
  ICM20948_WriteReg(ICM20948_ACCEL_CONFIG, 0x01U);

  ICM20948_SelectBank(0U);

  return 1U;
}

static uint8_t ICM20948_ReadRaw(ICM20948_Raw_t *raw)
{
  uint8_t buf[12];

  ICM20948_SelectBank(0U);
  ICM20948_ReadRegs(ICM20948_ACCEL_XOUT_H, buf, 12U);

  raw->ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
  raw->ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
  raw->az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
  raw->gx = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
  raw->gy = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
  raw->gz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

  return 1U;
}

/* ================================================================
 *  Normalization Function
 * ================================================================ */
static void Normalize_Sample(const ICM20948_Raw_t *raw, float out[IMU_NUM_FEATURES])
{
  int16_t vals[IMU_NUM_FEATURES];
  vals[0] = raw->ax;
  vals[1] = raw->ay;
  vals[2] = raw->az;
  vals[3] = raw->gx;
  vals[4] = raw->gy;
  vals[5] = raw->gz;

  for (uint8_t i = 0U; i < IMU_NUM_FEATURES; i++)
{
    out[i] = ((float)vals[i] - ch_min[i]) / ch_range[i];
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  /* ── LED init ── */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* ── Button init ── */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* ── Default LED: GREEN ON (normal state) ── */
  BSP_LED_On(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_Off(LED_RED);

  // UART_SendString("Hello World!\r\n");
  /* ── ICM-20948 initialization ── */
  UART_SendINFO("ICM-20948 init start...");

  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(10);

  imu_ok = ICM20948_Init_Minimal();
  if (imu_ok == 1U)
  {
	UART_SendINFO("ICM-20948 init SUCCESS");
    ICM20948_Config_Basic();
  }
  else
  {
    UART_SendERROR("ICM-20948 init FAIL");
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);
    while (1)
    {
        /* Halt on IMU failure */
    }
  }

  /* ── AI model init check ── */
  /* Already initialized in MX_X_CUBE_AI_Init() */
  UART_SendINFO("AI model ready (X-CUBE-AI 10.2.0)");

  UART_SendString("INFO, Threshold=");
  UART_SendFloat(AE_THRESHOLD);
  UART_SendString("\r\n");

  /* ── Start data collection ── */
  UART_SendINFO("Buffering (10 samples)...");
  sample_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* USER CODE BEGIN BSP */

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ── Collect IMU data every 0.1s (100ms) ── */
    if ((HAL_GetTick() - sample_tick) >= IMU_SAMPLE_PERIOD_MS)
    {
      sample_tick += IMU_SAMPLE_PERIOD_MS;

      /* (1) Read IMU */
      ICM20948_Raw_t raw;
      if (ICM20948_ReadRaw(&raw) != 1U)
    {
        UART_SendERROR("IMU read FAIL");
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
        continue;
      }

      /* (2) Normalize and store in ring buffer */
      Normalize_Sample(&raw, ring_buf[ring_write_idx]);
      ring_write_idx = (ring_write_idx + 1U) % IMU_WINDOW_SIZE;
      total_samples++;

      /* (3) Buffering (before first 10 samples) */
      if (total_samples < IMU_WINDOW_SIZE)
      {
      BSP_LED_Toggle(LED_GREEN);
        continue;
      }

      /* (4) First inference notification (once) */
      if (total_samples == IMU_WINDOW_SIZE)
      {
        UART_SendINFO("Inference started");
        BSP_LED_On(LED_GREEN);
      }

      /* ── (5) Ring buffer -> input array (time-ordered) ── */
      for (uint16_t t = 0U; t < IMU_WINDOW_SIZE; t++)
      {
        uint16_t src = (ring_write_idx + t) % IMU_WINDOW_SIZE;
        for (uint16_t c = 0U; c < IMU_NUM_FEATURES; c++)
        {
          ai_in_buf[t * IMU_NUM_FEATURES + c] = ring_buf[src][c];
        }
      }

      /* ── (6) Run inference ── */
      int res = AI_Run_Model(ai_in_buf, ai_out_buf);

      if (res != 0)
      {
        UART_SendERROR("AI run FAIL");
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
        continue;
      }

      /* ── (7) Compute reconstruction MSE ── */
      float mse = 0.0f;
      uint16_t total_elems = IMU_WINDOW_SIZE * IMU_NUM_FEATURES;  /* 60 */
      for (uint16_t i = 0U; i < total_elems; i++)
      {
        float diff = ai_in_buf[i] - ai_out_buf[i];
        mse += diff * diff;
      }
      mse /= (float)total_elems;

      /* ── (8) Anomaly detection + LED control ── */
      if (mse > AE_THRESHOLD)
      {
        /* Anomaly detected -> LED_RED ON */
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);

        UART_SendString("[ANOMALY] MSE=");
        UART_SendFloat(mse);
        UART_SendString("\r\n");
      }
      else
      {
        /* Normal -> LED_GREEN ON */
        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);

        UART_SendString("[NORMAL ] MSE=");
        UART_SendFloat(mse);
        UART_SendString("\r\n");
      }
    }
    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  MX_X_CUBE_AI_Process();
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
