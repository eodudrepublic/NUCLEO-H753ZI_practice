/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PDM Microphone 2-second recorder (Button-triggered)
  *                   Board : NUCLEO-H753ZI
  *                   Mic   : Adafruit ada-3492 (PDM, SAI1)
  *                   Output: USART3 (ST-Link VCP)
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
#include "dma.h"
#include "sai.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ── PDM / PCM 설정 ──
 *
 * SAI1이 PDM 클럭을 생성하고, DMA가 PDM 비트스트림을 pdm_buf에 채움.
 * 소프트웨어에서 popcount 기반 CIC-1차 데시메이션으로 PCM 변환.
 *
 * DECIMATION_RATIO = 128  →  PDM 128비트 = PCM 1샘플
 * PDM_HW_PER_PCM   = 128/16 = 8  (half-word 8개 = 128비트)
 *
 * PCM_SAMPLE_RATE   = 16000 Hz (대략, 실제는 SAI 클럭 분주에 따름)
 * RECORD_SECONDS    = 2
 * PCM_TOTAL_SAMPLES = 32000
 */

#define PDM_BUF_SIZE          256U
#define DECIMATION_RATIO      128U
#define PDM_HW_PER_PCM        (DECIMATION_RATIO / 16U)   /* 8 */

#define PCM_SAMPLE_RATE       16000U
#define RECORD_SECONDS        2U
#define PCM_TOTAL_SAMPLES     (PCM_SAMPLE_RATE * RECORD_SECONDS)  /* 32000 */

/* 상태 머신 */
#define STATE_IDLE            0U
#define STATE_RECORDING       1U
#define STATE_SENDING         2U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */

/*
 * [핵심] STM32H7 DMA 버퍼는 AXI-SRAM에 배치해야 합니다.
 *
 *   DTCM (0x20000000)      → DMA 접근 불가 ✘
 *   AXI-SRAM (0x24000000)  → DMA 접근 가능 ✔
 *
 * 링커 스크립트(.ld)에 아래 섹션이 없으면 추가하세요:
 *
 *   .RAM_D1 (NOLOAD) :
 *   {
 *     . = ALIGN(32);
 *     *(.RAM_D1)
 *     *(.RAM_D1.*)
 *   } >RAM_D1
 */
ALIGN_32BYTES(static uint16_t pdm_buf[PDM_BUF_SIZE])
    __attribute__((section(".RAM_D1")));

static int16_t  pcm_buf[PCM_TOTAL_SAMPLES];
static volatile uint32_t pcm_write_idx = 0U;
static volatile uint8_t  state         = STATE_IDLE;

/* PDM 데시메이션 누산기 */
static uint32_t pdm_ones_accum = 0U;
static uint8_t  pdm_hw_count   = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_SendString(const char *str);

static void UART_SendChar(char c);
static void UART_SendInt(int32_t value);
static void UART_SendFloat(float value);
static void UART_SendINFO(const char *str);
static void UART_SendERROR(const char *str);
static void UART_SendDATA(const char *label, int32_t value);

static uint8_t  Popcount16(uint16_t x);
static void     Process_PDM_Block(const uint16_t *buf, uint16_t len);
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

static void UART_SendDATA(const char *label, int32_t value)
{
  UART_SendString("DATA, ");
  UART_SendString(label);
  UART_SendChar('=');
  UART_SendInt(value);
  UART_SendString("\r\n");
}

/* ================================================================
 *  PDM -> PCM 데시메이션 (CIC 1차 / popcount)
 * ================================================================ */
static uint8_t Popcount16(uint16_t x)
{
  x = x - ((x >> 1) & 0x5555U);
  x = (x & 0x3333U) + ((x >> 2) & 0x3333U);
  x = (x + (x >> 4)) & 0x0F0FU;
  return (uint8_t)((x + (x >> 8)) & 0x00FFU);
}

static void Process_PDM_Block(const uint16_t *buf, uint16_t len)
{
  if (state != STATE_RECORDING)
  {
    return;
  }

  for (uint16_t i = 0U; i < len; i++)
  {
    pdm_ones_accum += Popcount16(buf[i]);
    pdm_hw_count++;

    if (pdm_hw_count >= PDM_HW_PER_PCM)
    {
      /* PCM 샘플 1개 완성 */
      int32_t centered = (int32_t)pdm_ones_accum
                       - (int32_t)(DECIMATION_RATIO / 2U);

      int32_t scaled = centered * (int32_t)(32767 / (DECIMATION_RATIO / 2U));

      if (scaled >  32767) scaled =  32767;
      if (scaled < -32768) scaled = -32768;

      if (pcm_write_idx < PCM_TOTAL_SAMPLES)
      {
        pcm_buf[pcm_write_idx] = (int16_t)scaled;
        pcm_write_idx++;
      }

      if (pcm_write_idx >= PCM_TOTAL_SAMPLES)
      {
        state = STATE_SENDING;
      }

      pdm_ones_accum = 0U;
      pdm_hw_count   = 0U;
    }
  }
}

/* ================================================================
 *  SAI DMA 콜백 (Circular Half / Complete)
 * ================================================================ */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)&pdm_buf[0],
        (int32_t)((PDM_BUF_SIZE / 2U) * sizeof(uint16_t)));

    Process_PDM_Block(&pdm_buf[0], PDM_BUF_SIZE / 2U);
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)&pdm_buf[PDM_BUF_SIZE / 2U],
        (int32_t)((PDM_BUF_SIZE / 2U) * sizeof(uint16_t)));

    Process_PDM_Block(&pdm_buf[PDM_BUF_SIZE / 2U], PDM_BUF_SIZE / 2U);
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
  MX_USART3_UART_Init();
  MX_SAI1_Init();
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

  /* ================================================================
   *  부팅 확인 메시지
   * ================================================================ */
  /* ── 마이크(SAI) 초기화 안내 ── */
  UART_SendINFO("SAI1 PDM init start...");

  /* SAI DMA 시작 (Circular 모드) */
  HAL_StatusTypeDef sai_status;
  sai_status = HAL_SAI_Receive_DMA(&hsai_BlockA1,
                                    (uint8_t *)pdm_buf,
                                    PDM_BUF_SIZE);

  if (sai_status != HAL_OK)
  {
    UART_SendERROR("SAI DMA start FAIL");
    UART_SendString("ERROR, HAL_Status=");
    UART_SendInt((int32_t)sai_status);
    UART_SendString("\r\n");

    /* SAI 에러 상태 출력 */
    UART_SendString("ERROR, SAI_ErrorCode=0x");
    {
      uint32_t err = HAL_SAI_GetError(&hsai_BlockA1);
      /* 간단한 hex 출력 */
      char hex[9];
      uint32_t idx = 0U;
      for (int s = 28; s >= 0; s -= 4)
      {
        uint8_t nibble = (uint8_t)((err >> s) & 0x0FU);
        hex[idx++] = (nibble < 10U) ? ('0' + nibble) : ('A' + nibble - 10U);
      }
      hex[idx] = '\0';
      UART_SendString(hex);
    }
    UART_SendString("\r\n");

    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);
    while (1)
    {
      /* Halt on SAI failure */
    }
  }

  UART_SendINFO("SAI1 PDM init SUCCESS");
  UART_SendINFO("SAI DMA started (circular)");

  /* ── 설정 정보 출력 ── */
  UART_SendString("INFO, PDM_BUF_SIZE=");
  UART_SendInt((int32_t)PDM_BUF_SIZE);
  UART_SendString("\r\n");

  UART_SendString("INFO, PCM_SAMPLE_RATE=");
  UART_SendInt((int32_t)PCM_SAMPLE_RATE);
  UART_SendString("Hz\r\n");

  UART_SendString("INFO, RECORD_SECONDS=");
  UART_SendInt((int32_t)RECORD_SECONDS);
  UART_SendString("s\r\n");

  UART_SendString("INFO, PCM_TOTAL_SAMPLES=");
  UART_SendInt((int32_t)PCM_TOTAL_SAMPLES);
  UART_SendString("\r\n");

  UART_SendINFO("Ready. Press USER button to record 2 seconds");

  /* USER CODE END 2 */

  /* USER CODE BEGIN BSP */

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER 버튼 눌림 -> 2초 녹음 시작 */
    if (BspButtonState == BUTTON_PRESSED)
    {
      BspButtonState = BUTTON_RELEASED;

      if (state == STATE_IDLE)
      {
        pcm_write_idx   = 0U;
        pdm_ones_accum  = 0U;
        pdm_hw_count    = 0U;
        memset(pcm_buf, 0, sizeof(pcm_buf));

        state = STATE_RECORDING;

        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_BLUE);
        BSP_LED_Off(LED_RED);

        UART_SendINFO("USER button pressed");
        UART_SendINFO("Recording start (2 seconds)");
      }
    }

    /* 녹음 완료 -> USART 전송 */
    if (state == STATE_SENDING)
    {
      BSP_LED_Off(LED_BLUE);
      BSP_LED_On(LED_GREEN);

      UART_SendINFO("Recording complete");

      UART_SendString("INFO, Collected_Samples=");
      UART_SendInt((int32_t)pcm_write_idx);
      UART_SendString("\r\n");

      UART_SendINFO("Sending PCM data...");

      UART_SendString("PCM_START,");
      UART_SendInt((int32_t)PCM_TOTAL_SAMPLES);
      UART_SendChar(',');
      UART_SendInt((int32_t)PCM_SAMPLE_RATE);
      UART_SendString("\r\n");

      for (uint32_t i = 0U; i < PCM_TOTAL_SAMPLES; i++)
      {
        UART_SendInt((int32_t)pcm_buf[i]);
        UART_SendString("\r\n");
      }

      UART_SendString("PCM_END\r\n");

      UART_SendINFO("Send complete");
      UART_SendINFO("Press USER button to record again");

      state = STATE_IDLE;
    }
  }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
