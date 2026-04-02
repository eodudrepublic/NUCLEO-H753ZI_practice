/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Wake-word MFCC data collector (Button-triggered, 1.5s)
  *                   Board : NUCLEO-H753ZI
  *                   Mic   : Adafruit ada-3492 (PDM, DFSDM1 Channel 1)
  *                   Output: USART3 (ST-Link VCP, 115200 baud)
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
#include "dfsdm.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ─────────────────────────────────────────────
 *  DFSDM / PCM config
 *
 *  Audio clock (DFSDM1 PLL) = 61.44 MHz
 *  CKOUT  = 61.44 MHz / 60 = 1.024 MHz   (PDM clock to mic)
 *  Sinc4, FOSR = 64, IOSR = 1
 *  PCM    = 1.024 MHz / 64 = 16,000 Hz    (exact)
 *
 *  DFSDM output: 24-bit signed in bits[31:8] of int32_t.
 *  Sinc4 FOSR=64 theoretical max = 64^4 = 2^24
 *  After RightBitShift=4 → max ±2^20 in 24-bit field.
 *  Convert to int16: (raw >> 8) >> 4 = raw >> 12, then clamp.
 * ───────────────────────────────────────────── */
#define DFSDM_DMA_BUF_SIZE   256U       /* int32_t units, circular DMA */
#define PCM_SAMPLE_RATE      16000U     /* exact: 61.44MHz / 60 / 64 */

/* ── Recording: 1.5 seconds ── */
#define RECORD_DURATION_MS   1500U
#define PCM_RECORD_SAMPLES   ((PCM_SAMPLE_RATE * RECORD_DURATION_MS) / 1000U)
                                        /* 16000 * 1500 / 1000 = 24000 */

/* ── MFCC config ── */
#define FFT_SIZE             1024U
#define HOP_SIZE             512U
#define MEL_FILTERS          15U
#define MFCC_COEFFS          15U

/* MFCC frames from 24000 PCM samples:
 * floor((24000 - 1024) / 512) + 1 = floor(22976/512) + 1 = 44 + 1 = 45 */
#define MFCC_TOTAL_FRAMES    (((PCM_RECORD_SAMPLES - FFT_SIZE) / HOP_SIZE) + 1U)

#define MEL_FMIN_HZ          20.0f
#define MEL_FMAX_HZ          4000.0f
#define PREEMPHASIS_ALPHA    0.97f
#define EPSILON_F            1.0e-10f
#define PI_F                 3.14159265358979f

/* ── DFSDM → int16 conversion ──
 *  Sinc4, FOSR=64, RightBitShift=4
 *  Effective bits = 4*log2(64) - 4 = 24 - 4 = 20 bits
 *  raw >> 8 extracts 24-bit signed data (up to ±2^20)
 *  >> 4 additional to fit 16-bit (up to ±2^16, clamp handles overflow) */
#define DFSDM_TO_PCM_SHIFT  12U         /* total: raw >> 12 = (raw>>8)>>4 */

/* ── State machine ── */
#define STATE_IDLE           0U
#define STATE_RECORDING      1U
#define STATE_PROCESSING     2U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */

/* ── DFSDM DMA buffer (AXI-SRAM for DMA access) ──
 *
 * Linker script (.ld) must contain:
 *   .RAM_D1 (NOLOAD) : { . = ALIGN(32); *(.RAM_D1) *(.RAM_D1.*) } >RAM_D1
 */
ALIGN_32BYTES(static int32_t dfsdm_dma_buf[DFSDM_DMA_BUF_SIZE])
    __attribute__((section(".RAM_D1")));

/* ── PCM recording buffer (1.5s = 24000 samples) ── */
static int16_t  pcm_buf[PCM_RECORD_SAMPLES];
static volatile uint32_t pcm_write_idx    = 0U;
static volatile uint8_t  state            = STATE_IDLE;

/* ── Timing ── */
static volatile uint32_t record_start_tick = 0U;
static volatile uint32_t record_end_tick   = 0U;

/* ── MFCC frontend ── */
static uint8_t frontend_ok = 0U;
static arm_rfft_fast_instance_f32 rfft_inst;
static float   hamming_win[FFT_SIZE];
static uint16_t mel_edges[MEL_FILTERS + 2U];

/* ── MFCC working buffers ── */
static float frame_f32[FFT_SIZE];
static float fft_f32[FFT_SIZE];
static float psd[(FFT_SIZE / 2U) + 1U];
static float mel_e[MEL_FILTERS];
static float mfcc_vec[MFCC_COEFFS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void        UART_SendString(const char *str);
static void UART_SendChar(char c);
static void UART_SendInt(int32_t value);
static void UART_SendFloat6(float value);
static void UART_SendINFO(const char *str);
static void UART_SendERROR(const char *str);

static void Process_DFSDM_Block(const int32_t *buf, uint16_t len);

static float HzToMel(float hz);
static float MelToHz(float mel);
static void  MFCC_Init(void);
static void  MFCC_ExtractFrame(const int16_t *pcm_start,
                                uint32_t prev_sample_idx,
                                float *out_mfcc);
static void  MFCC_ProcessAndSend(void);
static void  UART_SendMFCCFrame(uint32_t idx, const float *mfcc, uint32_t n);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================================
 *  UART helpers
 * ================================================================ */
void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)str,
                    (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void UART_SendChar(char c)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&c, 1U, HAL_MAX_DELAY);
}

static void UART_SendInt(int32_t value)
{
  char tmp[12];
  uint32_t i = 0U;
  uint32_t mag;

  if (value < 0) { UART_SendChar('-'); mag = (uint32_t)(-value); }
  else           { mag = (uint32_t)value; }

  do { tmp[i++] = (char)('0' + (mag % 10U)); mag /= 10U; } while (mag > 0U);
  while (i > 0U) { UART_SendChar(tmp[--i]); }
}

/* 6 decimal places for training-quality precision */
static void UART_SendFloat6(float value)
{
  if (value < 0.0f) { UART_SendChar('-'); value = -value; }

  uint32_t int_part  = (uint32_t)value;
  uint32_t frac_part = (uint32_t)(((double)(value - (float)int_part) * 1000000.0) + 0.5);

  if (frac_part >= 1000000U) { int_part += 1U; frac_part -= 1000000U; }

  UART_SendInt((int32_t)int_part);
  UART_SendChar('.');

  char frac_str[7];
  for (int d = 5; d >= 0; d--)
  {
    frac_str[d] = (char)('0' + (frac_part % 10U));
    frac_part /= 10U;
  }
  frac_str[6] = '\0';
  UART_SendString(frac_str);
}

static void UART_SendINFO(const char *str)
{
  UART_SendString("INFO,");
  UART_SendString(str);
  UART_SendString("\r\n");
}

static void UART_SendERROR(const char *str)
{
  UART_SendString("ERROR,");
  UART_SendString(str);
  UART_SendString("\r\n");
}

/* Format: MFCC,<frame_idx>,c0,c1,...,c14\r\n */
static void UART_SendMFCCFrame(uint32_t idx, const float *mfcc, uint32_t n)
{
  UART_SendString("MFCC,");
  UART_SendInt((int32_t)idx);
  for (uint32_t i = 0U; i < n; i++)
  {
    UART_SendChar(',');
    UART_SendFloat6(mfcc[i]);
  }
  UART_SendString("\r\n");
}

/* ================================================================
 *  MFCC frontend init
 * ================================================================ */
static float HzToMel(float hz)
{
  return 2595.0f * log10f(1.0f + (hz / 700.0f));
}

static float MelToHz(float mel)
{
  return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static void MFCC_Init(void)
{
  if (arm_rfft_fast_init_f32(&rfft_inst, FFT_SIZE) != ARM_MATH_SUCCESS)
  {
    UART_SendERROR("rfft_init_fail");
    return;
  }

  /* Hamming window */
  for (uint32_t n = 0U; n < FFT_SIZE; n++)
  {
    hamming_win[n] = 0.54f - 0.46f * cosf(
        (2.0f * PI_F * (float)n) / (float)(FFT_SIZE - 1U));
  }

  /* Mel filter bank edges */
  float mel_lo = HzToMel(MEL_FMIN_HZ);
  float mel_hi = HzToMel(MEL_FMAX_HZ);

  for (uint32_t i = 0U; i < (MEL_FILTERS + 2U); i++)
  {
    float t   = (float)i / (float)(MEL_FILTERS + 1U);
    float mel = mel_lo + t * (mel_hi - mel_lo);
    float hz  = MelToHz(mel);
    uint16_t bin = (uint16_t)((float)(FFT_SIZE + 1U) * hz / (float)PCM_SAMPLE_RATE);

    if (bin > (FFT_SIZE / 2U))
      bin = (FFT_SIZE / 2U);

    if ((i > 0U) && (bin <= mel_edges[i - 1U]))
    {
      bin = mel_edges[i - 1U] + 1U;
      if (bin > (FFT_SIZE / 2U))
        bin = (FFT_SIZE / 2U);
    }

    mel_edges[i] = bin;
  }

  frontend_ok = 1U;
}

/* ================================================================
 *  Extract one MFCC frame
 * ================================================================ */
static void MFCC_ExtractFrame(const int16_t *pcm_start,
                               uint32_t prev_sample_idx,
                               float *out_mfcc)
{
  float prev_x;

  /* Previous sample for pre-emphasis filter */
  if (prev_sample_idx == 0xFFFFFFFFU)
  {
    prev_x = 0.0f;  /* no prior sample */
  }
  else
  {
    prev_x = (float)pcm_buf[prev_sample_idx] / 32768.0f;
  }

  /* Pre-emphasis + Hamming window */
  for (uint32_t n = 0U; n < FFT_SIZE; n++)
  {
    float x = (float)pcm_start[n] / 32768.0f;
    float y = x - PREEMPHASIS_ALPHA * prev_x;
    prev_x = x;
    frame_f32[n] = y * hamming_win[n];
  }

  /* FFT */
  arm_rfft_fast_f32(&rfft_inst, frame_f32, fft_f32, 0U);

  /* Power spectrum */
  psd[0]              = fft_f32[0] * fft_f32[0];
  psd[FFT_SIZE / 2U]  = fft_f32[1] * fft_f32[1];
  for (uint32_t k = 1U; k < (FFT_SIZE / 2U); k++)
  {
    float re = fft_f32[2U * k];
    float im = fft_f32[2U * k + 1U];
    psd[k] = re * re + im * im;
  }

  /* Mel filter bank */
  for (uint32_t m = 0U; m < MEL_FILTERS; m++)
  {
    uint32_t l = mel_edges[m];
    uint32_t c = mel_edges[m + 1U];
    uint32_t r = mel_edges[m + 2U];
    float sum = 0.0f;

    if (c <= l) c = l + 1U;
    if (r <= c) r = c + 1U;

    for (uint32_t k = l; k < c; k++)
      sum += psd[k] * (float)(k - l) / (float)(c - l);
    for (uint32_t k = c; k < r; k++)
      sum += psd[k] * (float)(r - k) / (float)(r - c);

    mel_e[m] = logf(sum + EPSILON_F);
  }

  /* DCT-II → MFCC + per-frame cepstral mean subtraction */
  float mean = 0.0f;
  for (uint32_t c = 0U; c < MFCC_COEFFS; c++)
  {
    float acc = 0.0f;
    for (uint32_t m = 0U; m < MEL_FILTERS; m++)
    {
      acc += mel_e[m] * cosf(
          PI_F * (float)c * ((float)m + 0.5f) / (float)MEL_FILTERS);
    }
    out_mfcc[c] = acc;
    mean += acc;
  }
  mean /= (float)MFCC_COEFFS;
  for (uint32_t c = 0U; c < MFCC_COEFFS; c++)
    out_mfcc[c] -= mean;
}

/* ================================================================
 *  Extract all MFCC frames and send via USART
 *
 *  Protocol:
 *    MFCC_START,<frames>,<coeffs>,<sr>,<fft>,<hop>\r\n
 *    MFCC,<idx>,c0,c1,...,c14\r\n     (× frames)
 *    MFCC_END\r\n
 * ================================================================ */
static void MFCC_ProcessAndSend(void)
{
  uint32_t collected = pcm_write_idx;
  uint32_t n_frames;

  if (collected < FFT_SIZE)
  {
    UART_SendERROR("too_few_samples");
    return;
  }

  n_frames = ((collected - FFT_SIZE) / HOP_SIZE) + 1U;

  /* Header */
  UART_SendString("MFCC_START,");
  UART_SendInt((int32_t)n_frames);
  UART_SendChar(',');
  UART_SendInt((int32_t)MFCC_COEFFS);
  UART_SendChar(',');
  UART_SendInt((int32_t)PCM_SAMPLE_RATE);
  UART_SendChar(',');
  UART_SendInt((int32_t)FFT_SIZE);
  UART_SendChar(',');
  UART_SendInt((int32_t)HOP_SIZE);
  UART_SendString("\r\n");

  /* Frames */
  for (uint32_t f = 0U; f < n_frames; f++)
  {
    uint32_t start = f * HOP_SIZE;
    uint32_t prev  = (start == 0U) ? 0xFFFFFFFFU : (start - 1U);

    MFCC_ExtractFrame(&pcm_buf[start], prev, mfcc_vec);
    UART_SendMFCCFrame(f, mfcc_vec, MFCC_COEFFS);
  }

  /* Footer */
  UART_SendString("MFCC_END\r\n");
}

/* ================================================================
 *  DFSDM → int16 PCM conversion
 *
 *  DFSDM register format: 24-bit signed data in bits[31:8].
 *
 *  With Sinc4 FOSR=64, RightBitShift=4:
 *    Effective dynamic range = 4×log2(64) - 4 = 20 bits
 *    raw >> 8 gives 24-bit signed value (up to ±2^20)
 *    >> 4 additional to map to ~16-bit range
 *    Total: raw >> 12, then clamp to ±32767
 * ================================================================ */
static void Process_DFSDM_Block(const int32_t *buf, uint16_t len)
{
  if (state != STATE_RECORDING) return;

  for (uint16_t i = 0U; i < len; i++)
  {
    if (pcm_write_idx >= PCM_RECORD_SAMPLES)
    {
      record_end_tick = HAL_GetTick();
      state = STATE_PROCESSING;
      return;
    }

    int32_t s16 = buf[i] >> DFSDM_TO_PCM_SHIFT;

    if (s16 >  32767) s16 =  32767;
    if (s16 < -32768) s16 = -32768;

    pcm_buf[pcm_write_idx++] = (int16_t)s16;
  }
}

/* ================================================================
 *  DFSDM DMA callbacks (Circular Half / Complete)
 * ================================================================ */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if (hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)&dfsdm_dma_buf[0],
        (int32_t)((DFSDM_DMA_BUF_SIZE / 2U) * sizeof(int32_t)));

    Process_DFSDM_Block(&dfsdm_dma_buf[0], DFSDM_DMA_BUF_SIZE / 2U);
  }
}

void HAL_DFSDM_FilterRegConvCpltCallback(
    DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if (hdfsdm_filter->Instance == DFSDM1_Filter0)
  {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)&dfsdm_dma_buf[DFSDM_DMA_BUF_SIZE / 2U],
        (int32_t)((DFSDM_DMA_BUF_SIZE / 2U) * sizeof(int32_t)));

    Process_DFSDM_Block(&dfsdm_dma_buf[DFSDM_DMA_BUF_SIZE / 2U],
                         DFSDM_DMA_BUF_SIZE / 2U);
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
  MX_DFSDM1_Init();
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

  /* ── MFCC init ── */
  UART_SendINFO("mfcc_init_start");
  MFCC_Init();

  if (!frontend_ok)
  {
    UART_SendERROR("mfcc_init_fail");
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);
    while (1) { }
  }
  UART_SendINFO("mfcc_init_ok");

  /* ── DFSDM DMA start ── */
  UART_SendINFO("dfsdm_start");

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0,
                                         dfsdm_dma_buf,
                                         DFSDM_DMA_BUF_SIZE) != HAL_OK)
  {
    UART_SendERROR("dfsdm_dma_fail");
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);
    while (1)
    {
      /* Halt on DFSDM failure */
    }
  }
  UART_SendINFO("dfsdm_ok");

  /* ── Config printout ── */
  UART_SendString("CONFIG,sr=");
  UART_SendInt((int32_t)PCM_SAMPLE_RATE);
  UART_SendString(",fft=");
  UART_SendInt((int32_t)FFT_SIZE);
  UART_SendString(",hop=");
  UART_SendInt((int32_t)HOP_SIZE);
  UART_SendString(",mels=");
  UART_SendInt((int32_t)MEL_FILTERS);
  UART_SendString(",mfcc=");
  UART_SendInt((int32_t)MFCC_COEFFS);
  UART_SendString(",rec_samples=");
  UART_SendInt((int32_t)PCM_RECORD_SAMPLES);
  UART_SendString(",rec_ms=");
  UART_SendInt((int32_t)RECORD_DURATION_MS);
  UART_SendString(",frames=");
  UART_SendInt((int32_t)MFCC_TOTAL_FRAMES);
  UART_SendString("\r\n");

  UART_SendINFO("ready_press_button");

  /* USER CODE END 2 */

  /* USER CODE BEGIN BSP */

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ── Button → start 1.5s recording ── */
    if (BspButtonState == BUTTON_PRESSED)
    {
      BspButtonState = BUTTON_RELEASED;

      if (state == STATE_IDLE)
      {
        pcm_write_idx = 0U;
        memset(pcm_buf, 0, sizeof(pcm_buf));

        state = STATE_RECORDING;
        record_start_tick = HAL_GetTick();

        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_BLUE);      /* BLUE = recording */
        BSP_LED_Off(LED_RED);

        UART_SendINFO("rec_start");
      }
    }

    /* ── Recording complete → MFCC extract & send ── */
    if (state == STATE_PROCESSING)
    {
      BSP_LED_Off(LED_BLUE);
      BSP_LED_On(LED_RED);          /* RED = processing */

      uint32_t elapsed = record_end_tick - record_start_tick;

      UART_SendString("REC_DONE,samples=");
      UART_SendInt((int32_t)pcm_write_idx);
      UART_SendString(",elapsed_ms=");
      UART_SendInt((int32_t)elapsed);
      UART_SendString(",effective_sr=");
      if (elapsed > 0U)
        UART_SendInt((int32_t)((pcm_write_idx * 1000U) / elapsed));
      else
        UART_SendInt(0);
      UART_SendString("\r\n");

      MFCC_ProcessAndSend();

      BSP_LED_Off(LED_RED);
      BSP_LED_On(LED_GREEN);        /* GREEN = idle */

      UART_SendINFO("ready_press_button");
      state = STATE_IDLE;
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
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
