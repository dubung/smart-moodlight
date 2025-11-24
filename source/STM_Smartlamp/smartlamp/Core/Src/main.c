/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLE_BAUD        9600      // HC-06 기본
#define UNO_BAUD        115200    // UNO와 동일
#define STAT_PERIOD_MS  1000      // SENSOR 프레임 전송 주기
#define PIR_WARMUP_MS   3000
#define PIR_DEBOUNCE_MS 80
#define BLE_LINE_TIMEOUT_MS 400 

#define CMD_GAP_MS       1200
#define OCC_HOLD_MS      30000
#define MOTION_GRACE_MS  5000
#define DARK_MOTION_WINDOW_MS  8000  // 어둠/움직임 서로 8초 안에 오면 켬
#define ON_CMD  "MODE:SOLID;R:255;G:140;B:40;BRI:130\n"
#define OFF_CMD "MODE:OFF\n"

// ── EMA + 적응 임계 ──
#define EMA_ALPHA          0.14f   // 빠른 밝기 추종(0.05~0.2 권장)
#define BASE_ALPHA         0.03f   // 느린 기준선(환경 적응)
#define THR_RATIO          0.64f   // 기준선 대비 어두움 임계 비율
#define THR_MIN_V          0.32f   // 임계 하한(너무 낮게 내려가는 것 방지)
#define THR_MAX_V          1.80f   // 임계 상한
#define HYS_V              0.03f   // 히스테리시스 (밝기 진동 방지)
#define DARK_HOLD_MS       200     // 어두움 검출 후 유효 판정까지 유지 시간
#define BRIGHT_HOLD_MS     5000    // 밝아진 상태가 이 시간 지속되면 보조 소등 조건

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
char     ble_line[256]; 
volatile uint16_t ble_w = 0;
uint8_t  ble_rx, ble6_rx, uno_rx;                  

volatile uint16_t cds_raw_last = 0;

volatile uint8_t  pir_state = 0;
volatile uint32_t pir_last_ms = 0;
uint32_t          boot_ms = 0, last_stat_ms = 0;
volatile uint32_t ble_last_rx_ms = 0;
volatile uint32_t adc_hits = 0;
// AUTO enable/상태
uint8_t  auto_enabled   = 1;
uint8_t  led_on         = 0;
uint32_t last_motion_ms = 0;
uint32_t last_cmd_ms    = 0;

// EMA/임계 관련
float    cds_v_ema   = 0.0f;   // 단기 EMA
float    cds_v_base  = 1.0f;   // 장기 기준선(느리게 학습)
float    dark_thr_v  = 0.8f;   // 현재 사용 중 임계
uint8_t  is_dark     = 0;      // 어두움 상태 머신
uint32_t dark_since_ms   = 0;  // 어두움 시작 시각
uint32_t bright_since_ms = 0;  // 밝음 시작 시각
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void StartUarts(void);
static void StartAdcIt(void);
static void send_sensor_line(void);

static void send_uno_line(const char *s);
static void auto_tick(void);
static inline float clampf(float x, float lo, float hi);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
// 보레이트 보정(필요 시)
if (huart2.Init.BaudRate != BLE_BAUD) { HAL_UART_DeInit(&huart2); huart2.Init.BaudRate = BLE_BAUD; HAL_UART_Init(&huart2); }
if (huart1.Init.BaudRate != UNO_BAUD) { HAL_UART_DeInit(&huart1); huart1.Init.BaudRate = UNO_BAUD; HAL_UART_Init(&huart1); }

StartUarts();
StartAdcIt();
boot_ms = HAL_GetTick();

// 배너(디버그)
const char *bn = "READY@STM BRIDGE+SENSORS\r\n";
HAL_UART_Transmit(&huart2, (uint8_t*)bn, strlen(bn), 100);
HAL_UART_Receive_IT(&huart2, &ble_rx, 1);   // PC 
HAL_UART_Receive_IT(&huart6, &ble6_rx, 1);  // HC-06  
HAL_UART_Receive_IT(&huart1, &uno_rx, 1);   // UNO

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


uint32_t now = HAL_GetTick();

uint16_t n = 0;
char tmp[256];

__disable_irq();  // 아주 짧게: 길이 스냅샷 + 복사만
if (ble_w > 0 && (uint32_t)(now - ble_last_rx_ms) > BLE_LINE_TIMEOUT_MS) {
  n = ble_w;
  if (n > sizeof(tmp)) n = sizeof(tmp);
  memcpy(tmp, ble_line, n);
  ble_w = 0;
}
__enable_irq();

if (n) {
  HAL_UART_Transmit(&huart1, (uint8_t*)tmp, n, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);  // 줄끝 보정
}

// --- 2) 센서 프레임 주기 송신 ---
now = HAL_GetTick();
if ((uint32_t)(now - last_stat_ms) >= STAT_PERIOD_MS) {
  last_stat_ms = now;
  send_sensor_line();            // 블로킹 길지 않게 유지
}
auto_tick();
// --- 3) 짧게 쉼 ---
HAL_Delay(2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_Pin */
  GPIO_InitStruct.Pin = PIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void StartUarts(void){
  HAL_UART_Receive_IT(&huart2, &ble_rx, 1);   // PC
	HAL_UART_Receive_IT(&huart6, &ble6_rx, 1); // HC-06 수신 시작
  HAL_UART_Receive_IT(&huart1, &uno_rx, 1);   // (옵션) UNO 수신
}

static void StartAdcIt(void){
  HAL_ADC_Start_IT(&hadc1);                   // 연속 변환 + EOC마다 콜백
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
 if (huart == &huart2) {                 // PC/푸티
    ble_last_rx_ms = HAL_GetTick();
    char c = (char)ble_rx;

    if (c=='0'||c=='1'||c=='2') {
      HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, 20);
      uint8_t lf='\n'; HAL_UART_Transmit(&huart1, &lf, 1, 20);
    } else if (c=='\r' || c=='\n') {
      if (ble_w>0){
        HAL_UART_Transmit(&huart1, (uint8_t*)ble_line, ble_w, 50);
        uint8_t lf='\n'; HAL_UART_Transmit(&huart1, &lf, 1, 20);
        ble_w = 0;
      }
    } else {
      if (ble_w < sizeof(ble_line)-1) ble_line[ble_w++] = c;
      else ble_w = 0;
    }

    HAL_UART_Receive_IT(&huart2, &ble_rx, 1);
  }
  else if (huart == &huart6) {            // HC-06(BLE)
    ble_last_rx_ms = HAL_GetTick();
    char c = (char)ble6_rx;

    if (c=='0'||c=='1'||c=='2') {
      HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, 20);
      uint8_t lf='\n'; HAL_UART_Transmit(&huart1, &lf, 1, 20);
    } else if (c=='\r' || c=='\n') {
      if (ble_w>0){
        HAL_UART_Transmit(&huart1, (uint8_t*)ble_line, ble_w, 50);
        uint8_t lf='\n'; HAL_UART_Transmit(&huart1, &lf, 1, 20);
        ble_w = 0;
      }
    } else {
      if (ble_w < sizeof(ble_line)-1) ble_line[ble_w++] = c;
      else ble_w = 0;
    }

    HAL_UART_Receive_IT(&huart6, &ble6_rx, 1);
  }
  else if (huart == &huart1) {            // UNO → STM (지금은 무시)
    HAL_UART_Receive_IT(&huart1, &uno_rx, 1);
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if (hadc == &hadc1) {
    cds_raw_last = (uint16_t)HAL_ADC_GetValue(&hadc1);
		 adc_hits++;
		
  }
}


static void send_sensor_line(void){
  // UNO로 전송할 센서 프레임 (UNO는 이 라인을 Wi-Fi로 서버에 올림)
  char s[96];
  float v = (cds_raw_last / 4095.0f) * 3.3f;
  int n = snprintf(s, sizeof(s), "SENSOR@PIR=%u@CdS_raw=%u@V=%.3f\r\n",
                   pir_state, cds_raw_last, v);
  HAL_UART_Transmit(&huart1, (uint8_t*)s, n, 100);
}
static inline float clampf(float x, float lo, float hi){
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void send_uno_line(const char *s){
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 50);
}

static void auto_tick(void){
  if (!auto_enabled) return;
  uint32_t now = HAL_GetTick();

  // 1) ADC → 전압
	uint16_t raw = cds_raw_last;  
  float v = (cds_raw_last / 4095.0f) * 3.3f;

  // 2) EMA 업데이트 (첫 시작 시 초기화)
  if (cds_v_ema == 0.0f) {
    cds_v_ema  = v;
    cds_v_base = v;
  } else {
    cds_v_ema  += EMA_ALPHA  * (v - cds_v_ema);   // 빠른 추종
  // 어둠이 아닐 때만(밝을 때만) 기준선 천천히 학습 → 임계가 어둠 쪽으로 같이 내려가지 않음
		if (!is_dark) {
			cds_v_base += BASE_ALPHA * (cds_v_ema - cds_v_base);
}
  }

  // 3) 적응 임계 + 히스테리시스
  dark_thr_v = clampf(THR_RATIO * cds_v_base, THR_MIN_V, THR_MAX_V);

  if (is_dark) {
    if (cds_v_ema > dark_thr_v + HYS_V) {
      is_dark = 0;
      bright_since_ms = now;
    }
  } else {
    if (cds_v_ema < dark_thr_v - HYS_V) {
      is_dark = 1;
      dark_since_ms = now;
    }
  }
// 4) 켜기 조건:
	bool dark_ready    = is_dark && (now - dark_since_ms) >= DARK_HOLD_MS;
	bool recent_motion = (pir_state == 1) || ((now - last_motion_ms) <= DARK_MOTION_WINDOW_MS);

if (!led_on && dark_ready && recent_motion && (now - last_cmd_ms) > CMD_GAP_MS) {
  send_uno_line(ON_CMD);
  led_on      = 1;
  last_cmd_ms = now;
}
  // 5) 끄기 조건:
  //    (a) 무동작이 OCC_HOLD_MS 초과  또는
  //    (b) 충분히 밝아진 상태가 BRIGHT_HOLD_MS 지속
  if (led_on &&
      ((now - last_motion_ms) > OCC_HOLD_MS ||
       (!is_dark && (now - bright_since_ms) > BRIGHT_HOLD_MS)) &&
      (now - last_cmd_ms) > CMD_GAP_MS) {

    send_uno_line(OFF_CMD);
    led_on      = 0;
    last_cmd_ms = now;
  }
			// ===== 디버그 로그 (1초에 한 번) =====
static uint32_t last_dbg=0;
if (now - last_dbg > 1000) {
  last_dbg = now;
  char s[160];
  int n = snprintf(s, sizeof(s),
    "AUTO hits=%lu v=%.2f ema=%.2f base=%.2f thr=%.2f dark=%d pir=%d on=%d\r\n",
    (unsigned long)adc_hits,
    (float)((cds_raw_last/4095.0f)*3.3f),
    cds_v_ema, cds_v_base, dark_thr_v, is_dark, pir_state, led_on);
  HAL_UART_Transmit(&huart2, (uint8_t*)s, n, 200);
}
// ====================================
}

// PIR 인터럽트: 마지막 움직임 기록
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if (GPIO_Pin == PIR_Pin) {
    uint32_t t = HAL_GetTick();

    // 1) 워밍업 & 디바운스
    if (t - boot_ms < PIR_WARMUP_MS) return;
    if (t - pir_last_ms < PIR_DEBOUNCE_MS) return;
    pir_last_ms = t;

    // 2) 현재 레벨 읽고 상태 갱신
    uint8_t level = (HAL_GPIO_ReadPin(PIR_GPIO_Port, PIR_Pin) == GPIO_PIN_SET) ? 1 : 0;
    pir_state = level;

    // 3) 상승(감지) 시각 기록 → AUTO 로직이 이걸로 On/Off 판단
    if (level) {
      last_motion_ms = t;
    }

    // (선택) 디버그
     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc){
  if (hadc == &hadc1) {
    // OVR 플래그 클리어 후 재가동
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    HAL_ADC_Stop_IT(&hadc1);
    HAL_ADC_Start_IT(&hadc1);
  }
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
