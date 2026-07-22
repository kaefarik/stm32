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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SET_OK = 0,
    SET_ERR_TOO_LOW,     // ниже физического минимума платы
    SET_ERR_TOO_HIGH,    // выше физического максимума платы
	SET_ERR_NO_VOLTAGE,		// для напряжения v < 1.0f
    SET_ERR_NO_LOAD       // для тока: I_out1 = 0, сопротивление нагрузки неизвестно
} SetResult_t;

typedef struct {
    float Kp;             // Пропорциональный коэффициент
    float Ki;             // Интегральный коэффициент
    float Kd;             // Дифференциальный коэффициент
    float integrator;     // Накопленная интегральная сумма
    float prev_error;     // Ошибка на предыдущем шаге
    float dt;             // Шаг дискретизации в секундах (ADJUST_INTERVAL_MS / 1000.0)
    float out_min;        // Минимальное ограничение выхода (0.0)
    // Максимальное ограничение выхода (255.0)
    float out_max;
} PID_Controller_t;

#define EE_MAGIC_VALID   0xCAFEBABE   // маркер "данные во flash валидны"

typedef struct {
    uint32_t magic;
    float    voltage_k;
    float    voltage_b;
    float    current_k;
    float    current_b;

    float pid_Kp;
    float pid_Ki;
    float pid_Kd;

    float pid_i_Kp;
    float pid_i_Ki;
    float pid_i_Kd;
} eeStorage_t;

typedef enum {
    REG_NONE = 0,
    REG_VOLTAGE,
    REG_CURRENT
} RegMode_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define POT_STEP            1       // на сколько единиц кода двигаем за один шаг
#define ADJUST_INTERVAL_MS  50     	// как часто подстраиваем (мс)
#define MAX_CAL_POINTS      20		//количество точек для калибровки
#define DIGIT_AFTER_FLOAT   3		//количество знаков посля запятой для вывода

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile float target_voltage_g = 0.0f;
volatile float target_current_g = 0.0f;
uint8_t current_pot_code = 255;

volatile RegMode_t reg_mode = REG_NONE;

volatile float base_voltage_feedforward = 0.0f;

volatile uint32_t system_ms_ticks   = 0;   // инкрементируется в IRQ каждую 1мс
volatile uint8_t  flag_measure      = 0;   // выставляется в IRQ
volatile uint8_t  flag_report       = 0;   // выставляется в IRQ

// Последние измеренные значения (кэш, чтобы report не делал повторный ADC-опрос)
volatile float last_vsw = 0.0f;
volatile float last_vcu = 0.0f;

float g_voltage_k = 0.9943f;
float g_voltage_b = -0.1782f;
float g_current_k = 0.882f;
float g_current_b = 0.0116f;

eeStorage_t ee_storage = {
    .magic = EE_MAGIC_VALID,
    .voltage_k = 0.9943f,
    .voltage_b = -0.1782f,
    .current_k = 0.882f,
    .current_b = 0.0116f,

    .pid_Kp = 8.0f,
    .pid_Ki = 5.0f,
    .pid_Kd = 0.05f,

    .pid_i_Kp = 4.5f,
    .pid_i_Ki = 6.0f,
    .pid_i_Kd = 0.0f
};

typedef enum {
    CAL_MODE_NONE = 0,
    CAL_MODE_VOLTAGE,
    CAL_MODE_CURRENT
} CalMode_t;

volatile CalMode_t cal_mode = CAL_MODE_NONE;

float   cal_prog_points[MAX_CAL_POINTS];  // измеренные (некалиброванные) значения
float   cal_real_points[MAX_CAL_POINTS];  // эталонные значения от пользователя
uint8_t cal_point_count = 0;

uint8_t g_calibrated = 0;

volatile uint8_t  debug_output_enabled = 0;      // по умолчанию ВЫКЛ
volatile uint32_t debug_report_interval_ms = 1000; // по умолчанию 1 секунда

PID_Controller_t pid_v;
// Стартовые коэффициенты экспериментально
float pid_Kp = 8.0f;
float pid_Ki = 5.0f;
float pid_Kd = 0.05f;

PID_Controller_t pid_i;

float pid_i_Kp = 4.5f;
float pid_i_Ki = 6.0f;
float pid_i_Kd = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//float ReadVSW_Voltage(void);
float ReadVSW_Voltage_Avg(uint8_t samples);
float ReadVSW_Current_Avg(uint8_t samples);
void SoftSPI_WriteByte(uint8_t data);
void MCP41010_SetWiper(uint8_t value);
void FloatToStr(float value, char *buf, uint8_t decimals);
uint8_t UIntToStr(uint32_t value, char *buf);
void Regulation_Step(void);

void    Startup_ReferenceCalibration(void);      // одноразовая калибровка при старте

SetResult_t ComputeCodeForVoltage(float target_voltage, uint8_t *out_code); // чистый расчёт, без ADC/SPI
SetResult_t SetTargetVoltage(float target_voltage);  // применяется из USB-команды
SetResult_t SetTargetCurrent(float target_current);

void SendVoltageNow(void);   // формирует и шлёт "V=xx.xxxxx\r\n" немедленно
void SendCurrentNow(void);   // формирует и шлёт "I=xx.xxxxx\r\n" немедленно
void SendReportNow(void);    // формирует и шлёт "V=.. I=.." (общая телеметрия)

void    CalStart_Voltage(void);
void    CalStart_Current(void);
uint8_t CalAddPoint_Voltage(float real_value);
uint8_t CalAddPoint_Current(float real_value);
uint8_t CalStop_AndCompute(char *resp, uint8_t *resp_len); // 1=успех, 0=ошибка

void SaveCoefficientsToFlash(void);
void LoadCoefficientsFromFlash(void);

void PrintSavedVoltageCoefficients(void);
void PrintSavedCurrentCoefficients(void);

float PID_Compute(PID_Controller_t *pid, float error, float current_output);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3);   // ЗАПУСК прерывания таймера - без этого callback не будет вызываться

  EE_Init(&ee_storage, sizeof(eeStorage_t));   // ОБЯЗАТЕЛЬНО до первого EE_Read/EE_Write
  LoadCoefficientsFromFlash();                  // подхватываем сохранённую калибровку, если есть

  Startup_ReferenceCalibration();

  pid_v.Kp = ee_storage.pid_Kp;
  pid_v.Ki = ee_storage.pid_Ki;
  pid_v.Kd = ee_storage.pid_Kd;
  pid_v.dt = (float)ADJUST_INTERVAL_MS / 1000.0f; // Переводим мс в секунды (0.05s)
  pid_v.integrator = 0.0f;
  pid_v.prev_error = 0.0f;
  pid_v.out_min = 0.0f;
  pid_v.out_max = 255.0f;

  pid_i.Kp = ee_storage.pid_i_Kp;
  pid_i.Ki = ee_storage.pid_i_Ki;
  pid_i.Kd = ee_storage.pid_i_Kd;
  pid_i.dt = (float)ADJUST_INTERVAL_MS / 1000.0f;
  pid_i.integrator = 0.0f;
  pid_i.prev_error = 0.0f;
  pid_i.out_min = 0.0f;
  pid_i.out_max = 255.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- Флаг измерения/регулировки (каждые ADJUST_INTERVAL_MS) ---
	  if (flag_measure)
	  {
		  flag_measure = 0;

		  last_vsw = ReadVSW_Voltage_Avg(32);
		  last_vcu = ReadVSW_Current_Avg(32);

		  // Регулируем, только если сейчас НЕ идёт разовая калибровка
		  Regulation_Step();

	  }

	  // --- Флаг вывода в консоль (раз в 1000 мс) ---
	  if (flag_report)
	  {
		  flag_report = 0;

		  SendReportNow();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
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
  HAL_GPIO_WritePin(GPIOB, POT_SCK_Pin|POT_CS_Pin|POT_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : POT_SCK_Pin POT_CS_Pin POT_MOSI_Pin */
  GPIO_InitStruct.Pin = POT_SCK_Pin|POT_CS_Pin|POT_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Блокирующее ожидание заданного времени на основе счётчика миллисекунд от TIM3.
  * @note   Используется вместо HAL_Delay в местах, где нужен неблокирующий для
  *         прерываний busy-wait, синхронизированный с системным тиком system_ms_ticks.
  * @param  ms Время ожидания, мс.
  * @retval None
  */
static void WaitMs(uint32_t ms)
{
    uint32_t start = system_ms_ticks;
    while ((system_ms_ticks - start) < ms)
    {
        // ждём тиков от прерывания TIM3
    }
}

/**
  * @brief  Callback переполнения таймера, вызывается HAL из прерывания TIM3 каждую 1 мс.
  * @note   увеличивает системный счётчик миллисекунд system_ms_ticks на 1 и выставляет
  *         флаги flag_measure (каждые ADJUST_INTERVAL_MS) и flag_report (при включённом
  *         debug_output_enabled, с периодом debug_report_interval_ms).
  * @param  htim Указатель на структуру-хендл таймера, вызвавшего прерывание.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        system_ms_ticks++;

        if ((system_ms_ticks % ADJUST_INTERVAL_MS) == 0)
        {
            flag_measure = 1;
        }

        if (debug_output_enabled &&
                    (system_ms_ticks % debug_report_interval_ms) == 0)
                {
                    flag_report = 1;
                }
    }
}

/**
  * @brief  Измеряет выходное напряжение VSW усреднением нескольких отсчётов АЦП1 (PA4).
  * @note   Код АЦП пересчитывается в напряжение через делитель напряжения (R2/R5),
  *         затем корректируется калибровочными коэффициентами g_voltage_k/g_voltage_b.
  * @param  samples Количество отсчётов АЦП для усреднения.
  * @retval Измеренное и откалиброванное напряжение, В.
  */
float ReadVSW_Voltage_Avg(uint8_t samples)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    float raw_avg = (float)sum / samples;
    float v_adc = (raw_avg * 3.3f) / 4095.0f;
    float v_sw = v_adc * (180.0f + 15.0f) / 15.0f;

//  return v_sw;
    if (v_sw == 0.0f) return 0.0f;
    return v_sw * g_voltage_k + g_voltage_b;
//  return v_sw * 0.9943f - 0.1782f;
//  return 0.9921f * v_sw - 0.0736f - 0.08f;
//  return 0.000804f * v_sw * v_sw + 0.965f * v_sw + 0.069;
//  return v_sw * 0.994f - 0.182f;
}

/**
  * @brief  Измеряет выходной ток нагрузки усреднением нескольких отсчётов АЦП2 (PA3).
  * @note   Код АЦП пересчитывается в ток через усилитель тока INA180A4 и токовый шунт,
  *         затем корректируется калибровочными коэффициентами g_current_k/g_current_b.
  * @param  samples Количество отсчётов АЦП для усреднения.
  * @retval Измеренный и откалиброванный ток, А.
  */
float ReadVSW_Current_Avg(uint8_t samples)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 10);
        sum += HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
    }

    float raw_avg = (float)sum / samples;
    float v_adc = (raw_avg * 3.3f) / 4095.0f;
    float c_sw = v_adc / (200.0f * 0.005f);

//    return c_sw;
    if (c_sw == 0.0f) return 0.0f;
    return c_sw * g_current_k + g_current_b;
//    return c_sw * 0.882f + 0.0116f;
//    return -0.208f * c_sw *c_sw + 1.005f * c_sw - 0.00341f;
//    return c_sw * 0.8715f + 0.017;
}

/**
  * @brief  Короткая программная задержка между фронтами программного SPI.
  * @note   Используется в SoftSPI_WriteByte для формирования корректных по
  *         длительности импульсов SCK при программной реализации SPI на GPIO.
  * @retval None
  */
static inline void SoftSPI_Delay(void)
{
    __NOP(); __NOP(); __NOP(); __NOP();
}

/**
  * @brief  Передаёт один байт по программному SPI (bit-bang), режим Mode 0 (CPOL=0, CPHA=0).
  * @note   Используется для связи с цифровым потенциометром MCP41010, так как на
  *         STM32F103C6T6 отсутствует аппаратный SPI2 (доступен только SPI1 на других пинах).
  * @param  data Байт данных для передачи, бит выставляется начиная со старшего (MSB first).
  * @retval None
  */
void SoftSPI_WriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // Выставляем бит (MSB первый) пока SCK низкий
        if (data & 0x80)
            HAL_GPIO_WritePin(POT_MOSI_GPIO_Port, POT_MOSI_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(POT_MOSI_GPIO_Port, POT_MOSI_Pin, GPIO_PIN_RESET);

        SoftSPI_Delay();

        // Передний фронт SCK — приёмник (MCP41010) защёлкивает бит
        HAL_GPIO_WritePin(POT_SCK_GPIO_Port, POT_SCK_Pin, GPIO_PIN_SET);
        SoftSPI_Delay();

        // Спад SCK — готовим следующий бит
        HAL_GPIO_WritePin(POT_SCK_GPIO_Port, POT_SCK_Pin, GPIO_PIN_RESET);

        data <<= 1;
    }
}

/**
  * @brief  Устанавливает положение (код 0..255) цифрового потенциометра MCP41010.
  * @note   Формирует SPI-транзакцию из командного байта записи в Pot0 (0x11) и байта
  *         данных, обрамлённую низким уровнем сигнала POT_CS на время передачи.
  * @param  value Код положения потенциометра, 0..255.
  * @retval None
  */
void MCP41010_SetWiper(uint8_t value)
{
    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_RESET); // CS -> low

    SoftSPI_WriteByte(0x11);   // Command: write Pot0
    SoftSPI_WriteByte(value);  // Data: 0x00..0xFF

    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_SET);   // CS -> high
}

/**
  * @brief  Преобразует беззнаковое целое число в десятичную строку без использования stdio.h.
  * @param  value Число для преобразования.
  * @param  buf   Буфер для результата (без завершающего нулевого символа).
  * @retval Количество записанных в buf символов.
  */
uint8_t UIntToStr(uint32_t value, char *buf)
{
    char tmp[10];
    uint8_t len = 0;

    if (value == 0)
    {
        buf[0] = '0';
        return 1;
    }

    while (value > 0)
    {
        tmp[len++] = '0' + (value % 10);
        value /= 10;
    }

    // Разворачиваем (т.к. цифры получены в обратном порядке)
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = tmp[len - 1 - i];
    }

    return len;
}


/**
  * @brief  Форматирует число с плавающей точкой в строку с фиксированным числом знаков
  *         после запятой, без использования printf/sprintf из stdio.h.
  * @note   Поддерживает отрицательные значения и корректно обрабатывает перенос при
  *         округлении дробной части (например, 0.9999 при decimals=3 даёт "1.000").
  * @param  value    Значение для форматирования.
  * @param  buf      Буфер для результирующей, завершённой нулём строки.
  * @param  decimals Количество знаков после запятой.
  * @retval None
  */
void FloatToStr(float value, char *buf, uint8_t decimals)
{
    if (value < 0)
    {
        *buf++ = '-';
        value = -value;
    }

    int32_t int_part = (int32_t)value;
    float frac = value - (float)int_part;

    uint32_t scale = 1;
    for (uint8_t i = 0; i < decimals; i++) scale *= 10;

    int32_t frac_part = (int32_t)(frac * scale + 0.5f);

    // Перенос при округлении (например 0.9999 -> 1.000)
    if (frac_part >= (int32_t)scale)
    {
        frac_part -= (int32_t)scale;
        int_part += 1;
    }

    // Целая часть
    uint8_t pos = UIntToStr((uint32_t)int_part, buf);
    buf[pos++] = '.';

    // Дробная часть с ведущими нулями
    char frac_str[10];
    uint8_t frac_len = UIntToStr((uint32_t)frac_part, frac_str);

    for (uint8_t i = frac_len; i < decimals; i++)
    {
        buf[pos++] = '0';
    }

    memcpy(&buf[pos], frac_str, frac_len);
    buf[pos + frac_len] = '\0';
}

/**
  * @brief  Исполнительный шаг алгоритма стабилизации (вызывается циклически)
  * @note  В режиме REG_NONE регулирование отключено
  * В режиме REG_VOLTAGE рассчитывает ошибку рассогласования относительно заданного target_voltage
  * и вычисляет управляющее воздействие с помощью ПИД-структуры напряжения.
  * В режиме REG_CURRENT рассчитывает ошибку рассогласования относительно заданного target_current
  * и вычисляет управляющее воздействие с помощью ПИД-структуры тока.
  * @retval None
  */
void Regulation_Step(void)
{
	if (reg_mode == REG_NONE)
	{
		return;
	}

	if (reg_mode == REG_VOLTAGE)
	{
		float error = target_voltage_g - last_vsw;

		// Для напряжения ПИД вычисляет смещение кода потенциометра напрямую
		float pid_output = PID_Compute(&pid_v, error, (float)current_pot_code);
		float next_code = (float)current_pot_code - pid_output;

		if (next_code < pid_v.out_min) next_code = pid_v.out_min;
		if (next_code > pid_v.out_max) next_code = pid_v.out_max;

		current_pot_code = (uint8_t)next_code;
		MCP41010_SetWiper(current_pot_code);
	}
	else if (reg_mode == REG_CURRENT) // РЕГУЛИРОВАНИЕ ТОКА (как в working.c)
	{
		if (last_vcu < 0.002f)
		{
			pid_i.integrator = 0.0f;
			return;
		}
		float error = target_current_g - last_vcu;

		// Пропорциональная часть (Kp * error)
//		float p_term = pid_i.Kp * active_error;
		float p_term = pid_i.Kp * error;
		// Временный расчет следующего значения интеграла (шаг интегрирования dt = 0.05s)
//		float next_integrator = pid_i.integrator + active_error * pid_i.dt;
//		float next_integrator = pid_i.integrator + error * pid_i.dt;
		float next_integrator = pid_i.integrator + error * pid_i.dt;
		float i_term = pid_i.Ki * next_integrator;

		// Дифференциальная часть (при необходимости, если Kd != 0)
//		float d_term = pid_i.Kd * (active_error - pid_i.prev_error) / pid_i.dt;
//		float d_term = pid_i.Kd * (error - pid_i.prev_error) / pid_i.dt;
//		pid_i.prev_error = active_error;
//		pid_i.prev_error = error;
		// Расчет коррекции напряжения
//		float voltage_correction = p_term + i_term + d_term;
		float voltage_correction = p_term + i_term;

		// Результирующее требуемое напряжение
		float v_result = base_voltage_feedforward + voltage_correction;



		// Жесткие рамки напряжения как в рабочем working.c (12.25V ... 24.0V)
		// Защита Anti-windup: сохраняем интеграл только если напряжение не уперлось в рамки
		if (v_result >= 12.25f && v_result <= 24.0f)
		{
			pid_i.integrator = next_integrator;
		}
		else
		{
		if (v_result < 12.25f) v_result = 12.25f;
		if (v_result > 24.0f) v_result = 24.0f;
		}
		float load = 1.25f * 180000.0f / (v_result - 1.25f) - 10000.0f;
		int16_t pot;

		if (load < 0.0f){ pot = 0;}

		else if (load > 10000.0f) {pot = 255;}
		else
		{
			pot = (int16_t)((load) * 256.0f / 10000.0f) - 2;
			if (pot < 0.0f) pot = 0.0f;
			if (pot > 255.0f) pot = 255.0f;
		}

		current_pot_code = (uint8_t)pot;
		MCP41010_SetWiper(current_pot_code);
//		uint8_t next_code = current_pot_code;
//		if (ComputeCodeForVoltage(v_result, &next_code) == SET_OK)
//		{
//			current_pot_code = next_code;
//			MCP41010_SetWiper(current_pot_code);
//		}
	}

}

/**
  * @brief  Установка потенциометра в код 255 при старте МК.
  * @note   None
  * @retval None
  */
void Startup_ReferenceCalibration(void)
{
    // 1. Ставим код 255
    MCP41010_SetWiper(255);
    current_pot_code = 255;
    WaitMs(150); // даём преобразователю установиться

    g_calibrated = 1;

    // Кэшируем текущие измерения
    last_vsw = ReadVSW_Voltage_Avg(32);
    last_vcu = ReadVSW_Current_Avg(32);
}

/**
  * @brief  Рассчитывает код потенциометра, необходимый для получения целевого VSW.
  * @note   Расчёт ведётся аналитически
  * @param  target_voltage Целевое выходное напряжение, В.
  * @param  out_code       Указатель, куда записывается рассчитанный код (0..255).
  * @retval SET_OK при успешном расчёте; SET_ERR_TOO_LOW/SET_ERR_TOO_HIGH, если целевое
  *         напряжение вне физически достижимого диапазона.
  */
SetResult_t ComputeCodeForVoltage(float target_voltage, uint8_t *out_code)
{
    if (!g_calibrated)
    {
    //    return current_pot_code; // калибровка ещё не выполнена - не трогаем
    	*out_code = current_pot_code;
    	return SET_OK;
    }

    if (target_voltage <= 1.25f)
	{
		return SET_ERR_TOO_LOW;
	}

    float Rb_target = 180000.0f * 1.25f / (target_voltage - 1.25f) - 10000.0f;

    if (Rb_target < 0.0f)
	{
		return SET_ERR_TOO_HIGH; // Вольтаж слишком высокий (сопротивление ушло ниже нуля)
	}
	if (Rb_target > 10000.0f)
	{
		return SET_ERR_TOO_LOW; // Вольтаж слишком низкий (сопротивление превысило предел)
	}

    int16_t code_f = (int16_t)((Rb_target) * 256.0f / 10000.0f - 2);
 // code_f = (code_f - 10);
//
    if (code_f < 0.0f)   code_f = 0.0f;
    if (code_f > 255.0f) code_f = 255.0f;

    code_f = (uint8_t)code_f;

    *out_code = code_f;
    return SET_OK;
}

/**
  * @brief  Устанавливает целевое выходное напряжение и включает автоматическое регулирование.
  * @note   Проверяет аварийное состояние платы (последнее измеренное VSW ниже 1.25В),
  *         рассчитывает и применяет код потенциометра через ComputeCodeForVoltage;
  *         точная доводка до целевого значения выполняется фоновым циклом
  *         VoltageRegulation_Step. При ошибке состояние платы не изменяется.
  * @param  target_voltage Целевое выходное напряжение, В.
  * @retval SET_OK при успешном применении; иначе код ошибки SetResult_t
  */
SetResult_t SetTargetVoltage(float target_voltage)
{
	// Аварийная проверка: если на плате вообще пропало напряжение
	if (last_vsw < 1.25f)
	{
		return SET_ERR_NO_VOLTAGE;
	}

	uint8_t local_code = 0;
	SetResult_t calc_result = ComputeCodeForVoltage(target_voltage, &local_code);
	// Проверка границ ДО любых изменений на плате
	if (calc_result != SET_OK)
	{
		return calc_result;
	}
    target_voltage_g = target_voltage;

//    uint8_t code = ComputeCodeForVoltage(target_voltage);
    current_pot_code = local_code;
    MCP41010_SetWiper(current_pot_code);      // мгновенный прыжок близко к цели, БЕЗ сброса в минимум


    pid_v.integrator = 0.0f;
	pid_v.prev_error = 0.0f;

	reg_mode = REG_VOLTAGE;        // дальше Regulation_Step доводит точность

    return SET_OK;
}

/**
  * @brief  Устанавливает целевой выходной ток через пересчёт в эквивалентное напряжение.
  * @note   Сопротивление нагрузки вычисляется по последним измеренным значениям
  *         (R = last_vsw / last_vcu), расчитывает необходимое стартовое напряжение
  *         и ставит соответствующее значение потенциометра
  *         После этого управление передается контуру ПИД-регулирования тока.
  * @param  target_current Целевой выходной ток, А (> 0).
  * @retval SET_OK при успехе; SET_ERR_NO_LOAD, если последний измеренный ток равен 0
  *         (сопротивление нагрузки неизвестно); иные коды ошибок в ComputeCodeForVoltage.
  */
SetResult_t SetTargetCurrent(float target_current)
{
	if (target_current <= 0.0f)
	{
		return SET_ERR_TOO_LOW;   // отрицательный/нулевой ток - тоже "нет тока"
	}
    float I_out1 = last_vcu;   // последний измеренный ток
    float V_out1 = last_vsw;   // последнее измеренное напряжение

    // Защита от деления на ноль / отсутствия нагрузки
    if (I_out1 <= 0.0001f)
    {
        return SET_ERR_NO_LOAD; // I_out1 = 0 -> ничего не делаем, как и требовалось
    }

    float R_load = V_out1 / I_out1;

    float V_guess = target_current * R_load;
    if (V_guess < 12.5f)
    {
        V_guess = 12.5f;
    }
    else if (V_guess > 23.2f) V_guess = 23.2;

    uint8_t local_code = 0;
    SetResult_t calc_result = ComputeCodeForVoltage(V_guess, &local_code);
    if (calc_result != SET_OK)
    {
	   // Даже приблизительно недостижимо - отказываем, ничего не меняем
	   return calc_result;
    }

    target_current_g = target_current;
    base_voltage_feedforward = V_guess;

    current_pot_code = local_code;
    MCP41010_SetWiper(current_pot_code);   // быстрый начальный прыжок близко к цели

   // Сброс интегратора/дифференциатора контура тока при каждой новой цели
    pid_i.integrator = 0.0f;
    pid_i.prev_error = 0.0f;

    reg_mode = REG_CURRENT;   // дальше Regulation_Step (контур ТОКА) следит за last_vcu
							  // и сам подстраивает код при любом изменении нагрузки
    return SET_OK;
}

/**
  * @brief  Немедленно формирует и отправляет по USB CDC строку с текущим напряжением.
  * @note   Формат сообщения: "V=<знач>V\r\n". Источник данных — кэш last_vsw.
  * @retval None
  */
void SendVoltageNow(void)
{
    char v_str[16];
    FloatToStr(last_vsw, v_str, DIGIT_AFTER_FLOAT);

    char msg[32];
    uint8_t pos = 0;
    msg[pos++] = 'V'; msg[pos++] = '=';
    memcpy(&msg[pos], v_str, strlen(v_str)); pos += strlen(v_str);
    msg[pos++] = 'V';
    msg[pos++] = '\r'; msg[pos++] = '\n';

    CDC_Transmit_FS_Blocking((uint8_t*)msg, pos, 50);
}

/**
  * @brief  Немедленно формирует и отправляет по USB CDC строку с текущим током.
  * @note   Формат сообщения: "I=<знач>A\r\n". Источник данных — кэш last_vcu.
  * @retval None
  */
void SendCurrentNow(void)
{
    char i_str[16];
    FloatToStr(last_vcu, i_str, DIGIT_AFTER_FLOAT);

    char msg[32];
    uint8_t pos = 0;
    msg[pos++] = 'I'; msg[pos++] = '=';
    memcpy(&msg[pos], i_str, strlen(i_str)); pos += strlen(i_str);
    msg[pos++] = 'A';
    msg[pos++] = '\r'; msg[pos++] = '\n';

    CDC_Transmit_FS_Blocking((uint8_t*)msg, pos, 50);
}

/**
  * @brief  Формирует и отправляет по USB CDC общий отчёт по напряжению и току.
  * @note   Формат сообщения: "V=<знач>V I=<знач>A\r\n". Вызывается по флагу flag_report,
  *         выставляемому в HAL_TIM_PeriodElapsedCallback при включённом периодическом
  *         отладочном выводе (debug_output_enabled).
  * @retval None
  */
void SendReportNow(void)
{
    char v_str[16], i_str[16];
    FloatToStr(last_vsw, v_str, DIGIT_AFTER_FLOAT);
    FloatToStr(last_vcu, i_str, DIGIT_AFTER_FLOAT);

    char msg[64];
    uint8_t pos = 0;
    msg[pos++] = 'V'; msg[pos++] = '=';
    memcpy(&msg[pos], v_str, strlen(v_str)); pos += strlen(v_str);
    msg[pos++] = 'V'; msg[pos++] = ' ';
    msg[pos++] = 'I'; msg[pos++] = '=';
    memcpy(&msg[pos], i_str, strlen(i_str)); pos += strlen(i_str);
    msg[pos++] = 'A';
    msg[pos++] = '\r'; msg[pos++] = '\n';

    CDC_Transmit_FS_Blocking((uint8_t*)msg, pos, 50);
}

/**
  * @brief  Решает задачу линейной аппроксимации real = k*prog + b методом наименьших
  *         квадратов (аналитическое решение системы нормальных уравнений методом Крамера).
  * @param  prog  Массив "программных" (некалиброванных) измеренных значений.
  * @param  real  Массив соответствующих реальных значений.
  * @param  n     Количество точек в массивах prog и real.
  * @param  out_k Указатель, куда записывается рассчитанный коэффициент наклона k.
  * @param  out_b Указатель, куда записывается рассчитанный коэффициент смещения b.
  * @retval 1 при успешном расчёте; 0, если точек меньше двух либо система вырождена
  *         (все точки имеют одинаковое prog-значение).
  */
static uint8_t SolveLinearFit(float *prog, float *real, uint8_t n, float *out_k, float *out_b)
{
    if (n < 2) return 0;

    float Sx = 0.0f, Sy = 0.0f, Sxx = 0.0f, Sxy = 0.0f;

    for (uint8_t i = 0; i < n; i++)
    {
        Sx  += prog[i];
        Sy  += real[i];
        Sxx += prog[i] * prog[i];
        Sxy += prog[i] * real[i];
    }

    float det = Sxx * (float)n - Sx * Sx;

    // Вырожденный случай (все точки с одинаковым prog) - защищаемся
    if (det < 0.000001f && det > -0.000001f)
    {
        return 0;
    }

    *out_k = (Sxy * (float)n - Sx * Sy) / det;
    *out_b = (Sxx * Sy - Sx * Sxy) / det;

    return 1;
}

/**
  * @brief  Переводит канал измерения напряжения в режим сбора калибровочных точек.
  * @note   Временно обнуляет g_voltage_k/g_voltage_b (k=1, b=0) для получения "сырых"
  *         показаний на время калибровки и очищает буфер точек cal_point_count.
  * @retval None
  */
void CalStart_Voltage(void)
{
    g_voltage_k = 1.0f;   // обнуляем - на время калибровки считываем "сырые" значения
    g_voltage_b = 0.0f;
    cal_mode = CAL_MODE_VOLTAGE;
    cal_point_count = 0;
}

/**
  * @brief  Переводит канал измерения тока в режим сбора калибровочных точек.
  * @note   Временно обнуляет g_current_k/g_current_b (k=1, b=0) для получения "сырых"
  *         показаний на время калибровки и очищает буфер точек cal_point_count.
  * @retval None
  */
void CalStart_Current(void)
{
    g_current_k = 1.0f;
    g_current_b = 0.0f;
    cal_mode = CAL_MODE_CURRENT;
    cal_point_count = 0;
}

/**
  * @brief  Добавляет одну калибровочную точку для канала напряжения.
  * @note   В качестве "программного" значения используется текущий кэш last_vsw,
  *         снятый при коэффициентах k=1, b=0, установленных в CalStart_Voltage.
  * @param  real_value Эталонное значение напряжения (например, от внешнего мультиметра), В.
  * @retval 1, если точка успешно добавлена; 0, если режим калибровки напряжения не
  *         активен либо буфер точек уже заполнен >= MAX_CAL_POINTS
  */
uint8_t CalAddPoint_Voltage(float real_value)
{
    if (cal_mode != CAL_MODE_VOLTAGE) return 0;
    if (cal_point_count >= MAX_CAL_POINTS) return 0;

    cal_prog_points[cal_point_count] = last_vsw;  // текущее (некалиброванное, k=1,b=0) измерение
    cal_real_points[cal_point_count] = real_value;
    cal_point_count++;

    return 1;
}

/**
  * @brief  Добавляет одну калибровочную точку для канала тока.
  * @note   В качестве "программного" значения используется текущий кэш last_vcu,
  *         снятый при коэффициентах k=1, b=0, установленных в CalStart_Current.
  * @param  real_value Эталонное значение тока (например, от внешнего мультиметра), А.
  * @retval 1, если точка успешно добавлена; 0, если режим калибровки тока не
  *         активен либо буфер точек уже заполнен >= MAX_CAL_POINTS
  */
uint8_t CalAddPoint_Current(float real_value)
{
    if (cal_mode != CAL_MODE_CURRENT) return 0;
    if (cal_point_count >= MAX_CAL_POINTS) return 0;

    cal_prog_points[cal_point_count] = last_vcu;
    cal_real_points[cal_point_count] = real_value;
    cal_point_count++;

    return 1;
}

/**
  * @brief  Завершает текущий режим калибровки: рассчитывает и применяет новые
  *         коэффициенты, сохраняет их во Flash-память и формирует текстовый ответ.
  * @note   Расчёт выполняется функцией SolveLinearFit по накопленным точкам.
  *         При успехе автоматически вызывает SaveCoefficientsToFlash().
  * @param  resp     Буфер для строки ответа (минимум 96 байт).
  * @param  resp_len Указатель, куда записывается фактическая длина сформированного ответа.
  * @retval 1 при успешном расчёте; 0, если данных недостаточно (см. SolveLinearFit).
  */
uint8_t CalStop_AndCompute(char *resp, uint8_t *resp_len)
{
    float k = 0.0f, b = 0.0f;
    uint8_t ok = SolveLinearFit(cal_prog_points, cal_real_points, cal_point_count, &k, &b);

    uint8_t pos = 0;

    if (!ok)
    {
        const char msg[] = "ERR CAL FAILED (need >=2 distinct points)\r\n";
        memcpy(resp, msg, strlen(msg));
        pos = strlen(msg);

        cal_mode = CAL_MODE_NONE;
        cal_point_count = 0;

        *resp_len = pos;
        return 0;
    }

    if (cal_mode == CAL_MODE_VOLTAGE)
    {
        g_voltage_k = k;
        g_voltage_b = b;

        SaveCoefficientsToFlash();

        const char p1[] = "NEW COEFFS: g_voltage_k = ";
        memcpy(&resp[pos], p1, strlen(p1)); pos += strlen(p1);

        char kstr[16];
        FloatToStr(k, kstr, 4);
        memcpy(&resp[pos], kstr, strlen(kstr)); pos += strlen(kstr);

        const char p2[] = "f; g_voltage_b = ";
        memcpy(&resp[pos], p2, strlen(p2)); pos += strlen(p2);

        char bstr[16];
        FloatToStr(b, bstr, 4);
        memcpy(&resp[pos], bstr, strlen(bstr)); pos += strlen(bstr);

        const char p3[] = "f;\r\n";
        memcpy(&resp[pos], p3, strlen(p3)); pos += strlen(p3);
    }
    else if (cal_mode == CAL_MODE_CURRENT)
    {
        g_current_k = k;
        g_current_b = b;

        SaveCoefficientsToFlash();

        const char p1[] = "NEW COEFFS: g_current_k = ";
        memcpy(&resp[pos], p1, strlen(p1)); pos += strlen(p1);

        char kstr[16];
        FloatToStr(k, kstr, 4);
        memcpy(&resp[pos], kstr, strlen(kstr)); pos += strlen(kstr);

        const char p2[] = "f; g_current_b = ";
        memcpy(&resp[pos], p2, strlen(p2)); pos += strlen(p2);

        char bstr[16];
        FloatToStr(b, bstr, 4);
        memcpy(&resp[pos], bstr, strlen(bstr)); pos += strlen(bstr);

        const char p3[] = "f;\r\n";
        memcpy(&resp[pos], p3, strlen(p3)); pos += strlen(p3);
    }

    cal_mode = CAL_MODE_NONE;
    cal_point_count = 0;

    *resp_len = pos;
    return 1;
}

/**
  * @brief  Сохраняет текущие калибровочные коэффициенты во Flash-память (EEPROM emulation).
  * @note   Записывает структуру ee_storage целиком (с установленным маркером
  *         EE_MAGIC_VALID) через библиотеку NimaLTD I-CUBE-EE; сопровождается
  *         стиранием страницы Flash, зарезервированной под хранение (см. EE_Write).
  * @retval None
  */
void SaveCoefficientsToFlash(void)
{
    ee_storage.magic      = EE_MAGIC_VALID;
    ee_storage.voltage_k  = g_voltage_k;
    ee_storage.voltage_b  = g_voltage_b;
    ee_storage.current_k  = g_current_k;
    ee_storage.current_b  = g_current_b;

    ee_storage.pid_Kp     = pid_v.Kp;
	ee_storage.pid_Ki     = pid_v.Ki;
	ee_storage.pid_Kd     = pid_v.Kd;

	ee_storage.pid_i_Kp   = pid_i.Kp;
	ee_storage.pid_i_Ki   = pid_i.Ki;
	ee_storage.pid_i_Kd   = pid_i.Kd;

    EE_Write();   // стирает страницу и записывает всю структуру целиком
}

/**
  * @brief  Загружает ранее сохранённые калибровочные коэффициенты из Flash-памяти.
  * @note None
  * @retval None
  */
void LoadCoefficientsFromFlash(void)
{
    EE_Read();

    if (ee_storage.magic == EE_MAGIC_VALID)
    {
        // Во flash есть валидные ранее сохранённые коэффициенты - используем их
        g_voltage_k = ee_storage.voltage_k;
        g_voltage_b = ee_storage.voltage_b;
        g_current_k = ee_storage.current_k;
        g_current_b = ee_storage.current_b;

        pid_v.Kp = ee_storage.pid_Kp;
		pid_v.Ki = ee_storage.pid_Ki;
		pid_v.Kd = ee_storage.pid_Kd;

		pid_i.Kp = ee_storage.pid_i_Kp;
		pid_i.Ki = ee_storage.pid_i_Ki;
		pid_i.Kd = ee_storage.pid_i_Kd;
    }
    // Если magic не совпал (чистый/новый чип, либо страница ещё не записана) -
    // просто оставляем значения по умолчанию, заданные при объявлении g_voltage_k и т.д.
}

/**
  * @brief  Считывает и отправляет по USB CDC содержимое сохранённых во Flash коэффициентов.
  * @note   Используется командой get_cv.
  * @retval None
  */
void PrintSavedVoltageCoefficients(void)
{
    // 1. Принудительно вычитываем актуальное состояние Flash в структуру ee_storage
    EE_Read();

    char msg[128];
    uint8_t pos = 0;

    // Заголовок сообщения
    const char header[] = "--- STORED COEFFS ---\r\n";
    memcpy(&msg[pos], header, strlen(header)); pos += strlen(header);

    // Буферы под строки для float значений
    char vk_str[16], vb_str[16];

    // Преобразуем коэффициенты в строки с 4 знаками после запятой
    FloatToStr(ee_storage.voltage_k, vk_str, 4);
    FloatToStr(ee_storage.voltage_b, vb_str, 4);

    // Сборка итогового текста "склеиванием" строк через memcpy
    // Напряжение:
    const char v_text[] = "Voltage: k=";
    memcpy(&msg[pos], v_text, strlen(v_text)); pos += strlen(v_text);
    memcpy(&msg[pos], vk_str, strlen(vk_str)); pos += strlen(vk_str);
    msg[pos++] = ' '; msg[pos++] = 'b'; msg[pos++] = '=';
    memcpy(&msg[pos], vb_str, strlen(vb_str)); pos += strlen(vb_str);
    msg[pos++] = '\r'; msg[pos++] = '\n';

    // Конечный символ переноса
    msg[pos++] = '\r'; msg[pos++] = '\n';

    // Отправляем собранную строку в порт USB
    CDC_Transmit_FS_Blocking((uint8_t*)msg, pos, 50);
}

/**
  * @brief  Считывает и отправляет по USB CDC содержимое сохранённых во Flash коэффициентов.
  * @note   Используется командой get_ci.
  * @retval None
  */
void PrintSavedCurrentCoefficients(void)
{
    // 1. Принудительно вычитываем актуальное состояние Flash в структуру ee_storage
    EE_Read();

    char msg[128];
    uint8_t pos = 0;

    // Заголовок сообщения
    const char header[] = "--- STORED COEFFS ---\r\n";
    memcpy(&msg[pos], header, strlen(header)); pos += strlen(header);

    // Буферы под строки для float значений
    char ik_str[16], ib_str[16];

    // Преобразуем коэффициенты в строки с 4 знаками после запятой
    FloatToStr(ee_storage.current_k, ik_str, 4);
    FloatToStr(ee_storage.current_b, ib_str, 4);


    // Ток:
    const char i_text[] = "Current: k=";
    memcpy(&msg[pos], i_text, strlen(i_text)); pos += strlen(i_text);
    memcpy(&msg[pos], ik_str, strlen(ik_str)); pos += strlen(ik_str);
    msg[pos++] = ' '; msg[pos++] = 'b'; msg[pos++] = '=';
    memcpy(&msg[pos], ib_str, strlen(ib_str)); pos += strlen(ib_str);
    msg[pos++] = '\r'; msg[pos++] = '\n';

    // Конечный символ переноса
    msg[pos++] = '\r'; msg[pos++] = '\n';

    // Отправляем собранную строку в порт USB
    CDC_Transmit_FS_Blocking((uint8_t*)msg, pos, 50);
}


/**
  * @brief  Вычисляет управляющее воздействие по алгоритму ПИД-регулирования.
  * @note   Реализует расчет пропорциональной, интегральной и дифференциальной составляющих.
  *         Включает в себя двухступенчатую защиту от интегрального насыщения (Anti-windup):
  *         1. Условное прекращение интегрирования (Clamping), если исполнительный орган
  *            (`current_output`) уже достиг своих физических технологических границ (`out_max`/`out_min`).
  *         2. Жесткое ограничение (насыщение) самого накопителя интегратора диапазоном [-50.0, 50.0].
  * @param  pid            Указатель на структуру с коэффициентами и внутренним состоянием ПИД-регулятора.
  * @param  error          Текущая ошибка регулирования (рассогласование между целью и фактом).
  * @param  current_output Текущий установленный код/значение на исполнительном органе (например, код потенциометра).
  * @retval float          Рассчитанная величина коррекции (управляющее воздействие).
  */
float PID_Compute(PID_Controller_t *pid, float error, float current_output)
{
    // 1. Пропорциональная составляющая
    float p_term = pid->Kp * error;


    // 2. Интегральная составляющая с защитой от насыщения (Anti-windup)
        // Накапливаем интеграл только если потенциометр еще НЕ уперся в физический предел
        if (!((current_output >= pid->out_max && error < 0.0f) || (current_output <= pid->out_min && error > 0.0f)))
        {
            pid->integrator += error * pid->dt;
        }

        // Мягко ограничиваем вклад интегратора (clamping)
        float int_limit = 50.0f;
        if (pid->integrator > int_limit)  pid->integrator = int_limit;
        if (pid->integrator < -int_limit) pid->integrator = -int_limit;

        float i_term = pid->Ki * pid->integrator;

        // 3. Дифференциальная составляющая
        float d_term = pid->Kd * (error - pid->prev_error) / pid->dt;
        pid->prev_error = error;

        return p_term + i_term + d_term;
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








