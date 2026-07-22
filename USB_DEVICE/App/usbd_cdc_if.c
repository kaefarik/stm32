/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum {
    SET_OK = 0,
    SET_ERR_TOO_LOW,     // ниже физического минимума платы
    SET_ERR_TOO_HIGH,    // выше физического максимума платы
	SET_ERR_NO_VOLTAGE,  // для напряжения v < 1.0f
    SET_ERR_NO_LOAD       // для тока: I_out1 = 0, сопротивление нагрузки неизвестно
} SetResult_t;

typedef enum {
    REG_NONE = 0,
    REG_VOLTAGE,
    REG_CURRENT
} RegMode_t;

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

extern uint8_t UIntToStr(uint32_t value, char *buf);
extern void MCP41010_SetWiper(uint8_t value);
static uint8_t cmd_buffer[64];
static uint16_t cmd_index = 0;
extern volatile RegMode_t reg_mode;
extern SetResult_t SetTargetVoltage(float target_voltage);
extern SetResult_t SetTargetCurrent(float target_current);
extern volatile uint8_t  debug_output_enabled;
extern volatile uint32_t debug_report_interval_ms;

extern void CalStart_Voltage(void);
extern void CalStart_Current(void);
extern uint8_t CalAddPoint_Voltage(float real_value);
extern uint8_t CalAddPoint_Current(float real_value);
extern uint8_t CalStop_AndCompute(char *resp, uint8_t *resp_len);
extern void SendVoltageNow(void);
extern void SendCurrentNow(void);
extern void PrintSavedVoltageCoefficients(void);
extern void PrintSavedCurrentCoefficients(void);

extern PID_Controller_t pid_v;
extern PID_Controller_t pid_i;
extern void SaveCoefficientsToFlash(void);
extern void FloatToStr(float value, char *buf, uint8_t decimals);
/**
  * @brief  Отправляет данные по USB CDC с ожиданием готовности передатчика (retry-цикл).
  * @note   Оборачивает CDC_Transmit_FS, повторяя попытку до истечения timeout_ms,
  *         если хост временно не готов принять пакет (передатчик занят, USBD_BUSY).
  * @param  Buf        Указатель на буфер данных для отправки.
  * @param  Len        Длина данных, байт.
  * @param  timeout_ms Максимальное время ожидания готовности передатчика, мс.
  * @retval USBD_OK при успешной отправке; USBD_FAIL при истечении времени ожидания.
  */
uint8_t CDC_Transmit_FS_Blocking(uint8_t* Buf, uint16_t Len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t result;

    do
    {
        result = CDC_Transmit_FS(Buf, Len);
        if (result == USBD_OK)
        {
            return USBD_OK;
        }
        __asm("NOP");

    } while ((HAL_GetTick() - start) < timeout_ms);

    return USBD_FAIL; // Если USB завис, мы просто выйдем по таймауту, а не намертво
}


// Простая замена atoi() — парсит только неотрицательные целые числа
/**
  * @brief  Упрощённая замена atoi(): преобразует строку в неотрицательное целое число.
  * @note   Не поддерживает знак и останавливается на первом нецифровом символе.
  *         Используется вместо stdlib.h для экономии Flash-памяти.
  * @param  str Указатель на входную строку.
  * @retval Разобранное целое число.
  */
static int MyAtoi(const char *str)
{
    int result = 0;
    while (*str >= '0' && *str <= '9')
    {
        result = result * 10 + (*str - '0');
        str++;
    }
    return result;
}


/**
  * @brief  Упрощённая замена atof(): преобразует строку в число с плавающей точкой.
  * @note   Поддерживает необязательный знак «-» и дробную часть после точки.
  *         Используется вместо stdlib.h для экономии Flash-памяти.
  * @param  str Указатель на входную строку.
  * @retval Разобранное значение типа float.
  */
static float MyAtof(const char *str)
{
    float sign = 1.0f;
    if (*str == '-') { sign = -1.0f; str++; }

    float int_part = 0.0f;
    while (*str >= '0' && *str <= '9')
    {
        int_part = int_part * 10 + (*str - '0');
        str++;
    }

    float frac_part = 0.0f;
    if (*str == '.')
    {
        str++;
        float scale = 0.1f;
        while (*str >= '0' && *str <= '9')
        {
            frac_part += (*str - '0') * scale;
            scale *= 0.1f;
            str++;
        }
    }

    return sign * (int_part + frac_part);
}


/**
  * @brief  Формирует и отправляет по USB CDC текстовый ответ на основе кода результата
  *         установки напряжения/тока.
  * @note   Используется командами V и I для единообразного вывода как успешного
  *         результата (с меткой ok_tag), так и всех вариантов ошибок SetResult_t.
  * @param  result  Код результата выполнения операции установки.
  * @param  ok_tag  Текстовая метка, добавляемая к ответу "OK " при успехе
  *                 (например, "SETTING_V" или "SETTING_I").
  * @retval None
  */
static void SendSetResultResponse(SetResult_t result, const char *ok_tag)
{
    char resp[48];
    uint8_t pos = 0;

    switch (result)
    {
        case SET_OK:
        {
            const char p1[] = "OK ";
            memcpy(resp, p1, strlen(p1)); pos = strlen(p1);
            memcpy(&resp[pos], ok_tag, strlen(ok_tag)); pos += strlen(ok_tag);
            resp[pos++] = '\r'; resp[pos++] = '\n';
            break;
        }
        case SET_ERR_TOO_LOW:
        {
            const char msg[] = "ERR OUT OF RANGE (BELOW MIN)\r\n";
            memcpy(resp, msg, strlen(msg)); pos = strlen(msg);
            break;
        }
        case SET_ERR_TOO_HIGH:
        {
            const char msg[] = "ERR OUT OF RANGE (ABOVE MAX)\r\n";
            memcpy(resp, msg, strlen(msg)); pos = strlen(msg);
            break;
        }
        case SET_ERR_NO_LOAD:
        {
            const char msg[] = "ERR NO CURRENT (NO LOAD)\r\n";
            memcpy(resp, msg, strlen(msg)); pos = strlen(msg);
            break;
        }
        case SET_ERR_NO_VOLTAGE:
		{
			const char msg[] = "ERR NO VOLTAGE\r\n";
			memcpy(resp, msg, strlen(msg)); pos = strlen(msg);
			break;
		}
    }

    CDC_Transmit_FS((uint8_t*)resp, pos);
}

/**
 * @brief  Пропускает все пробельные символы в строке.
 * @param  str Исходная строка.
 * @return Указатель на первый непробельный символ.
 */
static char* SkipSpaces(char* str)
{
    while (*str == ' ' || *str == '\t')
    {
        str++;
    }
    return str;
}

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *  @note
  *         функция накапливает принятые байты в cmd_buffer до символа '\r' или '\n', после чего разбирает
  *         накопленную строку как одну из команд текстового протокола (SET, STOP,
  *         GETV, GETI, GETC, CALV, CALI, CALS, VREAL, IREAL, ON, OFF, F, V, I),
  *         вызывает соответствующий обработчик и отправляет текстовый ответ
  *         ("OK ..." или "ERR ...") через CDC_Transmit_FS.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    for (uint32_t i = 0; i < *Len; i++)
    {
        uint8_t c = Buf[i];

        if (c == '\n' || c == '\r')
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';

                if (strncmp((char*)cmd_buffer, "set_r", 5) == 0)
                {
                	char *arg_ptr = (char*)&cmd_buffer[5];
                	arg_ptr = SkipSpaces(arg_ptr);

                    int value = MyAtof(arg_ptr);
                    if (value < 0 || value > 255)
                    {
                    	char resp[] = "OUT OF RANGE [0;255]\r\n";
                    	CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                    }
                    else
                    {
                    	reg_mode = REG_NONE;
                    	MCP41010_SetWiper((uint8_t)value);

						char resp[32];
						uint8_t pos = 0;
						const char prefix[] = "OK SET_R=";
						memcpy(resp, prefix, strlen(prefix));
						pos = strlen(prefix);
						pos += UIntToStr((uint32_t)value, &resp[pos]);
						resp[pos++] = '\r'; resp[pos++] = '\n';

						CDC_Transmit_FS((uint8_t*)resp, pos);
                    }
                }
                else if (strncmp((char*)cmd_buffer, "get_v", 5) == 0)
                {
                    SendVoltageNow();
                }
                else if (strncmp((char*)cmd_buffer, "get_ci", 6) == 0)
				{
                	PrintSavedCurrentCoefficients();
				}
                else if (strncmp((char*)cmd_buffer, "get_cv", 6) == 0)
				{
                	PrintSavedVoltageCoefficients();
				}
                else if (strncmp((char*)cmd_buffer, "get_i", 5) == 0)
                {
                    SendCurrentNow();
                }
                else if (strncmp((char*)cmd_buffer, "cal_v", 5) == 0)
                {
                    CalStart_Voltage();
                    char resp[] = "OK CAL_V START\r\n";
                    CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                }
                else if (strncmp((char*)cmd_buffer, "cal_i", 5) == 0)
                {
                    CalStart_Current();
                    char resp[] = "OK CAL_I START\r\n";
                    CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                }
                else if (strncmp((char*)cmd_buffer, "cal_s", 5) == 0)
                {
                    char resp[96];
                    uint8_t resp_len = 0;
                    CalStop_AndCompute(resp, &resp_len);
                    CDC_Transmit_FS((uint8_t*)resp, resp_len);
                }
                else if (strncmp((char*)cmd_buffer, "vreal", 5) == 0)
				{
                	char *arg_ptr = SkipSpaces((char*)&cmd_buffer[5]);
					float real_value = MyAtof(arg_ptr);
					uint8_t ok = CalAddPoint_Voltage(real_value);

					if (ok)
					{
						char resp[] = "OK POINT ADDED\r\n";
						CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
					}
					else
					{
						char resp[] = "ERR NOT IN CAL_V MODE OR FULL\r\n";
						CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
					}
				}
                else if (strncmp((char*)cmd_buffer, "ireal", 5) == 0)
				{
                	char *arg_ptr = SkipSpaces((char*)&cmd_buffer[5]);
					float real_value = MyAtof(arg_ptr);
					uint8_t ok = CalAddPoint_Current(real_value);

					if (ok)
					{
						char resp[] = "OK POINT ADDED\r\n";
						CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
					}
					else
					{
						char resp[] = "ERR NOT IN CAL_I MODE OR FULL\r\n";
						CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
					}
				}
                else if (strncmp((char*)cmd_buffer, "on", 2) == 0)
                {
                    debug_output_enabled = 1;

                    char resp[] = "OK DEBUG_ON\r\n";
                    CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                }
                else if (strncmp((char*)cmd_buffer, "off", 3) == 0)
                {
                    debug_output_enabled = 0;

                    char resp[] = "OK DEBUG_OFF\r\n";
                    CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                }
                else if (strncmp((char*)cmd_buffer, "set_f", 5) == 0)
                {
                	char *arg_ptr = SkipSpaces((char*)&cmd_buffer[5]);
                    uint8_t number = MyAtoi(arg_ptr);

                    if (number < 1) number = 1;   // защита от 0 (деление/остаток по модулю 0 -> UB)

                    debug_report_interval_ms = (uint32_t)number * 50;

                    char resp[32];
                    uint8_t pos = 0;
                    const char prefix[] = "OK INTERVAL=";
                    memcpy(resp, prefix, strlen(prefix));
                    pos = strlen(prefix);
                    pos += UIntToStr((uint32_t)number, &resp[pos]);
                    resp[pos++] = 'm';
                    resp[pos++] = 's';
                    resp[pos++] = '\r'; resp[pos++] = '\n';

                    CDC_Transmit_FS((uint8_t*)resp, pos);
                }
                else if (strncmp((char*)cmd_buffer, "set_v", 5) == 0)
				{
                	char *arg_ptr = SkipSpaces((char*)&cmd_buffer[5]);
					float target = MyAtof(arg_ptr);
					SetResult_t result = SetTargetVoltage(target);
					SendSetResultResponse(result, "SETTING_V");
				}
                else if (strncmp((char*)cmd_buffer, "set_i", 5) == 0)
				{
                	char *arg_ptr = SkipSpaces((char*)&cmd_buffer[5]);
					float target_i = MyAtof(arg_ptr);

					SetResult_t result = SetTargetCurrent(target_i);
					SendSetResultResponse(result, "SETTING_I");
				}
                else if (strncmp((char*)cmd_buffer, "set_pid_v_kp", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_v.Kp = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_V_KP\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "set_pid_v_ki", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_v.Ki = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_V_KI\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "set_pid_v_kd", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_v.Kd = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_V_KD\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "set_pid_i_kp", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_i.Kp = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_I_KP\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "set_pid_i_ki", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_i.Ki = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_I_KI\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "set_pid_i_kd", 12) == 0)
				{
					char *arg_ptr = SkipSpaces((char*)&cmd_buffer[12]);
					pid_i.Kd = MyAtof(arg_ptr);
					SaveCoefficientsToFlash();
					char resp[] = "OK SET_PID_I_KD\r\n";
					CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
				}
				else if (strncmp((char*)cmd_buffer, "get_kv", 6) == 0)
				{
					char msg[64];
					char kp_s[16], ki_s[16], kd_s[16];
					FloatToStr(pid_v.Kp, kp_s, 3);
					FloatToStr(pid_v.Ki, ki_s, 3);
					FloatToStr(pid_v.Kd, kd_s, 3);

					uint8_t pos = 0;
					const char prefix[] = "PID_V: Kp=";
					memcpy(msg, prefix, strlen(prefix)); pos += strlen(prefix);
					memcpy(&msg[pos], kp_s, strlen(kp_s)); pos += strlen(kp_s);
					msg[pos++] = ' '; msg[pos++] = 'K'; msg[pos++] = 'i'; msg[pos++] = '=';
					memcpy(&msg[pos], ki_s, strlen(ki_s)); pos += strlen(ki_s);
					msg[pos++] = ' '; msg[pos++] = 'K'; msg[pos++] = 'd'; msg[pos++] = '=';
					memcpy(&msg[pos], kd_s, strlen(kd_s)); pos += strlen(kd_s);
					msg[pos++] = '\r'; msg[pos++] = '\n';

					CDC_Transmit_FS((uint8_t*)msg, pos);
				}
				else if (strncmp((char*)cmd_buffer, "get_ki", 6) == 0)
				{
					char msg[64];
					char kp_s[16], ki_s[16], kd_s[16];
					FloatToStr(pid_i.Kp, kp_s, 3);
					FloatToStr(pid_i.Ki, ki_s, 3);
					FloatToStr(pid_i.Kd, kd_s, 3);

					uint8_t pos = 0;
					const char prefix[] = "PID_I: Kp=";
					memcpy(msg, prefix, strlen(prefix)); pos += strlen(prefix);
					memcpy(&msg[pos], kp_s, strlen(kp_s)); pos += strlen(kp_s);
					const char ki_p[] = " Ki=";
					memcpy(&msg[pos], ki_p, strlen(ki_p)); pos += strlen(ki_p);
					memcpy(&msg[pos], ki_s, strlen(ki_s)); pos += strlen(ki_s);
					const char kd_p[] = " Kd=";
					memcpy(&msg[pos], kd_p, strlen(kd_p)); pos += strlen(kd_p);
					memcpy(&msg[pos], kd_s, strlen(kd_s)); pos += strlen(kd_s);
					msg[pos++] = '\r'; msg[pos++] = '\n';

					CDC_Transmit_FS((uint8_t*)msg, pos);
				}
                else
                {
                    char resp[] = "ERR UNKNOWN CMD\r\n";
                    CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
                }

                cmd_index = 0;
            }
        }
        else
        {
            if (cmd_index < sizeof(cmd_buffer) - 1)
            {
                cmd_buffer[cmd_index++] = c;
            }
            else
            {
                cmd_index = 0;
            }
        }
    }


	  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
