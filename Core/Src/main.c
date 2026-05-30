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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  PM_INT,
  PM_UINT,
  PM_FLOAT,
  PM_DOUBLE
} PrintMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LORA_SPED      0x1C
#define LORA_CHANNEL   0x06
#define LORA_OPTION    0xC4

/*
 * Estrutura do pacote recebido via UART3 (bytes de roteamento 0xFF 0xFF CANAL
 * sao consumidos internamente pelo E32 em Fixed Transmission):
 *
 * [0]      = CAN ID byte 3  (MSB)
 * [1]      = CAN ID byte 2
 * [2]      = CAN ID byte 1
 * [3]      = CAN ID byte 0  (LSB)
 * [4]      = tamanho do payload CAN (N bytes)
 * [5..N+4] = dados CAN
 */
#define PKT_HEADER_SIZE  5     /* 4 bytes CAN ID + 1 byte tamanho */
#define MAX_PAYLOAD_SIZE 300

#define RX_HEADER_TIMEOUT_MS  2000
#define RX_DATA_TIMEOUT_MS    1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t pktHeader[PKT_HEADER_SIZE];
uint8_t pktData[MAX_PAYLOAD_SIZE];
uint8_t packetSize;
uint32_t canID;

PrintMode print_mode = PM_INT;
uint32_t uint_result[8];
int32_t  int_result[8];
double double_result;
float    float_result[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void disp(char *msg);
void e32_waitAUX(uint32_t timeoutMs);
void e32_setMode(uint8_t m1, uint8_t m0);
uint8_t e32_configureModule(void);
int e32_Receive(uint32_t *outCanID, uint8_t *outData, uint8_t *outSize);

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline uint16_t u16(uint8_t hi, uint8_t lo) {
  return (uint16_t)((uint16_t)hi << 8 | (uint16_t)lo);
}
static inline int16_t s16(uint8_t hi, uint8_t lo) {
  return (int16_t)u16(hi, lo);
}
static inline uint64_t u64(uint8_t b7, uint8_t b6, uint8_t b5, uint8_t b4,
                           uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
{
    return ((uint64_t)b7 << 56) |
           ((uint64_t)b6 << 48) |
           ((uint64_t)b5 << 40) |
           ((uint64_t)b4 << 32) |
           ((uint64_t)b3 << 24) |
           ((uint64_t)b2 << 16) |
           ((uint64_t)b1 << 8 ) |
           ((uint64_t)b0);
}

static inline int64_t s64(uint8_t b7, uint8_t b6, uint8_t b5, uint8_t b4,
                          uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
{
    return (int64_t)u64(b7, b6, b5, b4, b3, b2, b1, b0);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  disp("FORMULA TESLA UFMG - TELEMETRIA V4 - PLACA RECEPTORA\r\n");

  e32_configureModule();


  disp("[MODE] Normal Mode...\r\n");

  e32_setMode(0, 0);

  disp("[RX] Aguardando pacotes...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (e32_Receive(&canID, pktData, &packetSize) == 0)
	{
	  HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);

      switch (canID) {
          case 76:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]); // SPEED_AVG
              float_result[1] = (float) u16(pktData[3], pktData[2]); // STEERING_WHEEL
              float_result[2] = (float) u16(pktData[5], pktData[4]); // THROTTLE
              float_result[3] = (float) u16(pktData[7], pktData[6]); // BRAKE
              print_mode = PM_FLOAT;
              break;

          case 77:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // MODE
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // TORQUER_GAIN
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // DISTANCE_P_ODOM
              uint_result[3] = (uint32_t) u16(pktData[7], pktData[6]); // DISTANCE_T_ODOM
              print_mode = PM_UINT;
              break;

          case 78:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // CONTROL_EVENT_FLAG_1
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // CONTROL_EVENT_FLAG_2
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // REF_TORQUE_R_MOTOR
              uint_result[3] = (uint32_t) u16(pktData[7], pktData[6]); // REF_TORQUE_L_MOTOR
              print_mode = PM_UINT;
              break;

          case 79:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]); // SPEED_LF
              float_result[1] = (float) u16(pktData[3], pktData[2]); // SPEED_LR
              float_result[2] = (float) u16(pktData[5], pktData[4]); // SPEED_RL
              float_result[3] = (float) u16(pktData[7], pktData[6]); // SPEED_RR
              print_mode = PM_FLOAT;
              break;

          case 80:
              memset(int_result,  0, sizeof(int_result));
              int_result[0] = (int32_t) s16(pktData[1], pktData[0]); // ID_PANEL_DEBUG_1
              int_result[1] = (int32_t) s16(pktData[3], pktData[2]); // ID_PANEL_DEBUG_2
              int_result[2] = (int32_t) s16(pktData[5], pktData[4]); // ID_PANEL_DEBUG_3
              int_result[3] = (int32_t) s16(pktData[7], pktData[6]); // ID_PANEL_DEBUG_4
              print_mode = PM_INT;
              break;

          case 81:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // ID_REGEN_BRAKE_STATE
              print_mode = PM_UINT;
              break;

          case 85:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) s16(pktData[1], pktData[0]); // SPEED_L_MOTOR
              float_result[1] = (float) s16(pktData[3], pktData[2]); // TORQUE_L_MOTOR
              float_result[2] = (float) s16(pktData[5], pktData[4]); // POWER_L_MOTOR
              float_result[3] = (float) s16(pktData[7], pktData[6]); // CURRENT_L_MOTOR
              print_mode = PM_FLOAT;
              break;

          case 86:
              memset(int_result,  0, sizeof(int_result));
              int_result[0] = (int32_t) s16(pktData[1], pktData[0]); // ENERGY_L_MOTOR
              int_result[1] = (int32_t) s16(pktData[3], pktData[2]); // OVERLOAD_L_MOTOR
              int_result[2] = (int32_t) s16(pktData[5], pktData[4]); // TEMPERATURE1_L
              int_result[3] = (int32_t) s16(pktData[7], pktData[6]); // TEMPERATURE2_L
              print_mode = PM_INT;
              break;

          case 87:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // ID_LOST_MSG_L_MOTOR
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // ID_BUS_OFF_L_MOTOR
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // ID_CAN_STATE_L_MOTOR
              print_mode = PM_UINT;
              break;

          case 88:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // ID_INV_STATE_L_MOTOR
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // ID_FAILURE_L_MOTOR
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // ID_ALARM_L_MOTOR
              print_mode = PM_UINT;
              break;

          case 95:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) s16(pktData[1], pktData[0]); // ID_SPEED_R_MOTOR
              float_result[1] = (float) s16(pktData[3], pktData[2]); // ID_TORQUE_R_MOTOR
              float_result[2] = (float) s16(pktData[5], pktData[4]); // ID_POWER_R_MOTOR
              float_result[3] = (float) s16(pktData[7], pktData[6]); // ID_CURRENT_R_MOTOR
              print_mode = PM_FLOAT;
              break;

          case 96:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]); // ID_ENERGY_R_MOTOR
              float_result[1] = (float) u16(pktData[3], pktData[2]); // ID_OVERLOAD_R_MOTOR
              float_result[2] = (float) u16(pktData[5], pktData[4]); // ID_TEMPERATURE1_R
              float_result[3] = (float) u16(pktData[7], pktData[6]); // ID_TEMPERATURE2_R
              print_mode = PM_FLOAT;
              break;

          case 97:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // ID_LOST_MSG_R_MOTOR
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // ID_BUS_OFF_R_MOTOR
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // ID_CAN_STATE_R_MOTOR
              print_mode = PM_UINT;
              break;

          case 98:
              memset(uint_result, 0, sizeof(uint_result));
              uint_result[0] = (uint32_t) u16(pktData[1], pktData[0]); // ID_INV_STATE_R_MOTOR
              uint_result[1] = (uint32_t) u16(pktData[3], pktData[2]); // ID_FAILURE_R_MOTOR
              uint_result[2] = (uint32_t) u16(pktData[5], pktData[4]); // ID_ALARM_R_MOTOR
              print_mode = PM_UINT;
              break;

          case 259:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) s16(pktData[1], pktData[0]); // AcelX
              float_result[1] = (float) s16(pktData[3], pktData[2]); // AcelY
              float_result[2] = (float) s16(pktData[5], pktData[4]); // AcelZ
              print_mode = PM_FLOAT;
              break;

          case 260:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) s16(pktData[1], pktData[0]); // GyroX
              float_result[1] = (float) s16(pktData[3], pktData[2]); // GyroY
              float_result[2] = (float) s16(pktData[5], pktData[4]); // GyroZ
              print_mode = PM_FLOAT;
              break;

          case 261:
              double_result = 0;
              double_result = (double) s64(pktData[7], pktData[6], pktData[5], pktData[4], pktData[3], pktData[2], pktData[1], pktData[0]); // Temp
              print_mode = PM_DOUBLE;
              break;

          case 262:
        	  double_result = 0;
				double_result = (double)s64(pktData[7], pktData[6], pktData[5], pktData[4], pktData[3], pktData[2], pktData[1], pktData[0]); // Temp
				print_mode = PM_DOUBLE;
				break;

          case 263:
        	  double_result = 0;
				double_result = (double)s64(pktData[7], pktData[6], pktData[5], pktData[4], pktData[3], pktData[2], pktData[1], pktData[0]);  // Temp
				print_mode = PM_DOUBLE;
				break;



          case 361:
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]); // ID_PANEL_DEBUG_1
              float_result[1] = (float) u16(pktData[3], pktData[2]); // ID_PANEL_DEBUG_2
              float_result[2] = (float) u16(pktData[5], pktData[4]); // ID_PANEL_DEBUG_3
              float_result[3] = (float) u16(pktData[7], pktData[6]); // ID_PANEL_DEBUG_4
              print_mode = PM_FLOAT;
              break;

          case 300: // STACK 1
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 301: // STACK 2
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 302: // STACK 3
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 303: // STACK 4
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 304: // STACK 5
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 305: // STACK 6
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // CELL_1
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // CELL_2
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10000.0f; // CELL_3
              float_result[3] = (float) u16(pktData[7], pktData[6]) / 10000.0f; // CELL_4
              print_mode = PM_FLOAT;
              break;

          case 306: // ACCUMULATOR PARAMS
              memset(float_result, 0, sizeof(float_result));
              float_result[0] = (float) u16(pktData[1], pktData[0]) / 10000.0f; // MIN_VOLTAGE
              float_result[1] = (float) u16(pktData[3], pktData[2]) / 10000.0f; // MAX_VOLTAGE
              float_result[2] = (float) u16(pktData[5], pktData[4]) / 10.0f;    // TOTAL_VOLTAGE
              float_result[3] = (float) u16(pktData[7], pktData[6]);            // SHUNT CURRENT
              print_mode = PM_FLOAT;
              break;

          case 307: // BMS PARAMS
              memset(int_result,  0, sizeof(int_result));
              int_result[0] = (int32_t) s16(pktData[1], pktData[0]); // BMS_MODE
              int_result[1] = (int32_t) s16(pktData[3], pktData[2]); // BMS_ERROR
              int_result[2] = (int32_t) s16(pktData[5], pktData[4]); // AIR_P
              int_result[3] = (int32_t) s16(pktData[7], pktData[6]); // AIR_N
              print_mode = PM_INT;
              break;

          default:
              memset(float_result, 0, sizeof(float_result));
              for (int i = 0; i < 8; i++){
                  float_result[i] = (float) pktData[i];
              }
              print_mode = PM_FLOAT;
              break;
      }


	  printf("%lu", canID); // Imprime o ID

	  // Imprime os dados conforme o modo
	  for (size_t i = 0; i < 8; i++) {
	      printf(","); // Vírgula separadora
	      if (print_mode == PM_UINT) {
	          printf("%lu", uint_result[i]);
	      } else if (print_mode == PM_INT) {
	          printf("%ld", int_result[i]);
	      } else if (print_mode == PM_FLOAT){ // PM_FLOAT
	          printf("%.2f", float_result[i]);
	      } else {
	          printf("%lf", double_result);
	          break;
	      }
	  }
	  printf("\r\n"); // Quebra de linha final

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void disp(char *msg)
{
    HAL_UART_Transmit(
        &huart1,
        (uint8_t*)msg,
        strlen(msg),
        HAL_MAX_DELAY
    );
}

void e32_waitAUX(uint32_t timeoutMs)
{
    uint32_t tickstart = HAL_GetTick();

    while(HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
    {
        if((HAL_GetTick() - tickstart) > timeoutMs)
        {
            disp("[AUX] Timeout!\r\n");
            return;
        }
    }
}

void e32_setMode(uint8_t m1, uint8_t m0)
{
    HAL_GPIO_WritePin(
        E32_M1_GPIO_Port,
        E32_M1_Pin,
        m1 ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        E32_M0_GPIO_Port,
        E32_M0_Pin,
        m0 ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    e32_waitAUX(500);
}

uint8_t e32_configureModule(void)
{
    disp("[CFG] Sleep Mode...\r\n");

    e32_setMode(1,1);

    HAL_Delay(100);

    uint8_t dummy;

    while(HAL_UART_Receive(&huart3, &dummy, 1, 10) == HAL_OK);

    uint8_t cfg[6] = {
        0xC0,
        0x00,
        0x00,
        LORA_SPED,
        LORA_CHANNEL,
        LORA_OPTION
    };

    disp("[CFG] Sending config...\r\n");

    HAL_UART_Transmit(
        &huart3,
        cfg,
        6,
        HAL_MAX_DELAY
    );

    HAL_Delay(300);

    uint8_t resp[6] = {0};

    HAL_UART_Receive(
        &huart3,
        resp,
        6,
        800
    );

    if(resp[0] == 0xC0)
    {
        disp("[CFG] OK!\r\n");
        return 1;
    }
    else
    {
        disp("[CFG] Using existing config.\r\n");
        return 0;
    }
}

int e32_Receive(uint32_t *outCanID, uint8_t *outData, uint8_t *outSize)
{
    //e32_waitAUX(2000);

    uint8_t buffer[12];

    if(HAL_UART_Receive(&huart3, buffer, 12, 2000) != HAL_OK)
    {
        return -1;
    }

    *outCanID = ((uint32_t)buffer[0] << 24) |
                ((uint32_t)buffer[1] << 16) |
                ((uint32_t)buffer[2] << 8)  |
                ((uint32_t)buffer[3]);

    for(int i = 0; i < 8; i++)
    {
        outData[i] = buffer[4 + i];
    }

    if (outSize != NULL)
    {
        *outSize = 8;
    }

    return 0;
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
  * where the assert_param error has occurred.
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
