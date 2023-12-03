/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PORT	22
#define SERVER	"192.168.2.123"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart3;

osThreadId StartHandle;
/* USER CODE BEGIN PV */
osThreadId ClientHandle;
int sock, g;
struct sockaddr_in address;
err_t err = -1;

char uart_bufT[1000];
char get_info[1000];
int get_state = 0;
//char uart_bufR[100];
int uart_buf_len;

char spi_buf[100];
char spi_addr[20];

const uint16_t READ_Intruction = 0x8000;

const uint16_t Config_Addr = 0x0000;
const uint8_t SDO_Active = 0x18 | 0x81; // 0x81 or 0x01

const uint16_t Read_Buffer_Addr = 0x0004;
const uint8_t Read_Buffer = 0x00 | 0x01;	// read buffered registers instead of currently used
const uint8_t Read_Currently = 0x00;

const uint16_t UPD_Addr = 0x0005;
const uint8_t UPD_Auto = 0x00 | 0x01;	// Automatically Update registers

const uint16_t Power_Addr = 0x0010;
const uint8_t PLL_Bypassed = 0xD0;	// Power of PLL_Multiplier will be down
const uint8_t PLL_Enabled = 0xC0;	// Power of PLL_Multiplier will be up

const uint16_t N_divider_Addr = 0x0020;
uint8_t N_Divider = 0x03;	// range 0x00 to 0x1F (0 to 31) + 2 = 2 to 33. def. is 0x12 (18) + 2 = 20 (x 2) = 40 (sysclk = 25 MHz) = 1 GHz

const uint16_t FTW_Addr6 = 0x01A6;
const uint16_t FTW_Addr7 = 0x01A7;
const uint16_t FTW_Addr8 = 0x01A8;
const uint16_t FTW_Addr9 = 0x01A9;
const uint16_t FTW_AddrA = 0x01AA;
const uint16_t FTW_AddrB = 0x01AB;
double f_s = 1000.0;	// MHz of sysclk
double f_DDS = 0.0;	// MHz of initial frequency
double f_ref = 100.0;	// MHz of reference frequency

const uint16_t DAC_Current_AddrB = 0x040B;
const uint16_t DAC_Current_AddrC = 0x040C;
const double I_DAC_REF = 0.120;	// mA if R_DAC_REF = 10 k[ohm]
const double I_DAC_FS = 10.0;	// mA in output

char help[] = "\nCorrect format for communication with DDS AD9912:\r\n"
			  "\r\n"
			  "for getting help!\r\n"
			  "help\r\n"
			  "\r\n"
			  "for closing connection!\r\n"
			  "exit\r\n"
			  "\r\n"
			  "for sending values:\r\n"
			  "DDS:state val\r\n"
			  "\r\n"
			  "EXAMPLE:\r\n"
			  "DDS:FREQ 120.56842\r\n"
			  "\r\n"
			  "for sending more state:\r\n"
			  "DDS:state val;state val;...\r\n"
			  "\r\n"
			  "for getting values:\r\n"
			  "DDS:state?\r\n"
		      "\r\n"
			  "for getting MAX and MIN values:\r\n"
			  "DDS:state :MAX\r\n"
			  "and\r\n"
			  "DDS:state :MIN\r\n"
			  "\r\n"
			  "all states:\r\n"
			  "state1: FREQ; state2: AMP; state3: REF\r\n"
			  "\r\n"
			  "REF should be 100 MHz, 250 MHz, or 1000 MHz\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART3_UART_Init(void);
void StartThread(void const * argument);

/* USER CODE BEGIN PFP */
void AcceptanceNewClient(int * argument);
int ExtractMessage(char* msg);
void send_freq(double fdds);
void send_current(double idac);
void set_ref(double ref);
void get_freq(void);
void get_max_freq(void);
void get_min_freq(void);
void get_current(void);
void get_max_current(void);
void get_min_current(void);
void get_ref(void);
void get_max_ref(void);
void get_min_ref(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // CS should be High by default
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // star communication by UART
  uart_buf_len = sprintf(uart_bufT, "Ethernet Communication with AD9912\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

  // RESET DDS chip
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  // active SDO Pin
  spi_addr[0] = ((uint8_t*)&Config_Addr)[1];
  spi_addr[1] = ((uint8_t*)&Config_Addr)[0];
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&SDO_Active, 1, 100);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // active Automatically update registers
//  spi_addr[0] = ((uint8_t*)&UPD_Addr)[1];
//  spi_addr[1] = ((uint8_t*)&UPD_Addr)[0];
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//  HAL_SPI_Transmit(&hspi4, (uint8_t*)&UPD_Auto, 1, 100);
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // read buffered registers instead of currently used
//  spi_addr[0] = ((uint8_t*)&Read_Buffer_Addr)[1];
//  spi_addr[1] = ((uint8_t*)&Read_Buffer_Addr)[0];
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//  HAL_SPI_Transmit(&hspi4, (uint8_t*)&Read_Buffer, 1, 100);
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // active PLL bypassed or enabled
  spi_addr[0] = ((uint8_t*)&Power_Addr)[1];
  spi_addr[1] = ((uint8_t*)&Power_Addr)[0];
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//  HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Bypassed, 1, 100);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Enabled, 1, 100);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // set N-divider for PLL
  spi_addr[0] = ((uint8_t*)&N_divider_Addr)[1];
  spi_addr[1] = ((uint8_t*)&N_divider_Addr)[0];
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
  HAL_SPI_Transmit(&hspi4, (uint8_t*)&N_Divider, 1, 100);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Start */
  osThreadDef(Start, StartThread, osPriorityNormal, 0, 512);
  StartHandle = osThreadCreate(osThread(Start), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RESET_Pin|CS_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_Pin CS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|CS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSH_BTN_Pin */
  GPIO_InitStruct.Pin = PUSH_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSH_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_UPD_Pin */
  GPIO_InitStruct.Pin = IO_UPD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IO_UPD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void AcceptanceNewClient(int * argument)
{

	char msg[200] = {};
	int newVal = 0, state = 1;
	int conn = *argument;

	while(1)
	{

		if(newVal == 3){

			close(conn);
			uart_buf_len = sprintf(uart_bufT, "finished! \r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
			osThreadTerminate(NULL);
//			osThreadSuspend(NULL);
		}else if(newVal == 2){

			newVal = 0;
		}else{

			memset(msg, 0, sizeof msg);
			state = read(conn, (char*)msg, 200);
			newVal = ExtractMessage(msg);
			send(conn, (char*)uart_bufT, strlen(uart_bufT), 0);	// we have to use send instead of write for avoiding of crash

			if(state <= 0){

				newVal = 3;
			}
		}
		osDelay(100);
	}
}

int ExtractMessage(char* msg){

	char temp[100] = {};
	int j = 0, f1 = 1, f2 = 1, f3 = 0;
	memset(get_info, 0, sizeof get_info);

	for(int i = 0; i < strlen(msg); i++){

		if(msg[i] == ':'){

			if(strcmp(temp, "DDS") != 0){

				uart_buf_len = sprintf(uart_bufT, "wrong msg: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
				HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
				return 0;
			}else{

				f1 = 1;
				j = 0;
				memset(temp, 0, sizeof temp);
				while(f1){

					i++;

					if(msg[i] == ' ' || msg[i] == '?'){

						f2 = 1;

						if(strcmp(temp, "FREQ") == 0){

							j = 0;
							memset(temp, 0, sizeof temp);

							if(msg[i] == '?'){

								get_freq();
								f2 = 0;
								f3 = 1;
							}

							while(f2){

								i++;

								if(strcmp(temp, ":MAX") == 0){

									get_max_freq();
								}else if(strcmp(temp, ":MIN") == 0){

									get_min_freq();
								}else if(msg[i] == ';'){

									send_freq(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){

									send_freq(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else{

									temp[j] = msg[i];
									j++;
									continue;
								}

								f2 = 0;
								f3 = 1;
							}
						}else if(strcmp(temp, "AMP") == 0){

							j = 0;
							memset(temp, 0, sizeof temp);

							if(msg[i] == '?'){

								get_current();
								f2 = 0;
								f3 = 1;
							}

							while(f2){

								i++;

								if(strcmp(temp, ":MAX") == 0){

									get_max_current();
								}else if(strcmp(temp, ":MIN") == 0){

									get_min_current();
								}else if(msg[i] == ';'){

									send_current(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){

									send_current(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else{

									temp[j] = msg[i];
									j++;
									continue;
								}

								f2 = 0;
								f3 = 1;
							}
						}else if(strcmp(temp, "REF") == 0){

							j = 0;
							memset(temp, 0, sizeof temp);

							if(msg[i] == '?'){

								get_ref();
								f2 = 0;
								f3 = 1;
							}

							while(f2){

								i++;

								if(strcmp(temp, ":MAX") == 0){

									get_max_ref();
								}else if(strcmp(temp, ":MIN") == 0){

									get_min_ref();
								}else if(msg[i] == ';'){

									set_ref(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){

									set_ref(atof(temp));
									j = 0;
									memset(temp, 0, sizeof temp);
								}else{

									temp[j] = msg[i];
									j++;
									continue;
								}

								f2 = 0;
								f3 = 1;
							}
						}else{

							uart_buf_len = sprintf(uart_bufT, "the format is wrong: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
							HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
							return 0;
						}
					}else{

						temp[j] = msg[i];
						j++;
						continue;
					}

					while(f3){

						if(msg[i] == ';'){

							f3 = 0;
							continue;
						}else if(i >= strlen(msg)){

							f3 = 0;
							f1 = 0;
							uart_buf_len = sprintf(uart_bufT, "the message has been registered!\r\n");
							HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

							if(get_state){
								strcat(get_info, uart_bufT);
								strcpy(uart_bufT, get_info);
								get_state = 0;
							}

							return 2;
						}
						i++;
					}
				}
			}
		}else{

			temp[j] = msg[i];

			if(strcmp(temp, "exit") == 0 || strcmp(temp, "Exit") == 0 || strcmp(temp, "EXIT") == 0){

				uart_buf_len = sprintf(uart_bufT, "Exit!\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

				return 3;
			}else if(strcmp(temp, "help") == 0 || strcmp(temp, "Help") == 0 || strcmp(temp, "HELP") == 0){

				uart_buf_len = sprintf(uart_bufT, "%s\r\n", (char*)help);
				HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

				return 1;
			}

			j++;
			continue;
		}
	}

	uart_buf_len = sprintf(uart_bufT, "wrong msg: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
	return 0;
}

void send_freq(double fdds)
{
	uint64_t ftw;
	uint8_t value;

	if(fdds > 400.0){
		fdds = 400.0;
	}

	ftw = round((fdds/f_s) * pow(2, 48));

	spi_addr[0] = ((uint8_t*)&FTW_Addr6)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr6)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[0];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr7)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr7)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[1];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr8)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr8)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[2];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr9)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr9)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[3];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_AddrA)[1];
	spi_addr[1] = ((uint8_t*)&FTW_AddrA)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[4];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_AddrB)[1];
	spi_addr[1] = ((uint8_t*)&FTW_AddrB)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[5];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	// Update registers
	HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_SET);
	osDelay(1);
	HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_RESET);
}

void send_current(double idac)
{
	uint16_t fsc;
	uint8_t value;

	if(idac < 8.6){

		idac = 8.6;
	}else if(idac > 31.7){

		idac = 31.7;
	}

	fsc = round(1024/192 * (idac/I_DAC_REF - 72));

	spi_addr[0] = ((uint8_t*)&DAC_Current_AddrB)[1];
	spi_addr[1] = ((uint8_t*)&DAC_Current_AddrB)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&fsc)[0];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&DAC_Current_AddrC)[1];
	spi_addr[1] = ((uint8_t*)&DAC_Current_AddrC)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&fsc)[1];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);
}

void set_ref(double ref)
{
	f_ref = ref;

	spi_addr[0] = ((uint8_t*)&Power_Addr)[1];
	spi_addr[1] = ((uint8_t*)&Power_Addr)[0];

	if(ref == 100.0){

		f_s = 1000.0;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Enabled, 1, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//		// set N-divider for PLL
//		spi_addr[0] = ((uint8_t*)&N_divider_Addr)[1];
//		spi_addr[1] = ((uint8_t*)&N_divider_Addr)[0];
//		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//		HAL_SPI_Transmit(&hspi4, (uint8_t*)&N_Divider, 1, 100);
//		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	}else if(ref == 250.0 || ref == 1000.0){

		f_s = ref;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Bypassed, 1, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		send_freq(f_DDS);
	}
}

void get_freq(){

	uint16_t address;
	uint64_t ftw = 0x0;
	double fDDS = 0;
//	uint16_t fcs = 0x0;
//	double IDAC = 0;

//	// read serial config. value
//	address = READ_Intruction | Config_Addr;
//	spi_addr[0] = ((uint8_t*)&address)[1];
//	spi_addr[1] = ((uint8_t*)&address)[0];
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//
//	uart_buf_len = sprintf(uart_bufT,
//			"Currently Serial Config. Value at the Address %#x is: %#x\r\n",
//			address, (unsigned int)spi_buf[0]);
//	strcat(get_info, uart_bufT);
//	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
//
//	// read power down and enabled value
//	address = READ_Intruction | Power_Addr;
//	spi_addr[0] = ((uint8_t*)&address)[1];
//	spi_addr[1] = ((uint8_t*)&address)[0];
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//
//	uart_buf_len = sprintf(uart_bufT,
//			"Currently Power Down Value at the Address %#x is: %#x\r\n",
//			address, (unsigned int)spi_buf[0]);
//	strcat(get_info, uart_bufT);
//	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
//
//	// read N-Divider value
//	address = READ_Intruction | N_divider_Addr;
//	spi_addr[0] = ((uint8_t*)&address)[1];
//	spi_addr[1] = ((uint8_t*)&address)[0];
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//
//	uart_buf_len = sprintf(uart_bufT,
//			"Currently N-Divider Value at the Address %#x is: %#x\r\n",
//			address, (unsigned int)spi_buf[0]);
//	strcat(get_info, uart_bufT);
//	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	// read freq. value
	address = READ_Intruction | FTW_Addr6;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[0] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr7;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[1] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr8;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[2] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr9;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[3] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_AddrA;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[4] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_AddrB;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[5] = (unsigned int)spi_buf[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	fDDS = (ftw/pow(2, 48)) * f_s;
	uart_buf_len = sprintf(uart_bufT,
			"Currently Frequency Value is: %0.15f MHz with FTW: %#lx%lx\r\n",
			fDDS, ((uint32_t *)&ftw)[1], ((uint32_t *)&ftw)[0]);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_max_freq(){

	double maxf = 0.4*f_s;

	uart_buf_len = sprintf(uart_bufT,
			"Maximum Output Frequency Value is: %0.3f MHz\r\n",
			maxf);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_min_freq(){

	uart_buf_len = sprintf(uart_bufT,
			"Minimum Output Frequency Value is: %0.3f MHz\r\n",
			0.0);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_current(){

	uint16_t address;
	uint16_t fcs = 0x0;
	double IDAC = 0;

	// read DAC Full Scale Current value
	address = READ_Intruction | DAC_Current_AddrB;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&fcs)[0] = (unsigned int)spi_buf[0];
	address = READ_Intruction | DAC_Current_AddrC;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&fcs)[1] = (unsigned int)spi_buf[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	IDAC = (72 + (192 * fcs)/pow(2, 10)) * I_DAC_REF;
	uart_buf_len = sprintf(uart_bufT,
			"Currently Output Current Value is: %0.3f mA with FSC: %#x\r\n",
			IDAC, fcs);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_max_current(){

	uart_buf_len = sprintf(uart_bufT,
			"Maximum Current Value is: %0.3f mA\r\n",
			31.7);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_min_current(){

	uart_buf_len = sprintf(uart_bufT,
			"Minimum Current Value is: %0.3f mA\r\n",
			8.6);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_ref(){

	uart_buf_len = sprintf(uart_bufT,
			"Currently Reference Frequency Value is: %0.3f MHz\r\n",
			f_ref);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_max_ref(){

	uart_buf_len = sprintf(uart_bufT,
			"Maximum Reference Frequency Value is: %0.3f MHz\r\n",
			1000.0);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void get_min_ref(){

	uart_buf_len = sprintf(uart_bufT,
			"Minimum Output Frequency Value is: %0.3f MHz\r\n",
			100.0);
	strcat(get_info, uart_bufT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

	get_state = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==PUSH_BTN_Pin)
  {

	  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartThread */
/**
  * @brief  Function implementing the Start thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartThread */
void StartThread(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  sock = socket(AF_INET, SOCK_STREAM, 0);

  address.sin_family = AF_INET;
  address.sin_port = htons(PORT);
  address.sin_addr.s_addr = INADDR_ANY;//inet_addr(SERVER);//INADDR_ANY;

  err = bind(sock, (struct sockaddr *)&address, sizeof (address));
  err = listen(sock, 0);

  // create the acceptance thread
  osThreadDef(Acceptance, AcceptanceNewClient, osPriorityLow, 0, configMINIMAL_STACK_SIZE *2);

  send_freq(f_DDS);

  /* Infinite loop */
  for(;;)
  {
		g =  accept(sock, NULL, NULL);

		if(g < 0){

			osDelay(100);
			continue;
		}

		ClientHandle = osThreadCreate(osThread(Acceptance), &g);

		uart_buf_len = sprintf(uart_bufT, "new connection...! \r\n"
									      "Connected to DDS_AD9912! \r\n\n%s\r\n", (char*)help);
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
		write(g, (char*)uart_bufT, strlen(uart_bufT));
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

		osDelay(100);
  }
  /* USER CODE END 5 */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
