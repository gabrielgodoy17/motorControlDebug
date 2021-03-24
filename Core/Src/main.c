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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include <stdio.h>
#include <inttypes.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t byte;
//uint8_t byte,
uint8_t indRx=0, flagRx, imprimir = 1, sentido;
char buffer[MAX_BUFFER];
uint8_t in_buffer[17];
uint8_t in_second_buffer[7]; //la logica muy rancia logica pero sirve revisar

int contOUFlow = 0, pulsosAnt = 0, pulsosAct = 0;
int contOUFlow2=0, pulsosAnt2=0, pulsosAct2 =0;
double velocidadPulsos = 0, velocidadRPM = 0, deltaT = 0.01;
double velocidadPulsos2 = 0, velocidadRPM2 = 0;
int num_spi=0;
int indexBuf=0;
uint8_t out_buffer[40] = {':','w','1','-','2','5',';',':','w','2','-','2','5',';'};


//variables para control
double error_vel_act = 0, error_vel_ant = 0;
double error_vel_act2 = 0, error_vel_ant2 = 0;
double velocidad_consigna=0;
double velocidad_consigna2=0;
double KP1 = 20, KI1 = 2, KD1 = 4;
double KP2 = 20, KI2 = 2, KD2 = 4;

double Ui_anterior=0, Ui_actual=0; // para control integral
double Up=0;
double Ud=0;
double Ui_anterior2=0, Ui_actual2=0; // para control integral
double Up2=0;
double Ud2=0;

int control=1;
int contador=0; //contador de tiempo para activar la señal de referencia;
int flagCont=0;
uint32_t duty_cycle_pid = 0;
uint32_t duty_cycle_pid2 = 0;
int stop1=1;
int stop2=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void interpreteComando(uint8_t *consigna_buffer){

	//uint32_t duty_cycle;
	double consigna;

	switch (consigna_buffer[0]) {
	case 'W':
	case 'w':
		switch (consigna_buffer[1]) {
		/*codigo ascii de '1' = 49*/
		case 49:
			if (consigna_buffer[2]) {
				/*codigo ascii de '+' = 43*/
				if (consigna_buffer[2] == 43) {
					stop1=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					if (consigna_buffer[3]) {
						consigna = atof((char *) &consigna_buffer[3]);
						if (consigna < 35) {
							velocidad_consigna = consigna;
						} else {
							velocidad_consigna = 35;
						}
//						printf("\r\n Velocidad consigna motor 1 : %s %5.3f \r\n","+", velocidad_consigna);
					}
					/*codigo ascii de '-' = 45*/
				} else if (consigna_buffer[2] == 45) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					stop1=0;
					if (consigna_buffer[3]) {
						consigna = atof((char *) &consigna_buffer[3]);
						if (consigna < 35) {
							velocidad_consigna = -consigna;
						} else {
							velocidad_consigna = -35;
						}
//						printf("\r\n Velocidad consigna motor 1 : %5.3f \r\n", velocidad_consigna);
					}
					/*codigo ascii de '0' = 48*/
				}else if(consigna_buffer[2] == 48){
					velocidad_consigna=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					stop1=1;
//					printf("\r\n Velocidad consigna motor 1 : %5.3f \r\n", velocidad_consigna);

				}
			}
			break;
			/*codigo ascii de '2' = 50*/
		case 50:
			if (consigna_buffer[2]) {
				if (	consigna_buffer[2] == 43) {
					stop2=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
					if (consigna_buffer[3]) {
						consigna = atof((char *) &consigna_buffer[3]);
						if (consigna < 35) {
							velocidad_consigna2 = consigna;
						} else {
							velocidad_consigna2 = 35;
						}
//						printf(
//								"\r\n Velocidad consigna motor 2 :  %s %5.3f \r\n","+", velocidad_consigna2);
					}
				} else if (consigna_buffer[2] == 45) {
					stop2=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
					if (consigna_buffer[3]) {
						consigna = atof((char *) &consigna_buffer[3]);
						if( consigna==0){
							velocidad_consigna2=0;
						}else if (consigna < 35) {
							velocidad_consigna2 = -consigna;
						} else {
							velocidad_consigna2 = -35;
						}
//						printf("\r\n Velocidad consigna motor 2 :  %5.3f \r\n",velocidad_consigna2);
					}
				}else if(consigna_buffer[2] == 48){
					velocidad_consigna2=0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
					stop2=0;
//					printf("\r\n Velocidad consigna motor 2 :  %5.3f \r\n",velocidad_consigna2);
				}
			}
			break;
		/*codigo ascii simbolo '?' = 63 */
		case 63:
			/*Transmit velocidad*/
			indexBuf=0;
		   // HAL_SPI_TransmitReceive_IT(&hspi2, &out_buffer[indexBuf], &byte, 1);
		    //HAL__IT(&hspi2, out_buffer, 14);
		    break;
		}
			break;

//		case 'v':
//		case 'V':
//			if(buffer[1]){
//				consigna = atof(&buffer[1]);
//				velocidad_consigna=(double) consigna;
//			break;
//			}
//
//		case 'c':
//		case 'C':
//			control=1;
//			break;
		case 'p':
		case 'P':
			switch (consigna_buffer[1]) {
					/*codigo ascii de '1' = 49*/
					case 49:
						consigna = atof((char *) &consigna_buffer[2]);
						KP1=(float) consigna;
						printf("\r\n");
						break;
					/*codigo ascii de '2' = 50*/
					case 50:
						consigna = atof((char *) &consigna_buffer[2]);
						KP2=(float) consigna;
						printf("\r\n");
						break;
					default:
					printf("\r\n Por favor indicar el valor correcto ( 1 ó 2 )\r\n");
					break;

			}
			break;
		case 'i':
		case 'I':
			switch (consigna_buffer[1]) {
					/*codigo ascii de '1' = 49*/
					case 49:
						consigna = atof((char *) &consigna_buffer[2]);
						KI1=(float) consigna;
						printf("\r\n");
						break;
					/*codigo ascii de '2' = 50*/
					case 50:
						consigna = atof((char *) &consigna_buffer[2]);
						KI2=(float) consigna;
						printf("\r\n");
						break;
					default:
						printf("\r\n Por favor indicar el valor correcto ( 1 ó 2 )\r\n");
						break;
			}
			break;
		case 'd':
		case 'D':
			switch (consigna_buffer[1]) {
					/*codigo ascii de '1' = 49*/
					case 49:
						consigna = atof((char *) &consigna_buffer[2]);
						KD1=(float) consigna;
						printf("\r\n");
						break;
					/*codigo ascii de '2' = 50*/
					case 50:
						consigna = atof((char *) &consigna_buffer[2]);
						KD2=(float) consigna;
						printf("\r\n");
						break;
					default:
						printf("\r\n Por favor indicar el valor correcto ( 1 ó 2 )\r\n");
						break;
			}
			break;

//		case 'r':
//		case 'R':
//			control=0;
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//			break;
	}

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){

		//pulsosAct = contOUFlow*(htim3.Instance->ARR) + __HAL_TIM_GET_COUNTER(&htim3);
		pulsosAct = (contOUFlow*65000) + __HAL_TIM_GET_COUNTER(&htim3);
		velocidadPulsos = (pulsosAct - pulsosAnt)/deltaT;
		velocidadRPM = velocidadPulsos/(168*44) * 60;
		pulsosAnt = pulsosAct;

		pulsosAct2 = (contOUFlow2*65000) + __HAL_TIM_GET_COUNTER(&htim4);
		velocidadPulsos2 = (pulsosAct2 - pulsosAnt2)/deltaT;
		velocidadRPM2 = velocidadPulsos2/(168*44) * 60;
		pulsosAnt2 = pulsosAct2;

		//para el control
		if(control==1){
			//calculo PID:
			/*
			 *----------------  control motor 1
			 */
			error_vel_act = velocidad_consigna-velocidadRPM;
//			if(error_vel_act<0 ){
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
//			    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//			}else if(error_vel_act>0){
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
//			   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
//			}
//			else if(error_vel_act==0){
//				Ui_anterior=0;
//			}
			if(stop1==1){
			  Ui_anterior=0;
			}
//			error_vel_acterror_vel_act);
			Up=KP1 * error_vel_act;
			Ui_actual=Ui_anterior + (KI1 * deltaT * error_vel_ant);
			Ud=(KD1/ deltaT) * (error_vel_act-error_vel_ant);
			if(velocidad_consigna>0){
				duty_cycle_pid = (uint32_t) fabs(duty_cycle_pid +(Up + Ui_actual + Ud));
			}else if(velocidad_consigna<0){
				duty_cycle_pid = (uint32_t) fabs(duty_cycle_pid -(Up + Ui_actual + Ud));
			}else{
				duty_cycle_pid = 0;
			}

			if(duty_cycle_pid > 14000){
				duty_cycle_pid=14000;
			}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle_pid);
			Ui_anterior=Ui_actual;
			error_vel_ant=error_vel_act;

			/*
			 * ------------------control motor 2
			 */
			error_vel_act2 = velocidad_consigna2-velocidadRPM2;

//			if(error_vel_act2<0 ){
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
//			}else if(error_vel_act2>0){
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
//			    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
//			}
//			else if(error_vel_act2==0){
//				Ui_anterior2=0;
//			}
//			error_vel_act2= fabs(error_vel_act2);
			if(stop2==1){
				Ui_anterior2=0;
			}
			Up2=KP2 * error_vel_act2;
			Ui_actual2=Ui_anterior2 + KI2 * deltaT * error_vel_ant2;
			Ud2=KD2/ deltaT * (error_vel_act2-error_vel_ant2);
			if(velocidad_consigna2 > 0){
				duty_cycle_pid2 = (uint32_t) fabs(duty_cycle_pid2 +(Up2 + Ui_actual2 + Ud2));
			}else if(velocidad_consigna2 < 0){
				duty_cycle_pid2 = (uint32_t) fabs(duty_cycle_pid2 -(Up2 + Ui_actual2 + Ud2));
			}else{
				duty_cycle_pid2 = 0;
			}
			if(duty_cycle_pid2 > 14000){
				duty_cycle_pid2 = 14000;
			}
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle_pid2);
			Ui_anterior2=Ui_actual2;
			error_vel_ant2=error_vel_act2;
		}
	}else if(htim->Instance == TIM3){
		if(__HAL_TIM_GET_COUNTER(&htim3) > (htim->Init.Period+1)/2){ //Underflow
			contOUFlow--;
//			printf("underflow motor 1 \r\n");
		}else { //Overflow
			contOUFlow++;
//			printf("overflow motor 1 \r\n");
		}
	}else if(htim->Instance == TIM4){
		if(__HAL_TIM_GET_COUNTER(&htim4) > (htim->Init.Period+1)/2){ //Underflow
			contOUFlow2--;
//			printf("underflow motor 2 \r\n");
		}else { //Overflow
			contOUFlow2++;
//			printf("overflow motor 2 \r\n");
		}
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi){

}

/**
  * @brief  The application entry point.
  * @retval int
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){
	num_spi=num_spi+10;
	  if (hspi->Instance == SPI2){
		  switch(in_buffer[0]){
		  	  case ':': //Comienzo de la trama

				  interpreteComando(&in_buffer[1]);//Consigna motor 1
				  interpreteComando(&in_buffer[8]);//Consigna motor 2
				  interpreteComando(&in_buffer[15]); //Consulta velocidades
				  break;
		  	  case '\r': //Retorno, fin de trama.
		  	  case ';':  //Fin de trama.
		  		  if(flagRx){
		  			flagRx = 0;
		  			buffer[indRx] = 0;
//		  			interpreteComando();
		  		  }
		  		  break;
		  	  case 8: //Retroceso es permitido de esta manera.
		  		  if(flagRx){
		  			  if(indRx > 0){
		  				indRx--;
		  			  }
		  		  }
		  		  break;
		  	  default: //Almacenamiento de la trama.
		  		  if(flagRx){
		  			  buffer[indRx] = byte;
		  			  if(indRx < MAX_BUFFER - 1){
		  				indRx++;
		  			  }

		  		  }
		  		break;
		  }
		  indexBuf++;
	    /* Receive one byte in interrupt mode */
		  //HAL_SPI_TransmitReceive_IT(&hspi2, &out_buffer[indexBuf], &byte, 1);
//		  HAL_SPI_Receive_IT(&hspi2, &byte, 1);
		  HAL_SPI_Receive_IT(&hspi2, in_buffer, 17);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	//HAL_UART_Receive_IT(&huart2, &byte, 1);
  	HAL_SPI_Receive_IT(&hspi2, in_buffer, 17);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);

	/*Se activan canales para generacion PWM- */
	/*canal 1 para motor 1 */
	/*canal  para motor 2*/
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

	/*Se activa en sentido positivo del motor  1*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

	/*Se activa el sentido positivo del motor 2*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	/*Se fuerza valor del contador del timer 3 y 4 para lecturas de encoders*/
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 11;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Period = 15000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 64999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA9 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
//	__disable_irq();
//	while (1) {
//	}
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
