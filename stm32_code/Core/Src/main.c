/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "motorZLG.h"
#include "stdio.h"
#include "lwip/udp.h"
#include "udp_server.h"
#include "ZLAC8015_CAN.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISCHARGE 0
#define CHARGE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//motor
ZLAC8015_typedef motorDriver[3], single;
uint16_t RotEnc[2];
int32_t ZL8051_velSP[3], singleVel;
int32_t ZL8051_ENC[3];

uint16_t BuffENC[3];

int16_t velocity[3];
int16_t testVel[3];
int16_t vel;
float WheelSpeed_SP[3];


//udp
uint32_t PC_TimeConnection, timer_od;

char pc_recv[64];
char pc_send[64];

struct udp_pcb *upcb;
uint32_t epochKirim;


int statusUDP;

//variabel pc to stm
uint32_t PC_Epoch;
uint8_t PC_Mode, prev_PC_Mode;
int16_t PC_Speed_X, PC_Speed_Y, PC_Speed_Q;


//batrai
uint32_t timerS;

uint8_t BMS_Trans[21] = {0x4E, 0x57, 0x00, 0x13 ,0x00 ,0x00, 0x00 ,0x00 ,0x06 ,0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x68 ,0x00 ,0x00 ,0x01 ,0x29};
uint8_t BMS_Recv[300];
uint16_t BMS_Len;

float BMS_Voltage,BMS_SOC,BMS_Current;
float BMS_Cell[8];

uint8_t ChargingStatus;

uint32_t x;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void raisa_3_wheel_kine(int16_t Vx, int16_t Vy, int16_t Vz);

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_LWIP_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ZL_InitCANBus();
  ZLAC8015_init(&motorDriver[0], &hcan2, 1, 0);
  ZLAC8015_init(&motorDriver[1], &hcan2, 1, 1);
  ZLAC8015_init(&motorDriver[2], &hcan2, 2, 0); //liat id nya

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);



  HAL_Delay(1000);

  udp_server_init((uint8_t*)pc_recv, (uint8_t*)pc_send, 9798, 64);

  for(int i=0; i<3; i++)
  {
	  ZLAC8015_Enable(&motorDriver[i]);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  v = z;
	  PC_TimeConnection = HAL_GetTick() - UDP_Recv_TimeStamp;
	  if(PC_TimeConnection < 600)
	  {
		  statusUDP = 1;
	  }
	  else
	  {
		  statusUDP = 0;
	  }
		if(prev_PC_Mode != 0 && PC_Mode == 0)
		{
			  for(int i=0; i<3; i++)
			  {
				  ZLAC8015_Disable(&motorDriver[i]);
			  }
		}
		else if(prev_PC_Mode == 0 && PC_Mode != 0)
		{
			  for(int i=0; i<3; i++)
			  {
				  ZLAC8015_Enable(&motorDriver[i]);
			  }
		}
		prev_PC_Mode = PC_Mode;

	  if(HAL_GetTick() - timer_od > 2)
	  {
		  timer_od = HAL_GetTick();
		  for(int i=0;i<3;i++)
		  {
			  ZLAC8015_SetSpeed(&motorDriver[i], motorDriver[i].Axis, ZL8051_velSP[i]);
			  HAL_Delay(1);
			  ZLAC8015_Request_Enc(&motorDriver[i], motorDriver[i].Axis);
			  HAL_Delay(1);
		  }
	  }

	  if (HAL_GetTick() - timerS > 100)
	  {
		  timerS = HAL_GetTick();

		  HAL_UART_Transmit(&huart3, BMS_Trans, sizeof(BMS_Trans), 500);
		  HAL_UARTEx_ReceiveToIdle(&huart3,BMS_Recv , sizeof(BMS_Recv), &BMS_Len, 500);

		  if(BMS_Len==267 && BMS_Recv[0] == 0x4E && BMS_Recv[1] ==0x57 && BMS_Recv[2] == 0x01)
		  {

			  BMS_Voltage = (BMS_Recv[47]<<8 | BMS_Recv[48]) * 0.01;
			  BMS_SOC = BMS_Recv[53];
			  BMS_Current = ((BMS_Recv[51] + BMS_Recv[50]*256) & 0xfff) * -0.01;

			  BMS_Cell[0] = (BMS_Recv[14]<<8 | BMS_Recv[15]) * 0.001;
			  BMS_Cell[1] = (BMS_Recv[17]<<8 | BMS_Recv[18]) * 0.001;
			  BMS_Cell[2] = (BMS_Recv[20]<<8 | BMS_Recv[21]) * 0.001;
			  BMS_Cell[3] = (BMS_Recv[23]<<8 | BMS_Recv[24]) * 0.001;
			  BMS_Cell[4] = (BMS_Recv[26]<<8 | BMS_Recv[27]) * 0.001;
			  BMS_Cell[5] = (BMS_Recv[29]<<8 | BMS_Recv[30]) * 0.001;
			  BMS_Cell[6] = (BMS_Recv[32]<<8 | BMS_Recv[33]) * 0.001;
			  BMS_Cell[7] = (BMS_Recv[35]<<8 | BMS_Recv[36]) * 0.001;

			  if(BMS_Current < -5.0 && BMS_Current >=-20.0)
				  ChargingStatus = CHARGE;
			  else if(BMS_Current <= 0 && BMS_Current > -5.0)
				  ChargingStatus = DISCHARGE;
			  else
				  ChargingStatus = 0;
		  }

	  }
//
//
//
	  MX_LWIP_Process();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg, CAN_RxData);

	if(RxMsg.StdId == 0x581)
	{
		if(CAN_RxData[0] == 0x43 && CAN_RxData[1] == 0x64 &&
		   CAN_RxData[2] == 0x60 && CAN_RxData[3] == 0x01)
		{
			memcpy(&ZL8051_ENC[0],CAN_RxData+4,4);
		}
		else if(CAN_RxData[0] == 0x43 && CAN_RxData[1] == 0x64 &&
				CAN_RxData[2] == 0x60 && CAN_RxData[3] == 0x02)
		{
			memcpy(&ZL8051_ENC[1],CAN_RxData+4,4);
		}

	}
	else if(RxMsg.StdId == 0x582)
	{
		if(CAN_RxData[0] == 0x43 && CAN_RxData[1] == 0x64 &&
		   CAN_RxData[2] == 0x60 && CAN_RxData[3] == 0x00)
		{
			memcpy(&ZL8051_ENC[2],CAN_RxData+4,4);
		}

	}



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM6) // Timer Routine 1Khz
  {

		RotEnc[0] = TIM3->CNT;
		RotEnc[1] = TIM1->CNT;

//		x++;

//
	  	BuffENC[0] = ZL8051_ENC[0];
	  	BuffENC[1] = ZL8051_ENC[1];
	  	BuffENC[2] = ZL8051_ENC[2];

	  	//pc_send[0]-[3] is for epoch
		memcpy(pc_send+0, &epoch_kirim, 4);
		memcpy(pc_send+4, &BuffENC[0], 2); //motor 0 depan kiri
		memcpy(pc_send+6, &BuffENC[1] ,2); //motor 1 depan kanan
		memcpy(pc_send+8, &BuffENC[2] ,2); //motor 2
		memset(pc_send+10, 0, 2);
		memset(pc_send+12, 0, 4);
		memcpy(pc_send+16,&BMS_Voltage,4);
		memcpy(pc_send+20,&BMS_SOC,4);
		memcpy(pc_send+24, &ChargingStatus, 1);
		memset(pc_send+25, 0, 2);
		memcpy(pc_send+27,&RotEnc[1],2); //kanan
		memcpy(pc_send+29,&RotEnc[0],2); // kiri



  }
  else if (htim->Instance == TIM7) // Timer Routine 50Hz
  {

	  	// Variabel Raisa Dari PC Ke STM
		memcpy(&PC_Epoch, pc_recv, 4);
		memcpy(&PC_Mode, pc_recv + 4, 1);
		memcpy(&PC_Speed_X, pc_recv + 5, 2);
		memcpy(&PC_Speed_Y, pc_recv + 7, 2);
		memcpy(&PC_Speed_Q, pc_recv + 9, 2);
//
		if(PC_Speed_X > 1000)
			PC_Speed_X = 1000;
		else if(PC_Speed_X <-1000)
			PC_Speed_X = -1000;


		if(PC_Speed_Y > 1000)
			PC_Speed_Y = 1000;
		else if(PC_Speed_Y <-1000)
			PC_Speed_Y = -1000;


		if(PC_Speed_Q > 1000)
			PC_Speed_Q = 1000;
		else if(PC_Speed_Q <-1000)
			PC_Speed_Q = -1000;


		//IF UDP Connected
		if(statusUDP == 1)
		{
			velocity[0] = PC_Speed_X;
			velocity[1] = PC_Speed_Y;
			velocity[2] = PC_Speed_Q;
		}
		else
		{
			velocity[0] = 0;
			velocity[1] = 0;
			velocity[2] = 0;
			PC_Mode = 0;
		}


		if(PC_Mode==1)
		{
			raisa_3_wheel_kine(velocity[0], velocity[1], velocity[2]);
		}
		else if(PC_Mode==2)
		{
			ZL8051_velSP[0] = PC_Speed_X;
			ZL8051_velSP[1] = PC_Speed_Y;
			ZL8051_velSP[2] = PC_Speed_Q;
		}
		else
		{
			ZL8051_velSP[0] = 0;
			ZL8051_velSP[1] = 0;
			ZL8051_velSP[2] = 0;
		}
//		raisa_3_wheel_kine(testVel[0], testVel[1], testVel[2]);


  }
}


void raisa_3_wheel_kine(int16_t Vx, int16_t Vy, int16_t Vz){
    float degree[3] = {120,60,270};
    float R_omni = 1, dist = 1;

    WheelSpeed_SP[0] = (1/R_omni)*(cosf(degree[0]*0.01745)*(Vx) + sinf(degree[0]*0.01745)*(Vy) + dist*(Vz));
    WheelSpeed_SP[1] = (1/R_omni)*(cosf(degree[1]*0.01745)*(Vx) + sinf(degree[1]*0.01745)*(Vy) + dist*(Vz));
    WheelSpeed_SP[2] = (1/R_omni)*(cosf(degree[2]*0.01745)*(Vx) + sinf(degree[2]*0.01745)*(Vy) + dist*(Vz));

    for(int i=0; i<3; i++)
    {
    	ZL8051_velSP[i] = WheelSpeed_SP[i];
    }

}

//tidak jadi rescale

//int16_t ReScale_enc(int32_t rawData)
//{
//	int div = rawData / 4095;
//	int16_t result;
//
//	if(rawData > 4095)
//	{
//		result = rawData - (div * 4095);
//	}
//	else if( rawData < 0)
//	{
//		result = 4096 + (rawData - (div * 4095));
//	}
//	else
//	{
//		result = rawData;
//	}
//
//	return result;
//}



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
