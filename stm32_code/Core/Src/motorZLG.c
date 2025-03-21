/*
 * motorZLG.c
 *
 *  Created on: Aug 22, 2022
 *      Author: ismarintan
 */


#include "motorZLG.h"
#include "can.h"

CAN_TxHeaderTypeDef TxMsg;
CAN_RxHeaderTypeDef RxMsg;

CAN_FilterTypeDef CanFilterConfig;

uint32_t CAN_TxMailbox;
uint8_t CAN_TxData[8],CAN_RxData[8];



void ZL_InitCANBus(void)
{

	CanFilterConfig.FilterBank = 0;
	CanFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CanFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	CanFilterConfig.FilterIdHigh = 0x0000;
	CanFilterConfig.FilterIdLow = 0x0000;
	CanFilterConfig.FilterMaskIdHigh = 0X0000;
	CanFilterConfig.FilterMaskIdLow = 0X0000;

	CanFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	CanFilterConfig.FilterActivation = ENABLE;
	CanFilterConfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan2, &CanFilterConfig);

	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	TxMsg.DLC = 8;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;

	HAL_CAN_Start(&hcan2);

}


void MotorZL_Enable(ZL_Motor_TypeDef *pMotor)
{


	TxMsg.StdId = pMotor->MOTOR_ID;
	CAN_TxData[0] = 0x00;
	CAN_TxData[1] = 0xDA;
	CAN_TxData[2] = 0x00;
	CAN_TxData[3] = 0x10;
	CAN_TxData[7] = 0x00;
	CAN_TxData[6] = 0x00;
	CAN_TxData[5] = 0x00;
	CAN_TxData[4] = 0x1F;
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, CAN_TxData, &CAN_TxMailbox);


}

void MotorZL_SpeedMode(ZL_Motor_TypeDef *pMotor)
{


	TxMsg.StdId = pMotor->MOTOR_ID;
	CAN_TxData[0] = 0x00;
	CAN_TxData[1] = 0xDA;
	CAN_TxData[2] = 0x00;
	CAN_TxData[3] = 0x2D;
	CAN_TxData[7] = 0x00;
	CAN_TxData[6] = 0x00;
	CAN_TxData[5] = 0x1F;
	CAN_TxData[4] = 0x40;
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, CAN_TxData, &CAN_TxMailbox);


}


void MotorZL_Disable(ZL_Motor_TypeDef *pMotor)
{
	TxMsg.StdId = pMotor->MOTOR_ID;
	CAN_TxData[0] = 0x00;
	CAN_TxData[1] = 0xDA;
	CAN_TxData[2] = 0x00;
	CAN_TxData[3] = 0x2D;
	CAN_TxData[7] = 0x00;
	CAN_TxData[6] = 0x00;
	CAN_TxData[5] = 0x00;
	CAN_TxData[4] = 0x00;
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, CAN_TxData, &CAN_TxMailbox);
}




void MotorZL_ControlHandler(ZL_Motor_TypeDef *pMotor)
{

	TxMsg.StdId = pMotor->MOTOR_ID;
	CAN_TxData[0] = 0x00;
	CAN_TxData[1] = 0xDA;
	CAN_TxData[2] = 0x00;
	CAN_TxData[3] = 0x11;
	CAN_TxData[7] = (uint8_t) (pMotor->MotorSP_Speed & 0xFF);
	CAN_TxData[6] = (uint8_t) ((pMotor->MotorSP_Speed >> 8) & 0xFF);
	CAN_TxData[5] = (uint8_t) ((pMotor->MotorSP_Speed >> 16) & 0xFF);
	CAN_TxData[4] = (uint8_t) ((pMotor->MotorSP_Speed >> 24) & 0xFF);
	HAL_CAN_AddTxMessage(&hcan2, &TxMsg, CAN_TxData, &CAN_TxMailbox);


}


//void MotorZL_ControlHandler(ZL_Motor_TypeDef *pMotor)
//{
//	if(RxMsg.StdId == pMotor->MOTOR_ID)
//	{
//		//Voltage
//		if(CAN_RxData[0] == 0x00 && CAN_RxData[1] == 0xDB && CAN_RxData[2] == 0x00 && CAN_RxData[3] == 0xE1)
//		{
//			pMotor->voltage = (float)((CAN_RxData[6] << 8) | CAN_RxData[7]);
//		}
//	}
//}





