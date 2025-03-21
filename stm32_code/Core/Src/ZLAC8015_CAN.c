/*
 * ZLAC8015.c
 *
 *  Created on: Jul 17, 2023
 *      Author: ismarintan
 */


#include "ZLAC8015_CAN.h"
#include "stdlib.h"
#include "string.h"


CAN_TxHeaderTypeDef ZL_TxMsg;
CAN_RxHeaderTypeDef ZL_RxMsg;
uint32_t ZL_TxMailbox;

uint8_t ZL_CAN_TxData[8];


void ZLAC8015_init(ZLAC8015_typedef *pDriver, CAN_HandleTypeDef *pCAN, uint32_t Driver_ID, uint32_t Axis)
{

	pDriver->pCAN = pCAN;
	pDriver->Driver_ID = 0x600 + Driver_ID;
	pDriver->Axis = Axis;

}


void ZLAC8015_Set_Async(ZLAC8015_typedef *pDriver)
{
	CAN_TxHeaderTypeDef TxMsg;
	uint8_t CAN_TxData[8];
	uint32_t TxMailbox;

	TxMsg.StdId = 0x600 + pDriver->Driver_ID;
	TxMsg.DLC = 8;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;

	CAN_TxData[0] = 0x2F;
	CAN_TxData[1] = 0x0F;
	CAN_TxData[2] = 0x20;
	CAN_TxData[3] = 0x00;

	CAN_TxData[4] = 0x00;
	CAN_TxData[5] = 0x00;
	CAN_TxData[6] = 0x00;
	CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN, &TxMsg, CAN_TxData, &TxMailbox);

}

void ZLAC8015_Set_VelocityMode(ZLAC8015_typedef *pDriver)
{
	ZL_TxMsg.DLC = 8;
	ZL_TxMsg.ExtId = 0;
	ZL_TxMsg.IDE = CAN_ID_STD;
	ZL_TxMsg.RTR = CAN_RTR_DATA;
	ZL_TxMsg.StdId = pDriver->Driver_ID;

	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x2B;
	ZL_CAN_TxData[1] = 0x40;
	ZL_CAN_TxData[2] = 0x60;
	ZL_CAN_TxData[3] = 0x00;

	ZL_CAN_TxData[4] = 0x00;
	ZL_CAN_TxData[5] = 0x00;
	ZL_CAN_TxData[6] = 0x00;
	ZL_CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);


}

void ZLAC8015_Enable(ZLAC8015_typedef *pDriver)
{
	ZL_TxMsg.DLC = 8;
	ZL_TxMsg.ExtId = 0;
	ZL_TxMsg.IDE = CAN_ID_STD;
	ZL_TxMsg.RTR = CAN_RTR_DATA;
	ZL_TxMsg.StdId = pDriver->Driver_ID;

	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x2B;
	ZL_CAN_TxData[1] = 0x40;
	ZL_CAN_TxData[2] = 0x60;
	ZL_CAN_TxData[3] = 0x00;

	ZL_CAN_TxData[4] = 0x00;
	ZL_CAN_TxData[5] = 0x00;
	ZL_CAN_TxData[6] = 0x00;
	ZL_CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);
	HAL_Delay(2);

	// ------------------------------------------------------------------
	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x2B;
	ZL_CAN_TxData[1] = 0x40;
	ZL_CAN_TxData[2] = 0x60;
	ZL_CAN_TxData[3] = 0x00;

	ZL_CAN_TxData[4] = 0x06;
	ZL_CAN_TxData[5] = 0x00;
	ZL_CAN_TxData[6] = 0x00;
	ZL_CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);
	HAL_Delay(2);
	// ------------------------------------------------------------------
	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x2B;
	ZL_CAN_TxData[1] = 0x40;
	ZL_CAN_TxData[2] = 0x60;
	ZL_CAN_TxData[3] = 0x00;

	ZL_CAN_TxData[4] = 0x07;
	ZL_CAN_TxData[5] = 0x00;
	ZL_CAN_TxData[6] = 0x00;
	ZL_CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);
	HAL_Delay(2);
	// ------------------------------------------------------------------
	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x2B;
	ZL_CAN_TxData[1] = 0x40;
	ZL_CAN_TxData[2] = 0x60;
	ZL_CAN_TxData[3] = 0x00;

	ZL_CAN_TxData[4] = 0x0F;
	ZL_CAN_TxData[5] = 0x00;
	ZL_CAN_TxData[6] = 0x00;
	ZL_CAN_TxData[7] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);
	HAL_Delay(2);


}

void ZLAC8015_Disable(ZLAC8015_typedef *pDriver)
{
    ZL_TxMsg.DLC = 8;
    ZL_TxMsg.ExtId = 0;
    ZL_TxMsg.IDE = CAN_ID_STD;
    ZL_TxMsg.RTR = CAN_RTR_DATA;
    ZL_TxMsg.StdId = pDriver->Driver_ID;

    memset(ZL_CAN_TxData, 0, sizeof(ZL_CAN_TxData));

    ZL_CAN_TxData[0] = 0x2B;
    ZL_CAN_TxData[1] = 0x40;
    ZL_CAN_TxData[2] = 0x60;
    ZL_CAN_TxData[3] = 0x00;

    ZL_CAN_TxData[4] = 0x00; // Command to disable the driver
    ZL_CAN_TxData[5] = 0x00;
    ZL_CAN_TxData[6] = 0x00;
    ZL_CAN_TxData[7] = 0x00;

    HAL_CAN_AddTxMessage(pDriver->pCAN, &ZL_TxMsg, ZL_CAN_TxData, &ZL_TxMailbox);
    HAL_Delay(2);
}


void ZLAC8015_SetSpeed(ZLAC8015_typedef *pDriver, uint32_t Axis, int32_t Speed)
{

	ZL_TxMsg.DLC = 8;
	ZL_TxMsg.ExtId = 0;
	ZL_TxMsg.IDE = CAN_ID_STD;
	ZL_TxMsg.RTR = CAN_RTR_DATA;
	ZL_TxMsg.StdId = pDriver->Driver_ID;

	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x23;
	ZL_CAN_TxData[1] = 0xFF;
	ZL_CAN_TxData[2] = 0x60;
	if(ZL_TxMsg.StdId == 1538)
	{
		ZL_CAN_TxData[3] = 0x00;
	}
	else
	{
		ZL_CAN_TxData[3] = 0x01 + Axis;
	}


	memcpy(ZL_CAN_TxData + 4,&Speed,sizeof(Speed));
	HAL_CAN_AddTxMessage(pDriver->pCAN,&ZL_TxMsg,ZL_CAN_TxData, &ZL_TxMailbox);

}



void ZLAC8015_Request_Enc(ZLAC8015_typedef *pDriver, uint32_t Axis)
{

	// Object Idx 6064h Sub Idx 1 Left 2 Right

	ZL_TxMsg.StdId = pDriver->Driver_ID;
	ZL_TxMsg.DLC = 8;
	ZL_TxMsg.IDE = CAN_ID_STD;
	ZL_TxMsg.RTR = CAN_RTR_DATA;

	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x43;
	ZL_CAN_TxData[1] = 0x64;
	ZL_CAN_TxData[2] = 0x60;
	if(ZL_TxMsg.StdId == 1538)
	{
		ZL_CAN_TxData[3] = 0x00;
	}
	else
	{
		ZL_CAN_TxData[3] = 0x01 + Axis;
	}

	HAL_CAN_AddTxMessage(pDriver->pCAN, &ZL_TxMsg, ZL_CAN_TxData, &ZL_TxMailbox);

}
void ZLAC8015_Request_VBus(ZLAC8015_typedef *pDriver)
{

	// Object Idx 2035h Sub Idx 0

	ZL_TxMsg.StdId = pDriver->Driver_ID;
	ZL_TxMsg.DLC = 8;
	ZL_TxMsg.IDE = CAN_ID_STD;
	ZL_TxMsg.RTR = CAN_RTR_DATA;

	memset(ZL_CAN_TxData,0,sizeof(ZL_CAN_TxData));

	ZL_CAN_TxData[0] = 0x4B;
	ZL_CAN_TxData[1] = 0x35;
	ZL_CAN_TxData[2] = 0x20;
	ZL_CAN_TxData[3] = 0x00;

	HAL_CAN_AddTxMessage(pDriver->pCAN, &ZL_TxMsg, ZL_CAN_TxData, &ZL_TxMailbox);

}
void ZLAC8015_RxHandler(ZLAC8015_typedef *pDriver, CAN_RxHeaderTypeDef RxMsg, uint8_t CAN_RxData[8])
{


}
