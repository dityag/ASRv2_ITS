/*
 * ZLAC8015_CAN.h
 *
 *  Created on: Jul 17, 2023
 *      Author: ismarintan
 */

#ifndef INC_ZLAC8015_CAN_H_
#define INC_ZLAC8015_CAN_H_

#include "main.h"
#include "can.h"


typedef struct{

	uint32_t Driver_ID;
	uint32_t Axis;
	int32_t Encoder;
	float Vbus;

	CAN_HandleTypeDef *pCAN;

}ZLAC8015_typedef;


void ZLAC8015_init(ZLAC8015_typedef *pDriver, CAN_HandleTypeDef *pCAN, uint32_t Driver_ID, uint32_t Axis);
//void ZLAC8015_SetSpeed_1CH(ZLAC8015_typedef *pDriver, uint32_t Axis, int32_t Speed);
void ZLAC8015_SetSpeed(ZLAC8015_typedef *pDriver, uint32_t Axis, int32_t Speed);
void ZLAC8015_Request_Enc(ZLAC8015_typedef *pDriver, uint32_t Axis);
void ZLAC8015_Request_VBus(ZLAC8015_typedef *pDriver);
void ZLAC8015_RxHandler(ZLAC8015_typedef *pDriver, CAN_RxHeaderTypeDef RxMsg, uint8_t CAN_RxData[8]);

void ZLAC8015_Set_Async(ZLAC8015_typedef *pDriver);
void ZLAC8015_Enable(ZLAC8015_typedef *pDriver);
void ZLAC8015_Disable(ZLAC8015_typedef *pDriver);
void ZLAC8015_Set_VelocityMode(ZLAC8015_typedef *pDriver);





#endif /* INC_ZLAC8015_CAN_H_ */
