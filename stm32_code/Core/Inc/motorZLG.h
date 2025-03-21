/*
 * motorZLG.h
 *
 *  Created on: Aug 22, 2022
 *      Author: ismarintan
 */

#ifndef INC_MOTORZLG_H_
#define INC_MOTORZLG_H_

#include "main.h"


typedef struct{

		uint16_t MOTOR_ID;
		int32_t MotorSpeed,MotorSP_Speed;
		float voltage;
		uint16_t position;


}ZL_Motor_TypeDef;


extern CAN_TxHeaderTypeDef TxMsg;
extern CAN_RxHeaderTypeDef RxMsg;

extern CAN_FilterTypeDef CanFilterConfig;

extern uint32_t CAN_TxMailbox;
extern uint8_t CAN_TxData[8],CAN_RxData[8];



void ZL_InitCANBus(void);
void MotorZL_Enable(ZL_Motor_TypeDef *pMotor);
void MotorZL_SpeedMode(ZL_Motor_TypeDef *pMotor);
void MotorZL_Disable(ZL_Motor_TypeDef *pMotor);
void MotorZL_ControlHandler(ZL_Motor_TypeDef *pMotor);

#endif /* INC_MOTORZLG_H_ */
