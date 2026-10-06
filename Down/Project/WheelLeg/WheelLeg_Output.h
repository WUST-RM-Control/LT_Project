#ifndef __WHEELLEG_OUTPUT_H__
#define __WHEELLEG_OUTPUT_H__

#include "main.h"




void WheelLeg_Output_Init(void);

/*
 * 目前只计算最终目标力矩。
 * 下一步再真正发送MIT。
 */
void WheelLeg_Output_UpdateTarget(void);

void WheelLeg_Output_SetLegEnable(uint8_t left_enable,uint8_t right_enable);

//更新使能输出
void WheelLeg_Output_UpdateEnable(void);

// target经过slew生成真正准备发送的command
void WheelLeg_Output_UpdateCommand(float dt);

// 将command真正发送给达妙
void WheelLeg_Output_Send(void);


#endif