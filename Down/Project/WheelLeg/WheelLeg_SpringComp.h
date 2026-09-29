#ifndef __WHEELLEG_SPRINGCOMP_H__
#define __WHEELLEG_SPRINGCOMP_H__

#include "main.h"
#define WHEELLEG_SPRING_TABLE_SIZE      1024

typedef struct
{
    //当前腿长
    float length_m;

    // 当前查到的静态机械补偿
    float force_n;

    // 当前索引
    uint16_t index;

    uint8_t valid;

} WheelLeg_SpringComp_Leg;

typedef struct
{
    WheelLeg_SpringComp_Leg left;
    WheelLeg_SpringComp_Leg right;

} WheelLeg_SpringComp;

/* 初始化 */
void WheelLeg_SpringComp_Init(void);

//更新补偿
void WheelLeg_SpringComp_Update(float left_length_m,float right_length_m);

extern WheelLeg_SpringComp wheelLeg_springComp;

#endif