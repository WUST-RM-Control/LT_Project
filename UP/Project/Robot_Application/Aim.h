#ifndef __Aim__
#define __Aim__

#include "main.h"


/*===| 发送给电脑的数据 |===*/
typedef __PACKED_STRUCT
{
    uint8_t cmd_ID;//命令码
    uint32_t time_stamp_pc;//收到的上一包的PC时间戳
    uint32_t time_stamp_receive_micro;//收到的上一包时STM32的时间戳 
    uint32_t time_stamp_send_micro;//发送此包时STM32的时间戳

    float yaw, pitch,roll;  //坐标系定义： +x:前，+y:左，+z：上，+yaw 左，+pitch 下。+roll 右倾，旋转顺序ZYX
    float yaw_vel, pitch_vel, roll_vel;
    float v_x, v_y, v_z;

    float bullet_speed;
    uint8_t detect_color; //0 r 1 b
    uint32_t bullet_count; //发出弹+1
    float operator_yaw_offset;
    float operator_pitch_offset;
} Aim_Send_StructTypedef;

/*===| 收到电脑的自瞄数据 |===*/
typedef __PACKED_STRUCT
{
    uint8_t cmd_ID; //命令码
    uint32_t time_stamp;

    uint8_t appear;

    float pitch, yaw;
    float target_yaw, target_pitch;
    

    float enable_yaw_diff, enable_pitch_diff;
    float v_yaw, v_pitch;
    float a_yaw, a_pitch;

    uint8_t detect_color;
} Aim_Receive_StructTypedef;
                                 
extern Aim_Receive_StructTypedef Aim_Receive_Struct;
extern Aim_Send_StructTypedef Aim_Send_Struct;
       

void Get_Average_Bullet_Speed(void);
void Aim_Get_Data_CallBack(uint8_t *Data);
uint8_t Aim_If_Allow_Shoot(void);
void Aim_Add_Operator_Offset(float Yaw, float Pitch);

#endif
