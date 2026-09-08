//======================================================================
//                      Shoot[发射机构控制]
//      根据RoboControl中的Shoot_State驱动相应电机实现单发，连发等功能
//======================================================================

#include "Shoot.h"
#include "RoboControl.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Referee_Unpack.h"
#include "Aim.h"
/*===| 发送控制数据结构体 |===*/
Shoot_Control_StructTypeDef Shoot_Control_Struct;

float Shoot_Fric_First_Left_Speed    = 20.5f;
float Shoot_Fric_First_Right_Speed   = 20.5f;

#define Shoot_Frequency_Max 23

float Vir_Heat;
float Shoot_Dt;
float Heat_Limit_surplus=40;
float Heat_Limit_linear=30;

void Shoot_Task(void *argument)
{;
    /*===| 发射机构PID参数初始化 |===*/
    PID_Init(&Shoot_Control_Struct.Fric_First_Left_Speed_PID_Struct,      Shoot_Fric_Speed_PID_Kp,     Shoot_Fric_Speed_PID_Ki,    Shoot_Fric_Speed_PID_Kd,    0, Shoot_Fric_Speed_PID_I_Output_Max,      Shoot_Fric_Speed_PID_Output_Max);
    PID_Init(&Shoot_Control_Struct.Fric_First_Right_Speed_PID_Struct,     Shoot_Fric_Speed_PID_Kp,     Shoot_Fric_Speed_PID_Ki,    Shoot_Fric_Speed_PID_Kd,    0, Shoot_Fric_Speed_PID_I_Output_Max,      Shoot_Fric_Speed_PID_Output_Max);  
    
    PID_Init(&Shoot_Control_Struct.Trigger_Angle_PID_Struct,        10,  0, 200, 0, 0,   500);
    PID_Init(&Shoot_Control_Struct.Trigger_Single_Speed_PID_Struct,        300,  5, 0, 30, 16000,   16000);
    
    Shoot_Control_Struct.Shoot_Frequency = Shoot_Frequency_Max;
    for(;;)
    {     
        Shoot_Dt = DWT_GetDeltaT(&Shoot_Control_Struct.DWT_Count);
        Shoot_Control_Struct.Cool_Down -= Shoot_Dt;
        Limit_float(&Shoot_Control_Struct.Cool_Down, 100, 0);
        
        
        /*===| 得到发射状态 |===*/
        Shoot_Control_Struct.Shoot_State = RoboControl_Struct.Shoot_State;
        
        /*===| 热量限制 |===*/
        Heat_Limit();

        /*===| 根据发射状态控制摩擦轮和拨弹轮 |===*/
            /*===| 发射机构关闭 |===*/
            if      (Shoot_Control_Struct.Shoot_State == Shoot_State_Off)
            {
                /*===| 摩擦轮关闭，拨弹轮速度0 |===*/
                Shoot_Control_Struct.Fric_First_Left_Target_Speed   = 0;
                Shoot_Control_Struct.Fric_First_Right_Target_Speed  = 0;
                
                /*===| 重置拨弹轮角度环，防止单发位置错误 |===*/
                Motor.Trigger.Round %= (int)(Trigger_Reduction);
                Shoot_Control_Struct.Trigger_Target_Angle = Motor.Trigger.Total_Angle / Trigger_Reduction;
            }
            /*===| 发射机构开启，准备单发或者连发 |===*/
            else if (Shoot_Control_Struct.Shoot_State == Shoot_State_Ready)
            {
                /*===| 摩擦轮开启，拨弹轮角度环控制 |===*/
                Shoot_Control_Struct.Fric_First_Left_Target_Speed   = Shoot_Fric_First_Left_Speed  ;
                Shoot_Control_Struct.Fric_First_Right_Target_Speed  = Shoot_Fric_First_Right_Speed ;
            }
            /*===| 发射机构开启，摩擦轮拨弹轮倒转 |===*/
            else if (Shoot_Control_Struct.Shoot_State == Shoot_State_Unload)
            {
                /*===| 摩擦轮开启，拨弹轮角度环控制 |===*/
                Shoot_Control_Struct.Fric_First_Left_Target_Speed   = -Shoot_Fric_First_Left_Speed  ;
                Shoot_Control_Struct.Fric_First_Right_Target_Speed  = -Shoot_Fric_First_Right_Speed ;
            }
            /*===| 连发模式 |===*/
            else if (Shoot_Control_Struct.Shoot_State == Shoot_State_Continue)
            {
                /*===| 摩擦轮开启，拨弹轮速度环控制 |===*/
                Shoot_Control_Struct.Fric_First_Left_Target_Speed   = Shoot_Fric_First_Left_Speed  ;
                Shoot_Control_Struct.Fric_First_Right_Target_Speed  = Shoot_Fric_First_Right_Speed ;
                
                Shoot_Single(1);
            }
            
            /*===| 摩擦轮斜坡加减速，防止瞬时功率过大导致Shoot断电 |===*/
            Acc_and_Dec(Shoot_Control_Struct.Fric_First_Left_Target_Speed, &Shoot_Control_Struct.Fric_First_Left_Set_Speed, Fric_Acc , Fric_Acc, Shoot_Dt);
            Acc_and_Dec(Shoot_Control_Struct.Fric_First_Right_Target_Speed, &Shoot_Control_Struct.Fric_First_Right_Set_Speed, Fric_Acc, Fric_Acc, Shoot_Dt);
            
            
            Limit_float(&Shoot_Control_Struct.Fric_First_Left_Set_Speed,    Shoot_Control_Struct.Fric_First_Left_Target_Speed,      -Shoot_Control_Struct.Fric_First_Left_Target_Speed   );
            Limit_float(&Shoot_Control_Struct.Fric_First_Right_Set_Speed,   Shoot_Control_Struct.Fric_First_Right_Target_Speed,     -Shoot_Control_Struct.Fric_First_Right_Target_Speed  );

            /*===| PID计算 |===*/
            /*===| 摩擦轮 |===*/
            PID_Position_Calculate(&Shoot_Control_Struct.Fric_First_Left_Speed_PID_Struct,    Shoot_Control_Struct.Fric_First_Left_Set_Speed,     Motor.Fric1.Speed_RPM * Fric_RPM_to_m_s);
            PID_Position_Calculate(&Shoot_Control_Struct.Fric_First_Right_Speed_PID_Struct,  -Shoot_Control_Struct.Fric_First_Right_Set_Speed,    Motor.Fric2.Speed_RPM * Fric_RPM_to_m_s);

            /*===| 拨弹轮 |===*/
                /*===| 发送机构关闭无PID输出 |===*/
                if(Shoot_Control_Struct.Shoot_State == Shoot_State_Off)
                {
                    Shoot_Control_Struct.Trigger_Send_Current = 0;
                }
                /*===| 角度环控制 |===*/
                else
                {
                    PID_Position_Calculate(&Shoot_Control_Struct.Trigger_Angle_PID_Struct,   Shoot_Control_Struct.Trigger_Target_Angle ,            Motor.Trigger.Total_Angle / Trigger_Reduction);
                    PID_Position_Calculate(&Shoot_Control_Struct.Trigger_Single_Speed_PID_Struct,   Shoot_Control_Struct.Trigger_Angle_PID_Struct.Output, Motor.Trigger.Speed_RPM / Trigger_Reduction);
                    Shoot_Control_Struct.Trigger_Send_Current = Shoot_Control_Struct.Trigger_Single_Speed_PID_Struct.Output;
                }   
                
            /*===| 拨弹轮堵弹检测与处理 |===*/
            Trigger_Stuck_Handle();
        
            /*===| 发送电流参数 |===*/
            if(RoboControl_Struct.Robo_Enable) 
            {
                Motor_DJI_SendCurrent(&Shoot_Fric_First_CAN, Shoot_Fric_First_Send_CAN_ID,  
                                    Shoot_Control_Struct.Fric_First_Right_Speed_PID_Struct.Output,
                                    Shoot_Control_Struct.Fric_First_Left_Speed_PID_Struct.Output,
                                    0,0 );
                Motor_DJI_SendCurrent(&Shoot_Trigger_CAN, Shoot_Trigger_Send_CAN_ID, 0,0,0,Shoot_Control_Struct.Trigger_Send_Current); 
            }
            else
            {
                Motor_DJI_SendCurrent(&Shoot_Fric_First_CAN, Shoot_Fric_First_Send_CAN_ID,  0, 0, 0, 0);
                Motor_DJI_SendCurrent(&Shoot_Trigger_CAN, Shoot_Trigger_Send_CAN_ID, 0, 0, 0, 0);
            }
                                                                  
        osDelay(1);
    }
}


/**
 * @brief 单发[调用一次射击一发]
 */
void Shoot_Single(float Num)
{
    if(
        (fabsf(Shoot_Control_Struct.Trigger_Target_Angle - Motor.Trigger.Total_Angle / Trigger_Reduction) <= 2 * 360.0f / Trigger_Bullet_Num)&& 
        Shoot_Control_Struct.Heat_Limit == 0 && 
        Shoot_Control_Struct.Cool_Down == 0 &&
        Shoot_Control_Struct.Trigger_Block_Flag == 0 &&
        Shoot_Control_Struct.Shoot_Frequency >0.0f
    ) 
    {
        Shoot_Control_Struct.Trigger_Target_Angle += Num * 360.0f / Trigger_Bullet_Num;
        // if(Shoot_Control_Struct.Shoot_Frequency <= 0) Shoot_Control_Struct.Shoot_Frequency = 1;
        Shoot_Control_Struct.Cool_Down += Num/Shoot_Control_Struct.Shoot_Frequency;
        Vir_Heat+=10.0f;
    }
}  

/**
 * @brief 退n发
 */
void Shoot_Getout(float n)
{
    Shoot_Control_Struct.Trigger_Target_Angle -= 360.0f / 9.0f * n;
}


/*
 * @brief 热量控制
 */

void Shoot_Update_VirHeat(void)
{
	Vir_Heat = Robo_PowerHeatData.shooter_17mm_barrel_heat;
}


void Heat_Limit(void)
{ 
	Vir_Heat -= Shoot_Dt * Robo_State.shooter_barrel_cooling_value;
	float Qrelease = Robo_State.shooter_barrel_heat_limit - Vir_Heat - Heat_Limit_surplus;

	float K = Qrelease / Heat_Limit_linear;
	// Limit_float(&K, 1, 0);
	Shoot_Control_Struct.Shoot_Frequency = Shoot_Frequency_Max * K;
    if(Shoot_Control_Struct.Shoot_Frequency<=0)Shoot_Control_Struct.Shoot_Frequency=0;
    else if(Shoot_Control_Struct.Shoot_Frequency<=(Robo_State.shooter_barrel_cooling_value/10.0f*0.8f))Shoot_Control_Struct.Shoot_Frequency=(Robo_State.shooter_barrel_cooling_value/10.0f*0.8f);
    else Limit_float(&Shoot_Control_Struct.Shoot_Frequency, Shoot_Frequency_Max, Robo_State.shooter_barrel_cooling_value/10.0f*0.8f);

	// Limit_float(&Vir_Heat, Robo_State.shooter_barrel_heat_limit, 0);
    if(Vir_Heat<0.0f)Vir_Heat=0.0f; 
	
	if(Vir_Heat + Heat_Limit_surplus >= Robo_State.shooter_barrel_heat_limit) Shoot_Control_Struct.Heat_Limit = 1;
	else Shoot_Control_Struct.Heat_Limit = 0;
}

/**
 * @brief 卡弹检测和处理
 */
void Trigger_Stuck_Handle(void)
{
    /*===| 如果拨弹轮有电流无转速，则代表卡弹 |===*/
    if(fabsf(Shoot_Control_Struct.Trigger_Send_Current) > 14000 && fabsf((float)Motor.Trigger.Speed_RPM) < 100)
    {
        if(Shoot_Control_Struct.Trigger_Block_Flag == 0)
        Shoot_Control_Struct.Trigger_Block_Time++; 
    }  
    else 
    {
        Shoot_Control_Struct.Trigger_Block_Time = 0;
    }

    /*===| 达到堵转时间阈值 |===*/
    if(Shoot_Control_Struct.Trigger_Block_Time >= 300) 
    {
        Shoot_Control_Struct.Trigger_Block_Flag = 1; 
        Shoot_Control_Struct.Trigger_Release_Time = 0;
        Shoot_Control_Struct.Trigger_Block_Time = 0;
    }           

    /*===| 卡弹处理 |===*/
    if(Shoot_Control_Struct.Trigger_Block_Flag)
    {   
        if(Shoot_Control_Struct.Trigger_Release_Time == 1)
        {
			/*===| 退0.3弹位 |===*/ 
            //重置目标角度
            Shoot_Control_Struct.Trigger_Target_Angle = Motor.Trigger.Total_Angle / Trigger_Reduction;
            Shoot_Control_Struct.Trigger_Target_Angle -= 0.8f * 360.0f / Trigger_Bullet_Num;
        }
        
        if(Shoot_Control_Struct.Trigger_Release_Time < 500)
        {
            Shoot_Control_Struct.Trigger_Release_Time++;
        }
        else
        {   
            Shoot_Control_Struct.Trigger_Block_Flag = 0;
            Shoot_Control_Struct.Trigger_Release_Time = 0;
        }
    }
}

void Fric_Stuck_Handle(void)
{
    /*===| 如果摩擦轮有电流无转速，则代表卡弹 |===*/
    if((fabs(Shoot_Control_Struct.Fric_First_Left_Speed_PID_Struct.Output) > 10000 && fabs((float)Motor.Fric1.Speed_RPM) < 100) 
        && (fabs(Shoot_Control_Struct.Fric_First_Right_Speed_PID_Struct.Output) > 10000 && fabs((float)Motor.Fric2.Speed_RPM) < 100) )
    {
        if(Shoot_Control_Struct.Fric_Block_Flag == 0)
        Shoot_Control_Struct.Fric_Block_Time++; 
    }  
    else 
    {
        Shoot_Control_Struct.Fric_Block_Time = 0;
    }

    /*===| 达到堵转时间阈值 |===*/
    if(Shoot_Control_Struct.Fric_Block_Time >= 500) 
    {
        Shoot_Control_Struct.Fric_Block_Flag = 1; 
        Shoot_Control_Struct.Fric_Release_Time = 0;
        Shoot_Control_Struct.Fric_Block_Time = 0;
    }           

	/*===| 卡弹处理 |===*/
    if(Shoot_Control_Struct.Fric_Block_Flag)
    {   
        if(Shoot_Control_Struct.Fric_Release_Time == 1)
        {
            /*===| 退0.3弹位 |===*/ 
            //重置目标角度
            Shoot_Control_Struct.Trigger_Target_Angle = Motor.Trigger.Total_Angle / Trigger_Reduction;
            Shoot_Control_Struct.Trigger_Target_Angle -= 0.3f * 360.0f / Trigger_Bullet_Num;
        }
        
        if(Shoot_Control_Struct.Fric_Release_Time < 1000)
        {
            /*===| 摩擦轮倒转1000ms |===*/
            RoboControl_Struct.Shoot_State = Shoot_State_Unload;
            
            Shoot_Control_Struct.Fric_Release_Time++;
        }
        else
        {   
            Shoot_Control_Struct.Fric_Block_Flag = 0;
            Shoot_Control_Struct.Fric_Block_Time = 0;
            Shoot_Control_Struct.Fric_Release_Time = 0;
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }
}

