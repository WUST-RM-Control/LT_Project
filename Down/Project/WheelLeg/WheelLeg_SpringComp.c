/**
 * @file    WheelLeg_SpringComp.c[氮气弹簧补偿]
 * @brief   氮气弹簧标定和补偿
 * @details 包含初始化函数和补偿函数，还有阻塞式氮气弹簧标定任务
 * 
 */

#include "WheelLeg_SpringComp.h"
#include "Buzzer.h"
#include <math.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//内部结构体定义
typedef enum
{
    Down =  1,
    Up   = -1
}Start_State_EnumTypedef;

typedef enum 
{
	Leg_L = 0,
	Leg_R = 1
} Chassis_Leg_RL_State_EnumTypedef;

typedef struct 
{
    //0正转(初始值转到末端值)，1反转
    uint8_t Caled_State;
    //初始转动的目标值(UP:一开始目标值减小，Down:一开始目标值增大)
    Start_State_EnumTypedef Start_State;

    float Compensation_Speed;//每秒改变的目标值
    int8_t Compensation_Round;
    float Compensation_Distance_UP;
    float Compensation_Distance_Down;
    //区间大小：一般360(略大于或等于最大的Compensation_Distance_UP-Compensation_Distance_Down，最大指排除机械限位后可能的角度)
    float Compensation_Distance_Range;
    
    float Distance_Target;
    float Distance_Feedback; 
    float SpeedRPM_Feedback; 
    float Current_Output;
    
    PID_Struct_TypeDef Distance_PID;
    PID_Struct_TypeDef Speed_PID;
    
    float* Data;
    
    uint32_t DWT_Counter;
    float Dt;

    float Friction_K;
    float Friction_Out;
    float Friction_ACC;
    float Friction_DEC;


    //Leg数据
        Chassis_Leg_RL_State_EnumTypedef Leg_RL_State;
		float L0_Target;//一阶倒立摆腿长
		float L0_Feedback;
		float L0_Last;
		float L0_Speed;
		float L0_Speed_Last;
		
		float A0_Target;//一阶倒立摆腿角度
		float A0_Feedback;
		float A0_Last;
		float A0_Round;//圈数

		float Total_A0_Target;//一阶倒立摆腿总角度
		float Total_A0_Feedback;
		float Total_A0_Last;
		float Total_A0_Speed;
	
	/*===| 雅可比力矩阵 |===*/
	float Jt[2][2];

	//关节电机目标力矩
	float T_Target[2];
	//关节电机VMC
	//0L:沿杆方向的，1T关节为绕轴力矩
	float L_T_Target[2];


} Motor_Compensation_Config_StructTypedef;

typedef struct
{
    float L0_Leg_L_Cal_Data[2048];
    float L0_Leg_R_Cal_Data[2048];
} Config_StructTypedef;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//内部函数声明
static void Leg_Date_Compensation_Calculate(Motor_Compensation_Config_StructTypedef *L0_Leg_Compensation_Config,const volatile Motor_Data_StructTypeDef * const Motor1,const volatile Motor_Data_StructTypeDef * const Motor2,const volatile Motor_Data_StructTypeDef * const Motor3,const volatile Motor_Data_StructTypeDef * const Motor4);
static void Motor_Compensation(Motor_Compensation_Config_StructTypedef *Config);
static float Motor_Compensation_Get_Data(float Distance, float Speed, Motor_Compensation_Config_StructTypedef *Config);
static float WheelLeg_SpringComp_Fit_K_Out_Of_Range(const float *Cal_Data,Motor_Compensation_Config_StructTypedef Config);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//补偿精度
//单圈采样点数，保存数组需要 2[正|反圈 & 齿槽补偿|摩擦补偿] * 4[float类型] * Point_Num字节大小
#define Point_Num 1024

//解算有关定义
#define Angle_to_Radain	 0.0174533f //角度to弧度(PI/180)
#define	Radain_to_Angle	 57.29578f  //弧度to角度(180/PI)
#define L1_UP    0.21    //大腿长度m
#define L2_Down  0.25    //小腿长度m

//超出标定范围外推:拟合所用末尾有效数据项数
#define Out_Of_Range_Fit_Num 40

//0:L   1:R
//末尾Out_Of_Range_Fit_Num项有效数据拟合直线斜率(力矩/腿长)
static float WheelLeg_SpringComp_K_Out_Of_Range[2];
//有效数据(默认UP的末尾15个不可用)，在WheelLeg_SpringComp_Fit_K_Out_Of_Range中赋值
static int16_t Encoder_Max[2];


static Config_StructTypedef Config;

static Motor_Compensation_Config_StructTypedef L0_Leg_L_Compensation_Config = 
{
    .Caled_State = 0,
    .Start_State =Up,

    .Compensation_Speed = 0.005,
    .Compensation_Round = 1,
    .Compensation_Distance_UP = 0.348f,
    .Compensation_Distance_Down = 0.128,
    .Compensation_Distance_Range = 1,

    .Data = Config.L0_Leg_L_Cal_Data,
    
    .Friction_ACC = 20000,
    .Friction_DEC = 50000,
    .Friction_K = 1,

    .Leg_RL_State = Leg_L
};

static Motor_Compensation_Config_StructTypedef L0_Leg_R_Compensation_Config = 
{
    .Caled_State = 0,
    .Start_State =Up,
    
    .Compensation_Round = 1,
    .Compensation_Speed = 0.005,
    .Compensation_Distance_UP = 0.348f,
    .Compensation_Distance_Down = 0.128,
    .Compensation_Distance_Range = 1,

    .Data = Config.L0_Leg_R_Cal_Data,
    
    .Friction_ACC = 20000,
    .Friction_DEC = 50000,
    .Friction_K = 1,

    .Leg_RL_State = Leg_R
};

//锁角度PID参数
static PID_Struct_TypeDef Leg_Angle_PID =
{
    .Kp = 80.0f,
    .Ki = 0.0f,
    .Kd = 150.0f,
    .Kf = 0.0f,
    .I_Output_Max = 0.0f,
    .Output_Max = 150.0f,
    .Targer_LowPass_K = 1.0f
};
static PID_Struct_TypeDef Leg_Angle_Speed_PID =
{
    .Kp = 0.1f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .Kf = 0.15f,
    .I_Output_Max = 0.0f,
    .Output_Max = 30.0f,
    .Targer_LowPass_K = 1.0f
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * ===| 氮气弹簧初始化 |===
 **/
void WheelLeg_SpringComp_Init(void)
{
	for (uint16_t i = 0; i < 1024; i++)
	{
		memcpy(&((uint64_t *)(Config.L0_Leg_L_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048 * (100 + 128) + 8 * i), 8);
	}
    for(uint16_t i = 0; i<1024; i++)
    {
        memcpy(&((uint64_t *)(Config.L0_Leg_R_Cal_Data))[i], (uint32_t *)(0x08000000 + 2048*(104+128) + 8*i), 8);
    }
	WheelLeg_SpringComp_K_Out_Of_Range[Leg_L] = WheelLeg_SpringComp_Fit_K_Out_Of_Range(Config.L0_Leg_L_Cal_Data, L0_Leg_L_Compensation_Config);
	WheelLeg_SpringComp_K_Out_Of_Range[Leg_R] = WheelLeg_SpringComp_Fit_K_Out_Of_Range(Config.L0_Leg_R_Cal_Data, L0_Leg_R_Compensation_Config);
}

/**
 * ===| 氮气弹簧补偿 |===
 * @param Leg_Length_Feedback---当前腿长
 * @param Leg_Length_Target-----目标腿长
 * @param GasSpring_Output------[out]输出补偿力矩(沿腿方向)
 * @param Leg_LR----------------0:左腿(Leg_L)，1:右腿(Leg_R)
 **/
void WheelLeg_SpringComp(float Leg_Length_Feedback,float Leg_Length_Target,float *GasSpring_Output,uint8_t Leg_LR)
{
    if(Leg_LR == Leg_L)
    {
        *GasSpring_Output = Motor_Compensation_Get_Data(Leg_Length_Feedback, Leg_Length_Target-Leg_Length_Feedback, &L0_Leg_L_Compensation_Config);
    }
    else if(Leg_LR == Leg_R)
    {
        *GasSpring_Output = Motor_Compensation_Get_Data(Leg_Length_Feedback, Leg_Length_Target-Leg_Length_Feedback, &L0_Leg_R_Compensation_Config);
    }
    else
    {
        *GasSpring_Output = 0;
    }
}

/**
 * @brief ===| 氮气弹簧标定任务 |===
 * 
 * @attention 不换氮气弹簧和氮气弹簧未受损的情况下大概2个月标定一次就够了！！
 * @attention 不要反复标定，对电机有寿命有影响
 * 
 * @note 说明：
 * @note Enable_Output：指向使能标志的指针。函数开始会阻塞等待其从 0 变为 1；
 * @note               标定过程中若其变为 0，会立即退出函数并返回 0，以保证安全;
 * @note                需确保 Enable_Output 可在其他进程中被置 0，以便意外时安全退出;
 * @note 阻塞式标定，正常结束返回 1，中途失能返回 0;
 * @note 标定过程中会控制关节电机输出力矩，轮电机输出 0;
 * @note 标定完成后会擦除对应 Flash 页并写入标定数据，延时 500ms 后播放系统启动音效;
 * 
 * @note 使用：
 * @note 电池电量大于等于3
 * @note 确保参数无误后放在Chassis_Task的While循环的前面，标定过程中完全由WheelLeg_SpringComp_Measure_Task控制底盘
 * @note 确保上电或代码烧录进去前遥控器是失能状态(Enable_Output==0)
 * @note 机体水平静止放置，将要标定的腿垂直于机体、竖直向下放置
 * @note 确保腿收伸时不会有其他干扰(例如:腿受到地面的支持力，受到其他物体的摩擦力或干涉)
 * @note 遥控器使能(Enable_Output==1)
 * @note 如遇到问题立即失能(Enable_Output==0)
 * 
 * 标定过程中腿应该是垂直于机体竖直向下的(有锁摆角的力)，如果有异常抖动请修改Leg_Angle_PID和Leg_Angle_Speed_PID参数(两腿共用此套参数)
 * 标定过程中电机会略微发热是正常情况，但不要反复标定，对电机有寿命有影响
 * 
 * 
 * @param Leg_LR-----------------0:左腿(Leg_L)，1:右腿(Leg_R)
 * @param Enable_Output----------使能输出指针，volatile uint8_t*
 * @param Fun_Motor_Joint_Wheel_Output-关节电机输出函数指针，参数依次为关节1、2、3、4和左轮、右轮的力矩
 * @param Motor1-----------------电机1数据
 * @param Motor2-----------------电机2数据
 * @param Motor3-----------------电机3数据
 * @param Motor4-----------------电机4数据
 * 
 * @return 1：标定成功并已保存 Flash
 * @return 0：中途 Enable_Output 变为 0 而退出
 * @return -1 Leg_LR非法
 **/
uint8_t WheelLeg_SpringComp_Measure_Task
(const uint8_t Leg_LR,
const volatile uint8_t * const Enable_Output,
void (* const Fun_Motor_Joint_Wheel_Output)(float,float,float,float,float,float),
const volatile Motor_Data_StructTypeDef * const Motor1,
const volatile Motor_Data_StructTypeDef * const Motor2,
const volatile Motor_Data_StructTypeDef * const Motor3,
const volatile Motor_Data_StructTypeDef * const Motor4)
{
    if(Leg_LR!=Leg_L&&Leg_LR!=Leg_R) return -1;

    while(*Enable_Output == 0)osDelay(5);

if(Leg_LR==Leg_L)
{
/*===| 左腿 |===*/

    //校准参数初始化
    L0_Leg_L_Compensation_Config.Caled_State = 0;
    L0_Leg_L_Compensation_Config.Compensation_Round = 1;

    if(L0_Leg_L_Compensation_Config.Start_State == Up)
    L0_Leg_L_Compensation_Config.Distance_Target = L0_Leg_L_Compensation_Config.Compensation_Distance_UP;
    else
    L0_Leg_L_Compensation_Config.Distance_Target = L0_Leg_L_Compensation_Config.Compensation_Distance_Down;
    //PID参数初始化
	PID_Init(&L0_Leg_L_Compensation_Config.Distance_PID, 	10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
	PID_Init(&L0_Leg_L_Compensation_Config.Speed_PID, 	    80.0f, 5.0f, 0.0f, 0.0f, 250.0f, 250.0f);

    //清空原先的校准数据
    for(uint16_t i = 0; i < 2*Point_Num; i++) L0_Leg_L_Compensation_Config.Data[i] = 0; 
    
    //循环校准
    while(L0_Leg_L_Compensation_Config.Caled_State <= 1)
    {
        L0_Leg_L_Compensation_Config.Dt = DWT_GetDeltaT(&L0_Leg_L_Compensation_Config.DWT_Counter);
        //前置计算与采样赋值
        Leg_Date_Compensation_Calculate(&L0_Leg_L_Compensation_Config,Motor1,Motor2,Motor3,Motor4);

        //更新校准数据(设置目标角度与速度，PID计算，保存校准值)
        Motor_Compensation(&L0_Leg_L_Compensation_Config);
        //输出电流
        {
                L0_Leg_L_Compensation_Config.L_T_Target[0] = L0_Leg_L_Compensation_Config.Current_Output;
                L0_Leg_L_Compensation_Config.L_T_Target[1] = Leg_Angle_Speed_PID.Output;

                L0_Leg_L_Compensation_Config.T_Target[0] = L0_Leg_L_Compensation_Config.Jt[0][0] * L0_Leg_L_Compensation_Config.L_T_Target[0]
                                                         + L0_Leg_L_Compensation_Config.Jt[0][1] * L0_Leg_L_Compensation_Config.L_T_Target[1];
                L0_Leg_L_Compensation_Config.T_Target[1] = L0_Leg_L_Compensation_Config.Jt[1][0] * L0_Leg_L_Compensation_Config.L_T_Target[0]
                                                         + L0_Leg_L_Compensation_Config.Jt[1][1] * L0_Leg_L_Compensation_Config.L_T_Target[1];
                Limit_float(&L0_Leg_L_Compensation_Config.T_Target[0] ,40.0f, -40.0f);
                Limit_float(&L0_Leg_L_Compensation_Config.T_Target[1] ,40.0f, -40.0f);
                
            //电管底盘失能 关节电机也失能
            if( *Enable_Output == 0)
            {
                Fun_Motor_Joint_Wheel_Output(0, 0, 0, 0, 0, 0);
                return 0;
            }		
            else
            {
                Fun_Motor_Joint_Wheel_Output(L0_Leg_L_Compensation_Config.T_Target[0], L0_Leg_L_Compensation_Config.T_Target[1], 0, 0, 0, 0);
            }

        }
        osDelay(1);
    }

    //计算齿槽补偿与摩擦补偿
    for(uint16_t i = 0; i < Point_Num; i++) 
    {
        float Coggin_Torque   = 0.5f * (L0_Leg_L_Compensation_Config.Data[i] + L0_Leg_L_Compensation_Config.Data[i+Point_Num]);
        float Friction_Torque = 0.5f * (L0_Leg_L_Compensation_Config.Data[i] - L0_Leg_L_Compensation_Config.Data[i+Point_Num]);
        L0_Leg_L_Compensation_Config.Data[i] = Coggin_Torque;
        L0_Leg_L_Compensation_Config.Data[i+Point_Num] = Friction_Torque;
    }
}
else if(Leg_LR==Leg_R)
{
/*===| 右腿 |===*/
    //校准参数初始化
    L0_Leg_R_Compensation_Config.Caled_State = 0;
    L0_Leg_R_Compensation_Config.Compensation_Round = 1;
    if(L0_Leg_R_Compensation_Config.Start_State == Up)
    L0_Leg_R_Compensation_Config.Distance_Target = L0_Leg_R_Compensation_Config.Compensation_Distance_UP;
    else
    L0_Leg_R_Compensation_Config.Distance_Target = L0_Leg_R_Compensation_Config.Compensation_Distance_Down;
    //PID参数初始化
	PID_Init(&L0_Leg_R_Compensation_Config.Distance_PID, 	10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
	PID_Init(&L0_Leg_R_Compensation_Config.Speed_PID, 	    80.0f, 5.0f, 0.0f, 0.0f, 250.0f, 250.0f);

    //清空原先的校准数据
    for(uint16_t i = 0; i < 2*Point_Num; i++) L0_Leg_R_Compensation_Config.Data[i] = 0; 
    
    //循环校准
    while(L0_Leg_R_Compensation_Config.Caled_State <= 1)
    {
        L0_Leg_R_Compensation_Config.Dt = DWT_GetDeltaT(&L0_Leg_R_Compensation_Config.DWT_Counter);
        //前置计算与采样赋值
        Leg_Date_Compensation_Calculate(&L0_Leg_R_Compensation_Config,Motor1,Motor2,Motor3,Motor4);


        //更新校准数据(设置目标角度与速度，PID计算，保存校准值)
        Motor_Compensation(&L0_Leg_R_Compensation_Config);
        //输出电流
        {
                L0_Leg_R_Compensation_Config.L_T_Target[0] = L0_Leg_R_Compensation_Config.Current_Output;
                L0_Leg_R_Compensation_Config.L_T_Target[1] = Leg_Angle_Speed_PID.Output;

                L0_Leg_R_Compensation_Config.T_Target[0] = L0_Leg_R_Compensation_Config.Jt[0][0] * L0_Leg_R_Compensation_Config.L_T_Target[0]
                                                         + L0_Leg_R_Compensation_Config.Jt[0][1] * L0_Leg_R_Compensation_Config.L_T_Target[1];
                L0_Leg_R_Compensation_Config.T_Target[1] = L0_Leg_R_Compensation_Config.Jt[1][0] * L0_Leg_R_Compensation_Config.L_T_Target[0]
                                                         + L0_Leg_R_Compensation_Config.Jt[1][1] * L0_Leg_R_Compensation_Config.L_T_Target[1];
                Limit_float(&L0_Leg_R_Compensation_Config.T_Target[0] ,40.0f, -40.0f);
                Limit_float(&L0_Leg_R_Compensation_Config.T_Target[1] ,40.0f, -40.0f);
                

            //电管底盘失能 关节电机也失能
            if( *Enable_Output == 0)
            {
                Fun_Motor_Joint_Wheel_Output(0, 0, 0, 0, 0, 0);
                return 0;
            }		
            else
            {
                Fun_Motor_Joint_Wheel_Output(0, 0, L0_Leg_R_Compensation_Config.T_Target[1], L0_Leg_R_Compensation_Config.T_Target[0], 0, 0);
            }

        }
        osDelay(1);
    }

    //计算齿槽补偿与摩擦补偿
    for(uint16_t i = 0; i < Point_Num; i++) 
    {
        float Coggin_Torque   = 0.5f * (L0_Leg_R_Compensation_Config.Data[i] + L0_Leg_R_Compensation_Config.Data[i+Point_Num]);
        float Friction_Torque = 0.5f * (L0_Leg_R_Compensation_Config.Data[i] - L0_Leg_R_Compensation_Config.Data[i+Point_Num]);
        L0_Leg_R_Compensation_Config.Data[i] = Coggin_Torque;
        L0_Leg_R_Compensation_Config.Data[i+Point_Num] = Friction_Torque;
    }
}

    //有效值保存
	if(*Enable_Output != 0)
	{
        if(Leg_LR == Leg_L)
        {
            FLASH_ErasePage(100);
			FLASH_ErasePage(101);
			FLASH_ErasePage(102);
			FLASH_ErasePage(103);
			for (uint16_t i = 0; i < 1024; i++)
			{
				FLASH_programword(0x08000000 + 2048 * (100 + 128) + 8 * i, ((uint64_t *)(Config.L0_Leg_L_Cal_Data))[i]);
			}
        }
		else if(Leg_LR == Leg_R)
        {

			FLASH_ErasePage(104);
			FLASH_ErasePage(105);
			FLASH_ErasePage(106);
			FLASH_ErasePage(107);
			for(uint16_t i = 0; i<1024; i++)
			{
				FLASH_programword(0x08000000 + 2048*(104+128) + 8*i, ((uint64_t *)(Config.L0_Leg_R_Cal_Data))[i]);
			}
		}

		osDelay(500);
		//标定完成音效
		Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SystemStart);
        return 1;
	}
    return 0;
}

static void Leg_Date_Compensation_Calculate(Motor_Compensation_Config_StructTypedef *L0_Leg_Compensation_Config,const volatile Motor_Data_StructTypeDef * const Motor1,const volatile Motor_Data_StructTypeDef * const Motor2,const volatile Motor_Data_StructTypeDef * const Motor3,const volatile Motor_Data_StructTypeDef * const Motor4)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//哈工程正运动学解算
	float Xb,Yb;
	float Xd,Yd;
	float Xc,Yc;
	
	float A0,B0,C0,lBD;
	float PHI0,PHI1,PHI2,PHI3,PHI4; //弧度制
	float l5=0;
	float phi1,phi2,phi3,phi4;		//角度制
	float L0;
	float phi0;
	
	if(L0_Leg_Compensation_Config->Leg_RL_State == Leg_L)
	{
		phi1 = Motor1->Total_Angle + 180.0f;
		phi4 = Motor2->Total_Angle;
	}
	else if(L0_Leg_Compensation_Config->Leg_RL_State == Leg_R)
	{
		phi1 = Motor4->Total_Angle + 180.0f;
		phi4 = Motor3->Total_Angle;
	}
	

	PHI1 = phi1 * Angle_to_Radain;
	PHI4 = phi4 * Angle_to_Radain;
	
	Xb = L1_UP*arm_cos_f32(PHI1);
	Yb = L1_UP*arm_sin_f32(PHI1);
	Xd = L1_UP*arm_cos_f32(PHI4) + l5;
	Yd = L1_UP*arm_sin_f32(PHI4);
	
	lBD=sqrtf((Xd-Xb)*(Xd-Xb)+(Yd-Yb)*(Yd-Yb));
	A0=2*L2_Down*(Xd-Xb);
	B0=2*L2_Down*(Yd-Yb);
	C0=L2_Down*L2_Down+lBD*lBD-L2_Down*L2_Down;
	
	PHI2=2*atan2f((B0+sqrtf(A0*A0+B0*B0-C0*C0)),(A0+C0));
	PHI3=atan2f(Yb-Yd+L2_Down*arm_sin_f32(PHI2),Xb-Xd+L2_Down*arm_cos_f32(PHI2));
	
	Xc=Xb+L2_Down*arm_cos_f32(PHI2);
	Yc=Yb+L2_Down*arm_sin_f32(PHI2);

	L0=sqrtf((Xc-l5/2)*(Xc-l5/2)+Yc*Yc);
	PHI0=atan2f(Yc,(Xc-l5/2));
	phi0=PHI0*Radain_to_Angle;
	

	//Last数据赋值
	L0_Leg_Compensation_Config->L0_Last = L0_Leg_Compensation_Config->L0_Feedback;
	L0_Leg_Compensation_Config->A0_Last = L0_Leg_Compensation_Config->A0_Feedback;
	L0_Leg_Compensation_Config->Total_A0_Last = L0_Leg_Compensation_Config->Total_A0_Feedback;
	//Feedback数据赋值
	L0_Leg_Compensation_Config->L0_Feedback = L0;
	L0_Leg_Compensation_Config->A0_Feedback = phi0;
	//计算总角度值
    if      (L0_Leg_Compensation_Config->A0_Feedback - L0_Leg_Compensation_Config->A0_Last >  355.0f) L0_Leg_Compensation_Config->A0_Round--;
    else if (L0_Leg_Compensation_Config->A0_Feedback - L0_Leg_Compensation_Config->A0_Last < -355.0f) L0_Leg_Compensation_Config->A0_Round++;
    L0_Leg_Compensation_Config->Total_A0_Feedback = 360.0f * L0_Leg_Compensation_Config->A0_Round +L0_Leg_Compensation_Config->A0_Feedback;
	//Speed数据赋值
	L0_Leg_Compensation_Config->L0_Speed = (L0_Leg_Compensation_Config->L0_Feedback - L0_Leg_Compensation_Config->L0_Last) / L0_Leg_Compensation_Config->Dt;
	L0_Leg_Compensation_Config->Total_A0_Speed = (L0_Leg_Compensation_Config->Total_A0_Feedback - L0_Leg_Compensation_Config->Total_A0_Last) / L0_Leg_Compensation_Config->Dt;
	
    //***采样电机角度与速度
    L0_Leg_Compensation_Config->Distance_Feedback = L0_Leg_Compensation_Config->L0_Feedback;
    L0_Leg_Compensation_Config->SpeedRPM_Feedback = L0_Leg_Compensation_Config->L0_Speed;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//哈工程VMC
	// float d_phi1_L, d_phi4_L;
	// float dxc_L   , dyc_L;

	// float J[2][2] ={0};
	// float Jt[2][2]={0};
	// float JT[2][2]={0};
	
	// d_phi1_L=Motor.Chassis_Joint1.Speed_RPM *2.0f*PI/60.0f;//角速度
	// d_phi4_L=Motor.Chassis_Joint2.Speed_RPM *2.0f*PI/60.0f;//角速度

	// /*===| 计算雅各比矩阵J |===*/
	// J[0][0]= (yang_L1*arm_sin_f32(PHI1-PHI2)*arm_sin_f32(PHI3)) / (arm_sin_f32(PHI2-PHI3));
	// J[0][1]= (yang_L1*arm_sin_f32(PHI3-PHI4)*arm_sin_f32(PHI2)) / (arm_sin_f32(PHI2-PHI3));
	// J[1][0]=-(yang_L1*arm_sin_f32(PHI1-PHI2)*arm_cos_f32(PHI3)) / (arm_sin_f32(PHI2-PHI3));
	// J[1][1]=-(yang_L1*arm_sin_f32(PHI3-PHI4)*arm_cos_f32(PHI2)) / (arm_sin_f32(PHI2-PHI3));
	/*===| 计算矩阵Jᵀ*R*M |===*/
	L0_Leg_Compensation_Config->Jt[0][0]=(L1_UP*arm_sin_f32(PHI0-PHI3)*arm_sin_f32(PHI1-PHI2)) / (arm_sin_f32(PHI3-PHI2));
	L0_Leg_Compensation_Config->Jt[0][1]=(L1_UP*arm_cos_f32(PHI0-PHI3)*arm_sin_f32(PHI1-PHI2)) / (L0*arm_sin_f32(PHI3-PHI2));
	L0_Leg_Compensation_Config->Jt[1][0]=(L1_UP*arm_sin_f32(PHI0-PHI2)*arm_sin_f32(PHI3-PHI4)) / (arm_sin_f32(PHI3-PHI2));
	L0_Leg_Compensation_Config->Jt[1][1]=(L1_UP*arm_cos_f32(PHI0-PHI2)*arm_sin_f32(PHI3-PHI4)) / (L0*arm_sin_f32(PHI3-PHI2));
	
	// /*===| 速度映射部分 |===*/
	// dxc_L=J[0][0]*d_phi1_L + J[0][1]*d_phi4_L;
	// dyc_L=J[1][0]*d_phi1_L + J[1][1]*d_phi4_L;

    //目标角度
    L0_Leg_Compensation_Config->Total_A0_Target = 360.0f * L0_Leg_Compensation_Config->A0_Round + 90.0f;
	/*===| 目标VMC力矩PID计算 |===*/
	PID_Position_Calculate(&Leg_Angle_PID, 	        L0_Leg_Compensation_Config->Total_A0_Target, 	    L0_Leg_Compensation_Config->Total_A0_Feedback);
	PID_Position_Calculate(&Leg_Angle_Speed_PID, 	Leg_Angle_PID.Output,	L0_Leg_Compensation_Config->Total_A0_Speed);
}

static void Motor_Compensation(Motor_Compensation_Config_StructTypedef *Config)
{
    if(Config->Caled_State <= 1)
    {
        
        //目标值增减
        if(fabsf(Config->Distance_Target - Config->Distance_Feedback) <= Config->Compensation_Distance_Range*0.01f
        ||((Config->Caled_State == 0)?1:-1)*(Config->Distance_Target - Config->Distance_Feedback) * Config->Start_State<0.0f)
        {
            //串级PID定速正反转(0正传，1反转)
            if     (Config->Caled_State == 0) Config->Distance_Target += Config->Dt * Config->Compensation_Speed * Config->Start_State;
            else if(Config->Caled_State == 1) Config->Distance_Target -= Config->Dt * Config->Compensation_Speed * Config->Start_State;
        }

        if(Config->Start_State == Up)
        {
            //检测正反转改变
            if     (Config->Distance_Target  <= Config->Compensation_Distance_Down  && Config->Caled_State == 0) {Config->Caled_State = 1;}
            else if(Config->Distance_Target  >= Config->Compensation_Distance_UP    && Config->Caled_State == 1) {Config->Caled_State = 0; Config->Compensation_Round--;}
            if(Config->Compensation_Round <= 0) {Config->Caled_State = 2; return;}
        }
        else
        {
            //检测正反转改变
            if     (Config->Distance_Target >= Config->Compensation_Distance_UP   && Config->Caled_State == 0) {Config->Caled_State = 1;}
            else if(Config->Distance_Target <= Config->Compensation_Distance_Down && Config->Caled_State == 1) {Config->Caled_State = 0; Config->Compensation_Round--;}
            if(Config->Compensation_Round <= 0) {Config->Caled_State = 2; return;}
        }
        
        PID_Position_Calculate(&Config->Distance_PID, Config->Distance_Target, Config->Distance_Feedback);
        PID_Position_Calculate(&Config->Speed_PID, Config->Distance_PID.Output, Config->SpeedRPM_Feedback);
        Config->Current_Output = Config->Speed_PID.Output;
        
        //计算角度编码器值
        int16_t Encoder = fmodf(Config->Distance_Feedback, Config->Compensation_Distance_Range) * Point_Num / Config->Compensation_Distance_Range;
        if(Encoder < 0) Encoder += Point_Num;
        
        //保存校准值(正转保存在数据数组前半部分，反转在后半部分)
        if(fabsf(Config->Distance_PID.Error) < Config->Compensation_Distance_Range*0.008f)
        {
            if(Config->Caled_State == 0) Config->Data[Encoder]           = 0.4f*Config->Data[Encoder]             + 0.6f*Config->Current_Output;
            if(Config->Caled_State == 1) Config->Data[Encoder+Point_Num] = 0.4f*Config->Data[Encoder+Point_Num]   + 0.6f*Config->Current_Output;
        }
    }
}

static float Motor_Compensation_Get_Data(float Distance, float Error, Motor_Compensation_Config_StructTypedef *Config)
{
    float Distance_Limit = Distance;
    Limit_float(&Distance_Limit, Config->Compensation_Distance_UP, Config->Compensation_Distance_Down);
    int16_t Encoder = fmodf(Distance_Limit, Config->Compensation_Distance_Range) * Point_Num / Config->Compensation_Distance_Range;
    if(Encoder < 0) Encoder += Point_Num; 

    if(Encoder>Encoder_Max[Config->Leg_RL_State]) Encoder = Encoder_Max[Config->Leg_RL_State];

    float Coggin_Torque = Config->Data[Encoder];
    float Friction_Torque_Target = 0.0f;
    float Error_K = 4.0f*fabsf(Error/(Config->Compensation_Distance_UP-Config->Compensation_Distance_Down));
    Limit_float(&Error_K,Config->Friction_K,0.0f);
    if(Error > 0)       Friction_Torque_Target =  Config->Data[Encoder+Point_Num] * Error_K * Config->Start_State;
    else if(Error < 0)  Friction_Torque_Target = -Config->Data[Encoder+Point_Num] * Error_K * Config->Start_State;
    // Acc_Slow(Friction_Torque_Target, &Config->Friction_Out, Config->Friction_ACC, Config->Friction_DEC, Dt);

    float Out_Of_Range_Torque = 0.0f;
    if(Distance>Config->Compensation_Distance_UP)
    {
        //超出标定范围的部分按末尾有效数据拟合的斜率线性外推补偿
        Out_Of_Range_Torque = WheelLeg_SpringComp_K_Out_Of_Range[Config->Leg_RL_State] * (Distance - Distance_Limit);
    }
    

    return (Coggin_Torque + Friction_Torque_Target + Out_Of_Range_Torque);
}

/**
 * ===| 拟合末尾有效数据得到超出标定范围的外推斜率 |===
 **/
static float WheelLeg_SpringComp_Fit_K_Out_Of_Range(const float *Cal_Data,Motor_Compensation_Config_StructTypedef Config)
{
    Encoder_Max[Config.Leg_RL_State] = fmodf(Config.Compensation_Distance_UP, Config.Compensation_Distance_Range) * Point_Num / Config.Compensation_Distance_Range -15;
    int16_t Encoder_Min = fmodf(Config.Compensation_Distance_Down, Config.Compensation_Distance_Range) * Point_Num / Config.Compensation_Distance_Range;
    
    double Sum_X = 0.0, Sum_Y = 0.0, Sum_XX = 0.0, Sum_XY = 0.0;
    uint16_t Num = 0;

    //有效数据区间为[Encoder_Min,Encoder_Max]，从区间末尾向前取 Out_Of_Range_Fit_Num 项
    //标定过程中未被采样到的点保持为0，不计入拟合
    for (int16_t i = Encoder_Max[Config.Leg_RL_State]; i >= Encoder_Min && Num < Out_Of_Range_Fit_Num; i--)
    {
        if (Cal_Data[i] == 0.0f) continue;
        double X = (double)i * Config.Compensation_Distance_Range / (double)Point_Num;
        double Y = Cal_Data[i];
        Sum_X  += X;
        Sum_Y  += Y;
        Sum_XX += X * X;
        Sum_XY += X * Y;
        Num++;
    }

    if (Num < 2) return 0.0f;
    double Denominator = (double)Num * Sum_XX - Sum_X * Sum_X;
    if (Denominator == 0.0) return 0.0f;
    return (float)(((double)Num * Sum_XY - Sum_X * Sum_Y) / Denominator);
}
