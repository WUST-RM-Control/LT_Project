#include "WheelLeg_Chassis.h"

//
#define Chassis_Recover_Angle_Speed 			125.0f		//翻倒回正摆角速度
#define Chassis_RESET_Slow_Frist_Angle_Speed 	125.0f		//缓倒地自起只动摆角时速度
//保持比例180:1(收腿的前中段恰好不碰地，后段碰地)
#define Chassis_RESET_Slow_Second_Angle_Speed 	90.0f		//缓倒地自起收腿时摆角速度
#define Chassis_RESET_Slow_Second_Link_Speed 	0.5f		//缓倒地自起收腿时腿长速度
/**
 * ===| 翻倒自起 & 检测意外翻倒 |===
 * 
 * @note 参数注意：
 * @note 车头抬高,车尾放低时pitch增大(灯条一侧为车尾)
 * @note Pitch范围:-180~180度,车水平正置时Pitch为0度
 * @note 腿在相对机体垂直、竖直向下时腿摆角总为0°+N*360°
 * @note 左腿从左看逆时针为正方向
 * @note 右腿从右看逆时针为正方向
 * 
 * @note 使用说明：
 * @note 只有当Chassis_State等于任意翻倒自起枚举时，才会有输出(改变指针)
 * @note 检测到意外翻倒：Chassis_State不等于任意翻倒自起枚举,而Pitch却不对劲时会返回-1(Error)
 * @note 指针未定义返回0，正常返回1
 * 
 * @note 使用：
 * @note 在需要自起时,让Chassis_State等于Chassis_Recover
 * @note 直接将Total_Angle_Target、Leg_Length_Target带入串级PID计算即可(摆角PID参数给硬一点)
 * @note 最终Chassis_State等于Chassis_FOLLOW时完成翻倒自起
 * 
 *  @param Chassis_Control->Chassis_Dt            控制周期(s)
 *  @param Chassis_Control->Chassis_Control.Chassis_State [out]当前底盘状态
 *  @param Chassis_Control->Chassis_Control.Pitch     当前底盘Pitch(极性见使用注意)
 *  @param Chassis_Control->Leg_Left.Total_Angle_Feedback 左腿总角度反馈(°)
 *  @param Chassis_Control->Leg_Right.Total_Angle_Feedback 右腿总角度反馈(°)
 *  @param L_Angle_Err       左腿摆角 PID 误差(°)
 *  @param R_Angle_Err       右腿摆角 PID 误差(°)
 *  @param Chassis_Control->Chassis_Control.Leg_Length_Feebback 	 腿长反馈(m)
 *  @param L_Length_Err      左腿腿长 PID 误差(m)
 *  @param R_Length_Err      右腿腿长 PID 误差(m)
 *
 *  @param Chassis_Control->Leg_Left.Total_Angle_Target [out] 左腿总角度目标(°)
 *  @param Chassis_Control->Leg_Right.Total_Angle_Target [out] 右腿总角度目标(°)
 *  @param Chassis_Control->Chassis_Control.Leg_Length_Target    [out] 目标腿长(m)
 * 
 * 待完善：
 * 摆角堵转保护
 * 改摆腿逻辑:收腿时腿部从自然位置起来(L:62 70 76 R:64 72 -77   低头，正置，抬头)
 * 改收腿逻辑:机体不会因为摆腿角动量守恒和腿长蹬地导致摇摇晃晃起来
 *=========================================================================================*/
int8_t Chassis_Smooth_Restand(Chassis_Control_StructTypedef *Chassis_Control)
{
    float L_Angle_Err = Chassis_Control->Leg_Left.Total_Angle_Target-Chassis_Control->Leg_Left.Total_Angle_Feedback;
    float R_Angle_Err = Chassis_Control->Leg_Right.Total_Angle_Target-Chassis_Control->Leg_Right.Total_Angle_Feedback;
    float L_Length_Err = Chassis_Control->Chassis_Control.Leg_Length_Target-Chassis_Control->Leg_Left.Length_Feedback;
    float R_Length_Err = Chassis_Control->Chassis_Control.Leg_Length_Target-Chassis_Control->Leg_Right.Length_Feedback;

	//储存Chassis_State值，用于比较底盘状态与赋值
	Chassis_State_EnumTypedef Chassis_Return = Chassis_Control->Chassis_Control.Chassis_State;
	Chassis_State_EnumTypedef Chassis_RESET_Slow_FLag_Return = Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag;

	//用于翻倒回正，左腿摆向，逆时针:1，顺时针:-1
	//翻倒回正:腿摆角方向相同，前翻(Pitch<0车头触地)，左腿逆时针转
	static int8_t Angle_Speed = 0;

	//记录目标最终摆角值用
	static Chassis_State_EnumTypedef Last_State = Chassis_OFF;
	static Chassis_State_EnumTypedef Chassis_RESET_Slow_FLag_Last = 0;
	static float L_Total_End = 0.0f, R_Total_End = 0.0f;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒自起腿摆向和速度
	//记录最终目标腿摆角Total_End
	if(Chassis_Return != Last_State||(Chassis_RESET_Slow_FLag_Last != Chassis_RESET_Slow_FLag_Return&&Chassis_Return == Chassis_RESET_Slow))
	{
		if(Chassis_Return == Chassis_Recover)
		{
			//起点 = 当前总角度反馈
			Chassis_Control->Leg_Left.Total_Angle_Target = Chassis_Control->Leg_Left.Total_Angle_Feedback;
			Chassis_Control->Leg_Right.Total_Angle_Target = Chassis_Control->Leg_Right.Total_Angle_Feedback;
			//车头触地,左腿逆时针转
			if(Chassis_Control->Chassis_Control.Pitch<0.0f)
			{
				Angle_Speed = 1;
			}
			//车尾触地,左腿顺时针转
			else
			{
				Angle_Speed = -1;
			}
		}
		if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 2)
		{
			//起点 = 当前总角度反馈
			Chassis_Control->Leg_Left.Total_Angle_Target = Chassis_Control->Leg_Left.Total_Angle_Feedback;
			Chassis_Control->Leg_Right.Total_Angle_Target = Chassis_Control->Leg_Right.Total_Angle_Feedback;

			//把总角包到 (-180,180],只用于判断"是否落在允许区间"
			float L_A0 = Caculate_Included_Angle(0.0f, Chassis_Control->Leg_Left.Total_Angle_Feedback);
			float R_A0 = Caculate_Included_Angle(0.0f, Chassis_Control->Leg_Right.Total_Angle_Feedback);

			
			//特殊区段，顺时针方向绕远
			if(L_A0 <= 0.0f && L_A0 >= -90.0f)
			{
				L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback -(360.0f-Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, 90.0f));
			}
			else//左腿目标 +90°:走最短弧;
			{
				L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, 90.0f);
			}
			
			if(R_A0 >= 0.0f && R_A0 <= 90.0f)
			{
				R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + (360.0f+Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, -90.0f));
			}
			else
			{
				R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, -90.0f);
			}

		}
		else if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 3)
		{
			//收腿阶段:端点 = 从当前总角度走"最短弧"回到 0°
			L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, 0.0f);
			R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, 0.0f);
		}
		Last_State = Chassis_Return;
        Chassis_RESET_Slow_FLag_Last = Chassis_RESET_Slow_FLag_Return;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒回正
	if(Chassis_Return == Chassis_Recover)
	{
		//自由腿长
		Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;

		//根据摆角夹角设置固定速度归中
		float CenterSpeed_Target = Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, -Chassis_Control->Leg_Right.Total_Angle_Feedback);
		Limit_float(&CenterSpeed_Target, 50.0f, -50.0f);
		
		//翻倒回正检测
		//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
		if(fabsf(Chassis_Control->Chassis_Control.Pitch)<25.0f)
		{
            if(Chassis_RESET_Slow_FLag_Return ==1 )
            {
                Chassis_Return = Chassis_RESET_Slow;
                Chassis_RESET_Slow_FLag_Return = 2;
            }
            else
            {
                Chassis_Return = Chassis_RESET;
                Chassis_RESET_Slow_FLag_Return = 0;
            }
			
		}
		//其他区间：不稳定平衡姿态，摆腿回正
		else
		{
			if(fabsf(L_Angle_Err)< 50.0f)
			Chassis_Control->Leg_Left.Total_Angle_Target += (Chassis_Recover_Angle_Speed * Angle_Speed + CenterSpeed_Target)*Chassis_Control->Chassis_Dt;
			if(fabsf(R_Angle_Err)< 50.0f)
			Chassis_Control->Leg_Right.Total_Angle_Target +=(-Chassis_Recover_Angle_Speed * Angle_Speed + CenterSpeed_Target)*Chassis_Control->Chassis_Dt;
		}
	}
	
	//倒地缓自起(摆腿至90度)
	else if(Chassis_Return == Chassis_RESET_Slow && Chassis_RESET_Slow_FLag_Return == 2)
	{
		//自由腿长
		Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;
		
		/*===| 摆腿阶段:双腿总角度分别缓摆至 90° / -90°|===*/
		Acc_and_Dec(L_Total_End, &Chassis_Control->Leg_Left.Total_Angle_Target, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(R_Total_End, &Chassis_Control->Leg_Right.Total_Angle_Target, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_Control->Chassis_Dt);

		/*===| 摆腿到位:锁存当前腿长(自由腿长)并进入收腿阶段 |===*/
		if(fabsf(L_Total_End - Chassis_Control->Leg_Left.Total_Angle_Target) < 0.05f
		&& fabsf(R_Total_End - Chassis_Control->Leg_Right.Total_Angle_Target) < 0.05f
		&& fabsf(L_Angle_Err) < 5.0f
		&& fabsf(R_Angle_Err) < 5.0f)
		{
			Chassis_Return     = Chassis_RESET_Slow;
            Chassis_RESET_Slow_FLag_Return = 3;
			Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;
		}
	}
	//倒地缓自起(收腿)
	else if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 3)
	{
		/*===| 收腿阶段:总角度回零、腿长缓收至最短(速率与原来一致:90°/s、0.5m/s) |===*/
		Acc_and_Dec(L_Total_End, &Chassis_Control->Leg_Left.Total_Angle_Target, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(R_Total_End, &Chassis_Control->Leg_Right.Total_Angle_Target, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(Leg_Length_Min, &Chassis_Control->Chassis_Control.Leg_Length_Target, Chassis_RESET_Slow_Second_Link_Speed, Chassis_RESET_Slow_Second_Link_Speed, Chassis_Control->Chassis_Dt);

		if(fabsf(L_Total_End - Chassis_Control->Leg_Left.Total_Angle_Target)  < 0.05f
		&& fabsf(R_Total_End - Chassis_Control->Leg_Right.Total_Angle_Target)  < 0.05f
		&& fabsf(L_Angle_Err)  < 5.0f
		&& fabsf(R_Angle_Err)  < 5.0f
		&& fabsf(L_Length_Err) < 0.07f
		&& fabsf(R_Length_Err) < 0.07f)
		{
			/*===| 收腿完成:允许站立 |===*/
			Chassis_Return = Chassis_FOLLOW;
            Chassis_RESET_Slow_FLag_Return = 0;
		}
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//检测意外翻倒计数器，单位:s
	static float Chassis_Pitch_Error_Tim = 0.0f;
	//翻倒检测
	//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
	if(Chassis_Return!=Chassis_Recover&&fabsf(Chassis_Control->Chassis_Control.Pitch)>100.0f)
	{	
		Chassis_Pitch_Error_Tim += Chassis_Control->Chassis_Dt;

		//仍在倒地自起
		if(Chassis_Return==Chassis_RESET_Slow)
		{
			//持续0.1s后重新翻倒回正
			Chassis_Return = Chassis_Recover;
            Chassis_RESET_Slow_FLag_Return = 1;
			Chassis_Pitch_Error_Tim = 0.0f;
		}
        else if(Chassis_Return==Chassis_RESET)
        {
			//持续0.1s后重新翻倒回正
			Chassis_Return = Chassis_Recover;
            Chassis_RESET_Slow_FLag_Return = 0;
			Chassis_Pitch_Error_Tim = 0.0f;
        }
		//意外翻倒
		else if(Chassis_Pitch_Error_Tim>0.05f)
		{
			Chassis_Pitch_Error_Tim = 0.06f;
			return -1;
		}
	}
	else
	{
		Chassis_Pitch_Error_Tim = 0.0f;
	}

	//检测底盘状态改变时间计数器，单位:s
	static float Chassis_State_Change_Tim = 0.0f;
	if(Chassis_Control->Chassis_Control.Chassis_State != Chassis_Return||Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag != Chassis_RESET_Slow_FLag_Return)
	{
		Chassis_State_Change_Tim += Chassis_Control->Chassis_Dt;
		if(Chassis_State_Change_Tim>0.1f)
		{
			Chassis_Control->Chassis_Control.Chassis_State = Chassis_Return;
			Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag = Chassis_RESET_Slow_FLag_Return;
			Chassis_State_Change_Tim = 0.0f;
		}
	}
	else
	{
		Chassis_State_Change_Tim = 0.0f;
	}
	return 1;
}


////错误状态检测
//int8_t Chassis_ErrorState_Detect(void)
//{
//     
//}