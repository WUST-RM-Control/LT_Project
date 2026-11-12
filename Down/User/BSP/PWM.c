#include "PWM.h"

#define PWM_TIM_ARR 20000

/*===| 初始化PWM TIM1 TIM8 |===*/
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}


/*===| 设置各个通道的占空比 |===*/
void PWM_SetEmpty(uint8_t Channel, float Empty)
{
	switch(Channel)
	{
		case 1: __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Empty * PWM_TIM_ARR); break;
		case 2: __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Empty * PWM_TIM_ARR); break;
		case 3: __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Empty * PWM_TIM_ARR); break;
		case 4: __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, Empty * PWM_TIM_ARR); break;
	}
}

/*===| 设置舵机角度 |===*/
void PWM_Set_Servo_Angle(uint8_t Channel, float Angle)
{
	switch(Channel)
	{
		case 1: __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_TIM_ARR/40 + PWM_TIM_ARR/10 * Angle); break;
		case 2: __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PWM_TIM_ARR/40 + PWM_TIM_ARR/10 * Angle); break;
		case 3: __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, PWM_TIM_ARR/40 + PWM_TIM_ARR/10 * Angle); break;
		case 4: __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, PWM_TIM_ARR/40 + PWM_TIM_ARR/10 * Angle); break;
	}
}

