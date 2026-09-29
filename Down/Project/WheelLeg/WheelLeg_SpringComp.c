#include "WheelLeg_SpringComp.h"

#include <math.h>
#include <string.h>


#define WHEELLEG_SPRING_MIN_LENGTH_M    0.128f
#define WHEELLEG_SPRING_MAX_LENGTH_M    0.348f

#define WHEELLEG_SPRING_LEFT_FLASH_ADDR     0x08072000UL
#define WHEELLEG_SPRING_RIGHT_FLASH_ADDR    0x08074000UL

#define WHEELLEG_SPRING_MAX_ABS_FORCE_N     500.0f

WheelLeg_SpringComp wheelLeg_springComp = {0};

//左右腿静态机械补偿表  当前全为0
static float spring_table_left[WHEELLEG_SPRING_TABLE_SIZE] = {0};
static float spring_table_right[WHEELLEG_SPRING_TABLE_SIZE] = {0};

static void WheelLeg_SpringComp_LoadTable(float *table,uint32_t flash_address)
{
    for(uint16_t i = 0;i < WHEELLEG_SPRING_TABLE_SIZE;i++)
    {
        memcpy(&table[i],(const void *)(flash_address + i * sizeof(float)),sizeof(float));
    }
}

//腿长转化为索引
static uint16_t WheelLeg_SpringComp_LengthToIndex(float length_m)
{
    uint16_t index;

    if(length_m < WHEELLEG_SPRING_MIN_LENGTH_M)
    {
        length_m =WHEELLEG_SPRING_MIN_LENGTH_M;
    }
    if(length_m > WHEELLEG_SPRING_MAX_LENGTH_M)
    {
        length_m =WHEELLEG_SPRING_MAX_LENGTH_M;
    }

    index =(uint16_t)(length_m*(float)WHEELLEG_SPRING_TABLE_SIZE);

    if(index >= WHEELLEG_SPRING_TABLE_SIZE)
    {
        index =WHEELLEG_SPRING_TABLE_SIZE - 1;
    }
    return index;
}

static uint8_t WheelLeg_SpringComp_CheckTable(const float *table)
{
    uint16_t valid_count = 0;

    //实际腿长工作区：0.128m ~ 0.348m  对应代码索引大约：131 ~ 356
    uint16_t min_index =WheelLeg_SpringComp_LengthToIndex(WHEELLEG_SPRING_MIN_LENGTH_M);

    uint16_t max_index =WheelLeg_SpringComp_LengthToIndex(WHEELLEG_SPRING_MAX_LENGTH_M);

    for(uint16_t i = min_index; i <= max_index; i++)
    {
        float force = table[i];

        if(!isfinite(force))
        {
            return 0;
        }
        // 明显不合理的巨大补偿力
        if(fabsf(force) >WHEELLEG_SPRING_MAX_ABS_FORCE_N)
        {
            return 0;
        }
        //防止整块 Flash 全是0
        if(fabsf(force) > 0.001f)
        {
            valid_count++;
        }
    }
    
    if(valid_count < 10)
    {
        return 0;
    }


    return 1;
}

//单腿查表函数
static void WheelLeg_SpringComp_UpdateLeg(WheelLeg_SpringComp_Leg *leg,float length_m,const float *table)
{
    uint16_t index;

    leg->length_m =length_m;

    //确定标定是否有效
    if(leg->valid == 0)
    {
        leg->force_n =0.0f;
        leg->index =0;
        return;
    }

    //输入腿长异常保护
    if(!isfinite(length_m))
    {
        leg->force_n =0.0f;
        leg->valid =0;
        return;
    }

    index =WheelLeg_SpringComp_LengthToIndex(length_m);
    leg->index =index;

    //读取当前腿长对应补偿力
    leg->force_n =table[index];

    //表内数据异常保护
    if(!isfinite(leg->force_n))
    {
        leg->force_n =0.0f;
        leg->valid =0;
    }
}

//初始化
void WheelLeg_SpringComp_Init(void)
{
    wheelLeg_springComp.left.length_m =0.0f;
    wheelLeg_springComp.right.length_m =0.0f;
    wheelLeg_springComp.left.force_n =0.0f;
    wheelLeg_springComp.right.force_n =0.0f;
    wheelLeg_springComp.left.index = 0;
    wheelLeg_springComp.right.index =0;


    //还未验证 静止补偿
    wheelLeg_springComp.left.valid =0;
    wheelLeg_springComp.right.valid =0;


    //从旧工程保存区域读取静态补偿表
     
    WheelLeg_SpringComp_LoadTable(spring_table_left,WHEELLEG_SPRING_LEFT_FLASH_ADDR);
    WheelLeg_SpringComp_LoadTable(spring_table_right, WHEELLEG_SPRING_RIGHT_FLASH_ADDR);

    // 数据检查通过后才允许启用
    wheelLeg_springComp.left.valid =WheelLeg_SpringComp_CheckTable( spring_table_left );
    wheelLeg_springComp.right.valid =WheelLeg_SpringComp_CheckTable(spring_table_right);
    
}

//更新双腿补偿
void WheelLeg_SpringComp_Update(float left_length_m,float right_length_m)
{
    WheelLeg_SpringComp_UpdateLeg(&wheelLeg_springComp.left,left_length_m,spring_table_left);
    WheelLeg_SpringComp_UpdateLeg(&wheelLeg_springComp.right,right_length_m,spring_table_right);

}