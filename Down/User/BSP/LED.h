#ifndef __LED__
#define __LED__

#include "main.h"

/* ==========================================================================
 *  WS2812B-3528 RGB 灯珠驱动（CtrBoard-H7_V1.0 板，位号 LED3）
 *
 *  硬件连接（摘自原理图 CtrBoard-H7_V1.0-240124.pdf）
 *  ---------------------------------------------------
 *      LED3 DIN   <---  SPI6_MOSI (PA7)   网络标号 "SPI6_MOSI(PA7)"
 *      LED3 VCC   <---  VCC_5VS           （由 PC15 即 POWER_5V 使能 Q5/Q6 开关管得到）
 *      LED3 GND   <---  GND
 *      LED3 DOUT  --->  悬空（板上只有这一颗灯珠）
 *
 *  数据位流由 SPI6 产生（主机、双线仅发送、高位先出）。
 *  这一套参数与官方例程 CtrBoard-H7_WS2812 完全一致：
 *  SPI6 内核时钟 = HSE = 24 MHz，分频 4 得 6 Mbit/s，采样点在第 2 个边沿：
 *
 *      1 个 WS2812 位 = 1 个 SPI 字节 = 1.333 us（手册 1.25 us ± 0.6 us）
 *      '0' = 0x60：T0H = 333 ns，T0L = 1000 ns
 *      '1' = 0x78：T1H = 667 ns，T1L = 667 ns
 *
 *  一颗灯珠 = 24 位 = 24 字节，线上颜色顺序为 G-R-B。
 *
 *  用法（与 Buzzer 模块完全相同的风格）：
 *      LED_Init();                     // 上电初始化一次
 *      LED_Set_Effect(...);            // 随时切换自动灯效
 *      for (;;) {
 *          LED_Task_Loop();            // 每个周期调用一次，负责把颜色发出去
 *          osDelay(LED_TASK_CONTROL_TIME);
 *      }
 *
 *  也可以直接给颜色：LED_Set_Color(255, 0, 0); 它会把灯效自动切成"自定义"。
 *  全部运算都是整数移位与乘法，没有浮点，也没有除法。
 * ========================================================================== */

/* 整条灯链上 WS2812 的颗数。这块板只有一颗 LED3，
 * 以后外接灯带把它改大即可。 */
#define WS2812_LED_NUM          1

/* 驱动数据线用的 SPI 句柄 */
#define WS2812_SPI              hspi6

/* WS2812 位码：一个 WS2812 位用一个字节表示（与官方例程一致）
 *   '0' = 0x60 = 0110 0000，'1' = 0x78 = 0111 1000 */
#define WS2812_CODE_0           0x60u
#define WS2812_CODE_1           0x78u

/* 每帧末尾补的低电平字节数（复位 / 锁存时间）：
 *   300 字节 = 300 * 8 / 6 MHz = 400 us
 * 官方例程这里只补 100 字节（约 133 us）也能亮，但老版 WS2812B 只要 >50 us、
 * V5 及部分国产替代片要求 >280 us，取 400 us 两种都稳。
 * （以前按 7.5 MHz 算成 51 us，正好卡在边界上，表现就是灯只锁存过一次、
 *  之后一直保持同一个颜色不变，也就是那次"一上电就一直绿"。） */
#define WS2812_RESET_BYTES      300u

/* 一颗灯珠占用多少字节 */
#define WS2812_BYTES_PER_LED    24u

/* ===== 颜色通道顺序 =====
 * 灯珠内部接收 24 bit 时按"固定顺序"分配到三个颜色，顺序不同的芯片
 * 拿到的颜色就会串位（典型现象：程序里写红色，实际亮成绿色）。
 * 官方例程 CtrBoard-H7_WS2812 发的是 G-R-B（先发 G 的 8 bit），
 * 这里保持一致，用默认的 WS2812_ORDER_GRB。
 *   万一你的灯珠是 R-G-B 的批次（写红色亮绿色），把下面这行改成
 *   WS2812_ORDER_RGB 即可，其它代码都不用动。 */
#define WS2812_ORDER_RGB        1
#define WS2812_ORDER_GRB        2
#define WS2812_COLOR_ORDER      WS2812_ORDER_GRB

/* 一帧总字节数（数据 + 复位低电平） */
#define WS2812_FRAME_BYTES      (WS2812_LED_NUM * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES)



/* ===== 闪烁节奏（单位：拍，1 拍 = LED_TASK_CONTROL_TIME = 10 ms） ===== */
#define LED_RED_FAST_TICKS      5u      /* 红灯快闪：亮 50 ms / 灭 50 ms */
#define LED_GREEN_SLOW_TICKS    50u     /* 绿灯慢闪：亮 500 ms / 灭 500 ms */

/* 彩色灯每拍色相前进的步长（色相一圈 256 拍）。
 *   2 => 约 1.28 s 转完一圈，变化比较明显
 *   1 => 约 2.56 s 一圈，更柔和 */
#define LED_HUE_STEP            1u

/* 全局亮度上限，取值 0 ~ 255。
 *   255 = 满亮度
 *   100 = 每个通道的最终输出最大只有 100（当前值，适合当指示灯，但近距离刺眼）
 *    20 = 适合近距离，不刺眼
 * 颜色仍然照常设置，只在送到灯珠之前按比例压缩，
 * 所以调这个宏不影响任何按键/效果逻辑。 */
// uint8_t LED_MAX_BRIGHTNESS=10u;
#define LED_MAX_BRIGHTNESS  20u

/*===| 自动灯效枚举定义 |===*/
typedef enum
{
    LED_Effect_OFF = 0,         /* 无灯光 */
    LED_Effect_Colorful,        /* 彩色灯：红绿蓝连续渐变转圈 */
    LED_Effect_RedFastBlink,    /* 红灯快闪 */
    LED_Effect_GreenSlowBlink,  /* 绿灯慢闪 */
    LED_Effect_Custom,          /* 自定义：显示 LED_Set_Color() 设定的颜色 */
} LED_Effect_EnumTypedef;


/* 初始化：清掉灯珠里残留的颜色，并做一次 红->绿->蓝 自检（约 0.45 s），
 * 结束后默认灯效为 LED_Effect_OFF。 */
void LED_Init(void);

/* 设置自动灯效（彩色灯 / 红灯快闪 / 无灯光 / 绿灯慢闪 / 自定义）。
 * 切到 LED_Effect_Custom 时显示最近一次 LED_Set_Color() 设定的颜色。 */
void LED_Set_Effect(LED_Effect_EnumTypedef Effect);

/* 设置自定义颜色（R/G/B 各 0~255）。
 * 注意：本函数会自动把灯效切成 LED_Effect_Custom，
 * 所以调用它之后灯会直接显示这个颜色，不用再点灯效。 */
void LED_Set_Color(uint8_t R, uint8_t G, uint8_t B);

/* 灯光驱动函数，必须每 LED_TASK_CONTROL_TIME(10ms) 调用一次。
 * 它按当前灯效算出这一拍该显示的颜色，然后通过 SPI6 发给灯珠：
 *   - 彩色灯：色相每拍前进 LED_HUE_STEP
 *   - 快闪/慢闪：按计拍数在亮、灭之间切换
 *   - 无灯光：每拍都发全灭，保证灯一定是灭的
 *   - 自定义：每拍都发同样的颜色（持续刷新，抗干扰）
 * 一帧 324 字节 @6 Mbit/s 约 432 us，占 10 ms 周期的 4%，对这块板可以忽略。 */
void LED_Task_Loop(void);

#endif
