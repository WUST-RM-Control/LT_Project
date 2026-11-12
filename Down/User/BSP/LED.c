#include "LED.h"
#include "spi.h"

/* ==========================================================================
 *  WS2812B-3528 驱动（数据线 SPI6_MOSI / PA7，BDMA 发送）
 *
 *  一、为什么帧缓冲区要放在 SRAM4（0x3800_0000 / D3_SRAM_BASE）
 *  ------------------------------------------------------------------------
 *  spi.c 里给 hspi6 配的是 BDMA_Channel0。BDMA 挂在 D3 域，能访问的只有
 *  D3 的 SRAM4；而本工程散射文件把 RW 数据放在 DTCM(0x2000_0000) 和
 *  AXI SRAM(0x2400_0000)，这两块 BDMA 都够不着，一旦启动 DMA 就报
 *  HAL_DMA_ERROR_TE，一个字节都发不出去——上一版就是因为这个退回阻塞发送的。
 *  SRAM4 共 16 KB，整个工程没有别的模块用它（散射文件里也没有），
 *  取首地址当帧缓冲区，一帧 324 B 绰绰有余。
 *
 *  二、为什么不去改散射文件加一个 DMA 段
 *  ------------------------------------------------------------------------
 *  那属于底层构建配置（.sct），你要求尽量别碰下位机底层，所以这里用固定
 *  地址把缓冲区锁在 SRAM4 头部，改动全部收敛在 LED.c 一个文件里。
 *  ！如果以后有人把 SRAM4 也划进 scatter，需要同步调整 WS2812_BUFFER_ADDR。
 *
 *  三、时序与开销
 *  ------------------------------------------------------------------------
 *  SPI6 = 6 Mbit/s，WS2812 一位用一字节表示：24 B 数据 + 300 B 复位
 *  ≈ 432 us/帧。DMA 期间 CPU 不参与，10 ms 周期里只剩几十条整数指令。
 *  上一帧还没发完时本拍直接跳过（最多丢一拍，观感无影响，也不会死等）。
 *
 *  四、初始化里的自检
 *  ------------------------------------------------------------------------
 *  LED_Init() 用"发送 + 等发送完"的同步版本，保证红/绿/蓝三帧真的上灯，
 *  不会被下一帧覆盖掉——DMA 是异步的，这一步必须等。
 * ========================================================================== */

/* 一次发送的超时时间（一帧约 432 us，留足余量）；只在初始化同步发送时用到 */
#define LED_SPI_TIMEOUT         10u

/* WS2812 帧缓冲区地址：D3 域 SRAM4 起始地址，BDMA 可直接访问 */
#define WS2812_BUFFER_ADDR      0x38000000uL

/* 帧缓冲区：24 字节灯珠数据 + 复位低电平（放 SRAM4，不能用普通 static 数组，
   否则会被链接到 AXI SRAM/DTCM，BDMA 访问不到） */
static uint8_t *const WS2812_Buffer = (uint8_t *)WS2812_BUFFER_ADDR;

/* 最近一次 HAL_SPI_Transmit_DMA() 的返回值，0 = HAL_OK，仅用于调试观察 */
volatile uint32_t LED_Dbg_Status = 0u;

/* 当前灯效，上电默认无灯光 */
static LED_Effect_EnumTypedef LED_Effect = LED_Effect_OFF;

/* 自定义灯效用的颜色（LED_Set_Color 写入） */
static uint8_t LED_Custom_R = 0u;
static uint8_t LED_Custom_G = 0u;
static uint8_t LED_Custom_B = 0u;

/* 灯效动画状态 */
static uint8_t  LED_Hue       = 0u;     /* 彩色灯色相 0~255 */
static uint16_t LED_Blink_Cnt = 0u;     /* 闪烁计时（单位：拍） */


/* 把 r/g/b 转成灯珠要的位流（每 bit 一个字节），末尾补低电平。
 * 字节顺序由 LED.h 的 WS2812_COLOR_ORDER 决定：
 *   R-G-B 芯片：先发 R 的 8 bit，再 G，再 B
 *   G-R-B 芯片：先发 G 的 8 bit，再 R，再 B */
static void LED_Build_Frame(uint8_t r, uint8_t g, uint8_t b)
{
#if (WS2812_COLOR_ORDER == WS2812_ORDER_RGB)
    uint32_t bits = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
#else
    uint32_t bits = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
#endif
    uint8_t *p   = WS2812_Buffer;
    uint32_t i;

    for (i = 0u; i < 24u; i++)
    {
        *p++ = (bits & (0x800000uL >> i)) ? WS2812_CODE_1 : WS2812_CODE_0;
    }

    for (i = 0u; i < WS2812_RESET_BYTES; i++)
    {
        *p++ = 0x00u;
    }
}

/* 按 LED_MAX_BRIGHTNESS 压缩最终输出。
 * 用 (c * (MAX+1)) >> 8 实现 c * MAX / 255：
 * c = 255、MAX = 255 时得到 255；MAX = 50 时得到 50。
 * 同样只有一次乘法加一次移位，没有除法。 */
static uint8_t LED_Limit(uint8_t c)
{
    return (uint8_t)(((uint16_t)c * ((uint16_t)LED_MAX_BRIGHTNESS + 1u)) >> 8);
}

/* SPI6 是否还在发上一帧（HAL 状态机在 DMA 期间是 BUSY_TX） */
static uint8_t LED_Is_Busy(void)
{
    return (HAL_SPI_GetState(&WS2812_SPI) != HAL_SPI_STATE_READY) ? 1u : 0u;
}

/* 发一帧（DMA 启动后立即返回）。
 * 上一帧没发完就跳过本拍：既不会阻塞，也不会改写正在被 DMA 搬运的缓冲区
 * （帧缓冲区只有一份，必须等上一帧结束才能重建）。 */
static void LED_Send(uint8_t r, uint8_t g, uint8_t b)
{
    if (LED_Is_Busy())
    {
        return;
    }

    /* 亮度上限在最后一步统一施加，前面的颜色/灯效计算都不用关心它 */
    LED_Build_Frame(LED_Limit(r), LED_Limit(g), LED_Limit(b));

    LED_Dbg_Status = (uint32_t)HAL_SPI_Transmit_DMA(&WS2812_SPI, WS2812_Buffer,
                                                   (uint16_t)WS2812_FRAME_BYTES);
}

/* 发一帧并等它真的发完：只在 LED_Init() 的自检里用，
 * 因为后面紧跟着可能是不同颜色的一帧，异步发送会把上一帧顶掉。 */
static void LED_Send_Sync(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t Tick = HAL_GetTick();

    LED_Send(r, g, b);

    while (LED_Is_Busy())
    {
        if ((HAL_GetTick() - Tick) > LED_SPI_TIMEOUT)
        {
            break;      /* 超时就放弃等待，不把初始化卡死 */
        }
    }
}

/* 色相 0~255 -> RGB，8 个 32 度扇区，扇区内线性插值。
 * 只有乘法与移位，没有除法、没有浮点，适合 10 ms 周期的实时刷新。 */
static void LED_Hsv_To_Rgb(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t sector = (uint8_t)(hue >> 5);          /* 0 ~ 7 */
    uint8_t frac   = (uint8_t)(hue & 0x1Fu);       /* 0 ~ 31 */
    uint8_t up     = (uint8_t)(((uint16_t)frac * 255u) >> 5);   /* 0 -> 255 */
    uint8_t down   = (uint8_t)(255u - up);         /* 255 -> 0 */

    switch (sector)
    {
        case 0u: *r = 255u; *g = up;   *b = 0u;   break;   /* 红 -> 黄 */
        case 1u: *r = down; *g = 255u; *b = 0u;   break;   /* 黄 -> 绿 */
        case 2u: *r = 0u;   *g = 255u; *b = up;   break;   /* 绿 -> 青 */
        case 3u: *r = 0u;   *g = down; *b = 255u; break;   /* 青 -> 蓝 */
        case 4u: *r = up;   *g = 0u;   *b = 255u; break;   /* 蓝 -> 品红 */
        case 5u: *r = 255u; *g = 0u;   *b = down; break;   /* 品红 -> 红 */
        default: *r = 255u; *g = 0u;   *b = 0u;   break;
    }
}


void LED_Init(void)
{
    LED_Effect     = LED_Effect_OFF;
    LED_Custom_R   = 0u;
    LED_Custom_G   = 0u;
    LED_Custom_B   = 0u;
    LED_Hue        = 0u;
    LED_Blink_Cnt  = 0u;

    /* 先连发两帧全灭：清掉上电前灯珠里残留的颜色 */
    LED_Send_Sync(0u, 0u, 0u);
    LED_Send_Sync(0u, 0u, 0u);

    /* 上电自检：红 -> 绿 -> 蓝 各亮 150 ms。
     * 三段颜色都能看到，说明数据通道和三色通道都没问题；
     * 如果一直卡在某个颜色不变，就是复位时间不够，灯珠没有锁存新数据。 */
    LED_Send_Sync(255u, 0u, 0u);
    HAL_Delay(150u);
    LED_Send_Sync(0u, 255u, 0u);
    HAL_Delay(150u);
    LED_Send_Sync(0u, 0u, 255u);
    HAL_Delay(150u);
    LED_Send_Sync(0u, 0u, 0u);
}

void LED_Set_Effect(LED_Effect_EnumTypedef Effect)
{
    if (LED_Effect == Effect)
    {
        return;
    }

    LED_Effect    = Effect;
    LED_Hue       = 0u;     /* 从头开始渐变 */
    LED_Blink_Cnt = 0u;     /* 从头开始计时，切过来立刻就是亮的一拍 */
}

void LED_Set_Color(uint8_t R, uint8_t G, uint8_t B)
{
    LED_Custom_R = R;
    LED_Custom_G = G;
    LED_Custom_B = B;

    /* 要求：设置 RGB 的同时自动切到"自定义"灯效 */
    LED_Set_Effect(LED_Effect_Custom);
}

void LED_Task_Loop(void)
{
    uint8_t r = 0u;
    uint8_t g = 0u;
    uint8_t b = 0u;

    switch ((uint8_t)LED_Effect)
    {
        case LED_Effect_Colorful:
            /* 彩色灯：色相每拍前进 LED_HUE_STEP，256 拍转完一圈 */
            LED_Hsv_To_Rgb(LED_Hue, &r, &g, &b);
            LED_Hue = (uint8_t)(LED_Hue + LED_HUE_STEP);
            break;

        case LED_Effect_RedFastBlink:
            /* 红灯快闪：前 LED_RED_FAST_TICKS 拍亮，后同样拍数灭 */
            r = (LED_Blink_Cnt < LED_RED_FAST_TICKS) ? 255u : 0u;
            LED_Blink_Cnt++;
            if (LED_Blink_Cnt >= (LED_RED_FAST_TICKS * 2u))
            {
                LED_Blink_Cnt = 0u;
            }
            break;

        case LED_Effect_GreenSlowBlink:
            /* 绿灯慢闪：前 LED_GREEN_SLOW_TICKS 拍亮，后同样拍数灭 */
            g = (LED_Blink_Cnt < LED_GREEN_SLOW_TICKS) ? 255u : 0u;
            LED_Blink_Cnt++;
            if (LED_Blink_Cnt >= (LED_GREEN_SLOW_TICKS * 2u))
            {
                LED_Blink_Cnt = 0u;
            }
            break;

        case LED_Effect_Custom:
            /* 自定义：一直显示 LED_Set_Color() 设定的颜色 */
            r = LED_Custom_R;
            g = LED_Custom_G;
            b = LED_Custom_B;
            break;

        case LED_Effect_OFF:
        default:
            /* 无灯光：r/g/b 保持 0，每拍都发全灭，保证灯一定是灭的 */
            break;
    }

    LED_Send(r, g, b);
}
