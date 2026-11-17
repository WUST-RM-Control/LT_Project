# LT_Project
轮腿上下板代码

## 本次提交说明

> CAN 发送去掉阻塞自旋改成"满了就丢帧"、VOFA/Tof 改 DMA 与中断解包；底盘功能标志位加"一键清零"并改为边沿触发；Roll 由单环改双环、腿长速度环加 P、LQR 系数重拟合与跳跃/自起判据细化。UP 板彻底移除麦轮底盘逻辑，修 packed 结构体的非对齐 HardFault，云台零点重标。

### Down（轮腿底盘 / STM32H723）

**通信与调试**
- `CAN_Send_Data_STD()` / `CAN_Send_Data_EXD()`：删掉"FIFO 满就空转 1000 次"的阻塞等待（总线 bus-off、电机断电时每帧白转约 30 µs，底盘 1 kHz × 5 帧/周期会把控制周期顶长），改为 `if(HAL_FDCAN_GetTxFifoFreeLevel(...) != 0)` 才入队——发不出去就丢帧，控制周期不再被通信拖住。
- `VOFA_JustFloat_Send()`：`HAL_UART_Transmit(..., 10)` 阻塞发送 → `HAL_UART_Transmit_DMA()`。
- `USART_Driver.c`：`huart10` 的 ReceiveToIdle 回调里补上 `Tof_Date_Unpack(UART10_RX_Data)`，Tof 测距数据真正开始解包。
- `Chassis_Task` 打开 VOFA 通道 0~4：底盘状态、Roll、Pitch、Yaw、`Chassis_Dt`；`WheelLeg_Output.c` 里留下一组常用的力矩/腿长通道注释，按需打开。

**底盘状态机**
- 新增 `WheelLeg_Control_Flag.Chassis_Flag_Reset`（`Chassis.h`）。`Get_Control_Data()` 里检测到它置 1，就一次性把 `Chassis_Flag_Reset / Chassis_Jump_FLag / Chassis_RESET_Slow_FLag / Off_ground_Flag / Up_Step_Flag` 全部清零——侧拨轮打到 `RC_Side < -0.7f` 模式下、左摇杆向左的边沿触发，用来"避免疯车"。
- 三个功能标志由**电平触发改为边沿触发**：`Chassis_RESET_Slow_FLag && 本地标志 == 0`、`Chassis_Jump_FLag && 本地 == 0`、`Up_Step_Flag && 本地 == 0`。原来遥控器按住不放会每周期重新触发，现在只在 0→1 的那一次生效。
- 跳跃只在 `Chassis_State == Chassis_FOLLOW` 时允许进入（原来任何状态都能被右摇杆向右打断）；右摇杆向左的上台阶标志位、右摇杆向下设零点都新增/保留在同一 `RC_Side` 模式分支里。
- 设零点触发位置调整：左摇杆向上的旧入口注释掉，改到**右摇杆向下**的边沿置 `RoboControl_Struct.If_SettingZero = 1`。
- `Robo_Stop()`：缓失能 `Chassis_OFF_Slow` → 立即失能 `Chassis_OFF`。
- `Roll_dot` 直接取 `INS_Chassis_Data.Roll_Speed`，不再对 Roll 做差分（1 kHz 下差分噪声太大）。

**轮腿控制**
- 腿长速度环 `P 100 → 120`。
- Roll 轴由单环改双环：`Leg_Roll_PID` 由 `P55 / D180 / 限幅80` 改为 `P15 / D500 / 限幅50`，并把原本注释掉的 `Leg_Roll_Speed_PID`（`P2.0 / D1.0 / 限幅80`）恢复成串联——角度环出目标、角速度环出补偿力。
- `WheelLeg_LQR.c`：
  - `Poly_Coefficient` 由普通全局改成 `static const float`，12×4 系数整表重拟合。
  - 跳跃标志 `Chassis_Jump_FLag == 1 / == 2` 单独成一个分支：轮力矩 = LQR 轮力矩 + `Wheel_Wz_Speed_PID.Output`，关节力矩补上 `xb / phi` 两项（原来跳起来只有摆角项，位移与俯仰不受控）。
  - 其它状态（含离地）的关节力矩补上 `K[10]*phi + K[11]*phi_d`。
- `WheelLeg_Chassis.c`：
  - 缓倒地自起二段完成判据补 `Leg_Length_Target - Leg_Length_Min < 0.01`——必须真的收到最短才算完成，避免"腿长目标还没到位就切状态"。
  - `Chassis_RESET_Slow_FLag_Return == 1` → `!= 0`。
  - 跳跃里补 `Off_ground_Flag = 1`，跳起来之后不再让离地检测改腿长。
  - 收腿行程按 200:1 的比例重标注释，并标注"激进参数，请确保电池电量在一格以上或接缓冲电容"。
- `WheelLeg_Output_Motor_Send_Current()` 由 `static` 提升为普通函数并在 `.h` 中声明，补 doxygen（灯条一侧为车尾；Joint1/2 左前/左后，Joint3/4 右前/右后）。

**宏与命名统一（`Define.h`）**
- 角度/弧度换算宏去重：只保留 `Angle2Radain` / `Radain2Angle`，并新增 double 版本 `Angle2Radain_double` / `Radain2Angle_double`；删除 `Angle_to_Radain` / `Radain_to_Angle` / `Rad2Angle` 三种旧写法。
- `WheelLeg_Kinematics.c`、`WheelLeg_SpringComp.c`（其内部重复定义一并删除）、`INS.c` 全部改用新宏名。

**安全**
- `INS.h`：`INS_HEAT_ENABLE` `1 → 0`，并在注释里写明"正常测试一定要关，Debug 断点在断点期间会让加热持续输出，喵板会烧"。

### UP（云台上板 / STM32G474）

- **彻底移除麦轮底盘**：`Chassis_Task` 里里程计积分、缓启动、按云台 Yaw 偏移的全向解算、麦轮运动学与四路 PID、功率限制、电流下发全部注释掉，`osDelay(1) → osDelay(10)`。上板现在只做云台。
- **修 packed 结构体的非对齐访问**：`Motor_Driver.h` 的 `typedef __PACKED_STRUCT` 改为 `typedef struct`——与之前 `IMU_Cal.h` 同一类问题（packed 成员取地址传给 `float *`，Cortex-M7 会报 UNALIGNED UsageFault → HardFault）。`Communicate.h` 6 处 `__packed struct` 统一改为 `__PACKED_STRUCT`。
- 云台零点重标：`Gimbal_Median_Yaw` `-173.995865f → 94.5195465f`。
- `Vofa.c` 同 Down 改 `HAL_UART_Transmit_DMA()`，`ErrorHandle.c` 打开 `VOFA_JustFloat_Send(&huart4)`。
- `RoboControl.c`：S2 下拨（`SW_Down`）也置 `Shoot_State_Off`，与中位一并处理。

### 工程文件
- FreeRTOS 的 `port.c` / `portmacro.h` 由 `Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/` 移到 `Down/Middlewares/` 根下（纯重命名，`R100`），`Kawashiro_MC02.uvprojx` 同步更新引用路径。
- 同步暂存了重新编译产生的 `Kawashiro_MC02.hex`、`Kawashiro_Frame_G474.hex`、`*.uvoptx`、`*.uvguix.*`、`.vscode/*`、`*.dep`。

## Zzz_test 分支提交记录

按时间顺序排列。

| 时间 | 提交消息 |
| --- | --- |
| 2026-09-08 00:52 | Initial commit |
| 2026-09-08 21:45 | init |
| 2026-09-25 20:56 | test |
| 2026-09-25 22:09 | DT7_军训招新用 |
| 2026-10-04 13:01 | 完成氮气弹簧移植 |
| 2026-10-06 03:20 | 优化后的氮气弹簧(加入摩擦力的补偿，抛弃末端15个脏数据点，并在超出末端的部分拟合，还有一些细枝末节的优化) |
| 2026-10-07 04:52 | 完成WheelLeg_Output封装 |
| 2026-10-09 17:58 | 完成底盘的封装 |
| 2026-10-13 04:37 | No_Error_No_Warning等待上车测试和任务函数的完善 |
| 2026-10-16 03:10 | 完成上车测试，LQR运控无问题<br>加入了Tof测距解包<br>优化了翻倒自起(最终版) |
| 2026-10-16 03:28 | 加了翻倒自起的堵转处理，现在是最终版翻倒自起 |
| 2026-11-02 00:28 | 补交翻倒自起的堵转处理<br>修改关节电机4的fdcan<br>加入跳跃的逻辑<br>加入离地检测，上台阶函数<br>注释腿部数据卡尔曼滤波，暂时改为新数据更新腿部数据<br>上台阶函数待改BUG，存在上台阶后未及时退出上台阶状态情况 |
| 2026-11-02 00:28 | 删除编译文件，并修改了README.md<br>优化了氮气弹簧补偿的一个bug<br>修正了腿部数据的一个逻辑错误<br>改了倒地自起的参数，让机体更快自起 |
| 2026-11-09 06:33 | 轮腿下位机由 G474 换成 DM_MC02(H723)<br>上层代码重新分层并迁移轮腿底盘<br>加入了更好的陀螺仪驱动<br>氮气弹簧标定落盘并加头帧校验<br>腿部 EKF 抗饱和与卡方逻辑改造<br>UP 修 Yaw_Err的赋值 bug，重标云台零点与 Pitch 限位 |
| 2026-11-13 01:29 | 云台Yaw轴改用达妙电机(MIT控制)并重调云台PID<br>修Pitch速度环反馈被除以6的bug<br>轮腿底盘新增"倒着走"方向判定，腿长调节与缓失能调整<br>修正达妙电机反馈解析(错误码/温度字段)<br>跳跃与倒地自起调参、LQR增益表重拟合<br>UP/Down接入VOFA JustFloat调试通道 |
| 2026-11-17 14:23 | CAN发送改为满则丢帧的非阻塞方式，VOFA改DMA，Tof测距加入解包<br>底盘标志位新增一键清零，跳跃/缓起/上台阶改边沿触发，跳跃限定FOLLOW<br>新增Roll角速度并取消差分，Roll加角速度环串联，腿长速度环加P<br>LQR增益表重拟合并整理跳跃/离地分支，倒地自起与跳跃判据细化<br>角度换算宏统一为Angle2Radain/Radain2Angle，INS加热默认关闭<br>UP移除麦轮底盘，修packed结构体非对齐HardFault，重标云台零点 |
