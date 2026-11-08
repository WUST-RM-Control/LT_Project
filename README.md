# LT_Project
轮腿上下板代码

## 本次提交说明

> 轮腿下位机由 G474 换成 DM_MC02（STM32H723VGTx，CtrBoard-H7 板），云台上板做云台标定与历史 bug 修复。

### Down（轮腿底盘：G474 → DM_MC02/H723）

**工程结构**
- 整块换成 CubeMX 的 `Core / Drivers / MDK-ARM / Middlewares / USB_DEVICE / User` 结构（`Kawashiro_MC02.ioc`），并附 CtrBoard-H7 原理图与管脚标注图。
- 删掉 G474 专有的 `dac`、`opamp`、`crc`、`system_stm32g4xx*`、`stm32g4xx_it`，新增 `bdma`、`octospi`（W25Q64）、`stm32h7xx_it`、H7 HAL 配置。
- 上层代码按 `User/Task`、`User/WheelLeg`、`User/Algorithm`、`User/BSP`、`User/Commnuicate`、`User/Hardware` 重新分层。
- `Gimbal.c`、`Ref.c` 移出工程（MC02 只跑轮腿底盘），`freertos.c` 的线程表相应删减；`Shoot_Update_VirHeat` 保留桩函数（`Referee_Unpack.c` 仍在调用）。

**底层适配（换板必需，逐项报备）**
- `Motor_Driver.h` 结构体做超集合并：追加 `Wheel_Motor1/2`、`Joint1~4`、`Shoot_Fric_First_Left/Right` 与 `Error_ID`、`Date_New` 字段，保留原云台/麦轮成员避免未迁出模块编译失败；`Motor_Driver.c` 新增 `Get_TotalAngle_Speed()`。
- `CAN_Driver.c` 收包分派按新电机表重写：DJI 3508 轮电机、DM MIT 关节电机 1~4、超电、IMU 上板帧、双板通信帧，掉线计数 `CAN1/2/3_Err_Ticker`。
- 算法层换用 Down 版本：`Power_Limit`、`PID`、`Function`、`user_lib`、`kalman_filter`、`Vofa`、`controller`、`CRC8_CRC16`；保留 `IMU_Cal`、`QuaternionEKF`、`Motor_Compensation`（并跟着 `kalman_filter.h` 的 `mat S → S1` 改名同步修改 `QuaternionEKF.c`）。
- `Define.h`：注释掉与 `kalman_filter.h` 的 `mat S` 冲突的 LQR 状态下标宏，补 `Rad2Angle`，加 `INS_Data_*`、`Robo` 兼容别名。

**新增 `INS_Chassis_Data` 中间层**
- 极性/轴映射与装配误差角的唯一改动点，底盘侧只读这一层，INS 模块零改动；装配误差用三个常值偏置宏（Pitch/Roll/Yaw），避免为 3° 级偏差引入整旋转。

**LED**
- WS2812 由 SPI 阻塞发送改为 **BDMA** 发送，帧缓冲搬到 SRAM4（`0x38000000`）——BDMA 属 D3 域，够不到 DTCM/AXI SRAM；发送用忙判断，不覆盖正在搬运的缓冲。

**IMU_Cal**
- 修 HardFault：两个结构体用 `__PACKED_STRUCT` 导致 `IMU_Cal_Data` 落在奇数地址，取 packed 成员地址交给 `float*` 时编译器生成对齐浮点访问，Cortex-M7 触发 UNALIGNED UsageFault；改回普通 `struct`（该结构体从不整块序列化）。

**氮气弹簧标定**
- `WheelLeg_SpringComp_Flash_Save()` 落盘到 `Config`（W25Q64）：头帧 `Magic/Version/Size` + 每腿 `CRC32` + `Valid` 位；上电校验不过（头帧不符、CRC 错、数据非有限值）则该腿 `GasSpring_Output = 0`，函数返回 `-1`。

**腿部姿态 EKF**
- 快速翻转后 Roll 出偏置的根因是陀螺（35.3 rad/s > ±2000 dps 量程）与加速度计（10.5 g > ±6 g 量程）双饱和，姿态一次性丢几十度后卡方检验整帧拒测、只剩陀螺积分。
- 参数与逻辑改造：加速度计 10 ms 低通（P0）、`R 1e7→1e6`、`Q2 0.001→0.005`（P2）；卡方门限改由"允许的重力方向残差（5.7°）"反推、取消整帧拒测、自适应增益夹到 `[0.2, 1]`（P1），避免原公式算出负增益导致反向修正。

### UP（云台上板）

- **修历史 bug**：`Gimbal_Task` 里 Yaw 误差折回 `[-180, 180]` 的公式写反，`360 - Err` / `360 + Err` 在误差超过 ±180° 时会得到同号的大角度而不是折到另一侧，云台会朝反方向绕大圈；改为 `Err - 360` / `Err + 360`。
- **云台零点重标**：`Gimbal_Median_Yaw` `63.32f` → `-50.7568359f`。
- **云台 Pitch 限位调整**：`Pitch_Angle_Limit_Max/Min` 由 `-40 / 7` 改为 `-100 / -55`。
- **射击射频上限**：`Shoot_Frequency_Max` `23` → `20`。
- **侧滑轮长按连发**：`RemoteControl_Bool` 新增 DWT 计时，侧滑轮压到底持续 1 s 进入 `Shoot_State_Continue`，松开恢复单发。

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
