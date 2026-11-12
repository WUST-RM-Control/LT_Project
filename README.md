# LT_Project
轮腿上下板代码

## 本次提交说明

> 云台 Yaw 轴改用达妙电机（MIT 控制）并重调云台 PID；轮腿底盘调参、修达妙反馈解析、加入"倒着走"方向判定、接入 VOFA。

### UP（云台上板）

**Yaw 轴改用达妙 DM 电机**
- 通信协议换成达妙：`Gimbal_Yaw_Send_CAN_ID` `0x1FF` → `0x05`，`Gimbal_Yaw_Feedback_CAN_ID` `0x206` → `0x105`；反馈解析由 `Motor_DJI_Storage_Data` 改为 `Motor_DM_Storage_Data`。
- 控制量由 DJI 电流帧改为 `Motor_DM_CMD_MIT`，输出限幅 `25000` → `16384`；电机补偿与 PID 输出统一走 MIT 力矩。
- 达妙驱动修正（`Motor_DAMIAO_Driver.c/h`）：温度取 `Data[7]`（原取 `Data[6]`，实际是错误码字节）、错误码改取 `Data[0] >> 4`、速度换算系数 `60` → `360`；力矩映射范围 `T_MIN/T_MAX` 由 `±5` 改为 `±16384`。
- 云台零点重标：`Gimbal_Median_Yaw` `-50.7568359f` → `-173.995865f`。

**云台控制**
- Yaw 误差改用 `Caculate_Included_Angle(Motor.Yaw.Total_Angle, Gimbal_Median_Yaw)`，替换原来手写的 ±180° 折返；倒地自起时不再单独用"锁 Yaw"环，与正常模式共用一套 PID（目标 0、反馈取就近 ±180° 方向），删除 `Yaw_Angle_Lock_PID_Struct` / `Yaw_Speed_Lock_PID_Struct` 与两套 `Aim` PID 成员。
- PID 重调：Pitch 角度环 `10 → 25`（积分限幅 `100`）、Pitch 速度环 `200 → 100`（积分上限 `500 → 2000`）；Yaw 角度环 `25 → 20`（D `30 → 750`）、Yaw 速度环 `350 → 25`（P `1.0 → 80`，输出限幅 `25000 → 16384`）。
- **修 bug**：Pitch 速度环反馈 `INS_Data_Internal.Gyro[1]*PI_to_Degree/6.0f` 去掉 `/6.0f`——原来反馈被缩小 6 倍，等效把速度环增益放大了 6 倍。
- 暂时绕过齿槽/摩擦补偿：输出直接取 `Smooth_Start_K * PID`，注释掉 `Motor_Compensation_Get_Data()` 与"从 Flash 手动 memcpy 补偿表"的代码（标定数据仍保留在 `Config`）。

**其他**
- `Vofa.c/.h` 换成 JustFloat 实现并在 `main.h` 引入；`Gimbal_Task` 打出 0~18 通道（Yaw/Pitch 角度环与速度环的目标、反馈、输出 + IMU Roll/Pitch/Yaw 与三轴角速度）。
- 恢复 Yaw 电机掉线重连：`Ticker == 110` 时 `Motor_DM_CMD_ClearErr()` → `Motor_DM_CMD_Enable()`。
- 遥控使能条件由 `S1 != SW_Down` 改为 `S1 == SW_Up || S1 == SW_Mid`。
- 电机补偿任务：Pitch 补偿角范围 `9 / -43` → `-55 / -100`，Yaw 补偿 PID 参数重调，输出改 MIT。

### Down（轮腿底盘）

**遥控与运控**
- 新增 `RoboControl_Struct.Target_Direction_Flag`：云台相对底盘超过 ±90° 时判定为"倒着走"，`Target_Direction_Err` 取 ±180° 就近方向，`Robo_Target_Vy` 随符号取反；`RoboControl.h` 补充字段与注释。
- 腿长调节速度 `0.16f` → `0.4f` m/s。
- S2 下拨由直接失能 `Chassis_OFF` 改为缓失能 `Chassis_OFF_Slow`。
- 通信任务周期 `5 ms` → `1 / 2 / 2 ms` 分段发送；`Chassis_Send_Func_Data()` 增加 `Chassis_OFF` 状态下也下发。
- `Leg_Length_Max` `0.38f` → `0.366f`。

**轮腿**
- 缓倒地自起二段：摆角速度 `90 → 100` °/s、收腿速度 `0.5 → 0.4` m/s、目标总摆角 `±9° → 0°`；摆腿的 `Acc_and_Dec()` 移入 `else` 分支，并增加"总摆角已接近 90°"时提前进入收腿判据；状态切换延时 `0.1 → 0.05` s。
- 跳跃：蹬伸腿长 `0.20/0.25/0.33` → `0.30/0.33/0.35` m，蹬腿前馈 `330 → 360` N、斜率 `25 → 15`。
- LQR 增益拟合表 `Poly_Coefficient` 全部重新拟合（12×4 系数更新）。

**底层（达妙电机反馈解析）**
- 错误码取 `Data[0] >> 4`（原 `Data[0] & 0x0F` 被当作错误码）、温度取 `Data[7]`、1 拖 4 收包补 `Error_ID = Data[7]`；只有 `Error_ID == 1` 才清掉线计数并置在线，`Date_New` 改为每帧置位。
- `Define.h` 删除遗留的 LQR 单字母下标宏（`S/S_D/YAW/...`）与旧电机 ID 宏；`PID.c` 缩进整理；`Chassis.c` 设零点完成提示音改 `SystemStart`。

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
