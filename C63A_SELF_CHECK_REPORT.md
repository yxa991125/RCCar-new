# C63A 侧自查报告

日期: 2026-04-13

## 1. 问题摘要

实车测试时出现两个直接现象:

- ROS 侧 `/odom` 话题存在且持续发布，但 `x/y` 和 `vx/vy/wz` 一直为 `0`
- `slam_toolbox` 收到激光后，地图基本停留在第一帧，不随底盘运动扩展

这次排查的重点是确认问题究竟出在:

- ROS 侧串口解析
- ROS 侧里程计发布
- 还是 C63A 下位机回传的速度数据本身

## 2. 已确认事实

### 2.1 ROS 侧串口协议正常

重新接线后，`/dev/ttyACM0` 上观测到的是干净的 `24` 字节基础帧流，符合 ROS 侧当前解析假设。

实测结果:

- 连续抓包 12 秒
- 合法帧数: `241`
- 频率约: `20 Hz`
- BCC 校验失败数: `0`
- 帧头/帧尾正常: `0x7B ... 0x7D`

结论:

- 串口链路是通的
- 协议层没有坏
- 不是“完全没回包”

### 2.2 ROS 侧 `/odom` 不是没发布，而是值始终为 0

ROS 侧 [autoracer_robot.cpp](/home/car/CodeWisdom-AutoRacer/src/turn_on_autoracer_robot/src/autoracer_robot.cpp#L128) 会把 `Robot_Vel.X/Y/Z` 直接发到 `/odom`。

ROS 侧 [autoracer_robot.cpp](/home/car/CodeWisdom-AutoRacer/src/turn_on_autoracer_robot/src/autoracer_robot.cpp#L213) 解析 `24` 字节基础帧时，`Byte2..7` 会被直接转成:

- `Robot_Vel.X`
- `Robot_Vel.Y`
- `Robot_Vel.Z`

然后在 [autoracer_robot.cpp](/home/car/CodeWisdom-AutoRacer/src/turn_on_autoracer_robot/src/autoracer_robot.cpp#L299) 中积分成位置并发布 `/odom`。

这说明:

- 只要 C63A 发出的 `vx/vy/wz` 非零，ROS 侧就会直接反映到 `/odom`
- 当前 `/odom` 持续为 0，说明 ROS 收到的速度字段本身就是 0

### 2.3 EKF 依赖 `/odom`

当前 EKF 配置在 [ekf.yaml](/home/car/CodeWisdom-AutoRacer/src/turn_on_autoracer_robot/config/ekf.yaml#L33) 明确使用 `/odom` 的 `vx/vy/vyaw` 作为轮式里程计输入。

因此一旦 `/odom` 为 0:

- `/odom_combined` 只能剩下 IMU 姿态变化
- 不会产生有效平移
- 2D SLAM 看起来就会像“车没动”

## 3. 当前最重要的定位结论

在确认车辆已经被遥控运动的前提下，本次排查已经可以把问题边界收窄到 C63A 侧。

原因是:

1. 串口帧合法
2. BCC 校验通过
3. ROS 能持续收到帧
4. 但原始串口帧里的 `vx/vy/wz` 在整段抓包中始终为 0

也就是说，问题不是:

- ROS 把非零速度解析错了
- ROS 把非零速度发布丢了
- SLAM 自己把 odom 吃没了

而是:

- C63A 送上来的速度反馈字段本身没有更新

## 4. C63A 侧数据链路

参考固件里，速度反馈链路是下面这条:

1. 驱动器/编码器反馈通过 CAN 上报左右轮转速
2. CAN 回调中解析出左右轮 rpm
3. 写入 `MotorA/B/C/D/E/F.feedback`
4. 正运动学计算 `feedbackVx/feedbackVy/feedbackVz`
5. `RobotDataTransmitTask` 把 `feedbackVx/Vy/Vz` 打包进 UART4 的基础帧 `Byte2..7`
6. ROS 串口读到后发布 `/odom`

参考代码位置如下。

### 4.1 驱动器反馈进入 C63A

CAN2 FIFO1 回调在 [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L91)。

驱动器速度帧解析在:

- [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L123)
- [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L129)
- [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L135)

这里会把:

- `0x190`
- `0x191`
- `0x192`

解析成左右轮 rpm，并调用 `WheelSpeedCalculation(...)`。

### 4.2 左右轮速度写入电机反馈变量

[can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L160) 的 `WheelSpeedCalculation(...)` 会把 rpm 转成线速度，并写入:

- `RobotControlParam.MotorA.feedback`
- `RobotControlParam.MotorB.feedback`
- `RobotControlParam.MotorC.feedback`
- `RobotControlParam.MotorD.feedback`
- `RobotControlParam.MotorE.feedback`
- `RobotControlParam.MotorF.feedback`

如果这里始终没有收到非零 rpm，后面所有速度都会继续是 0。

### 4.3 正运动学汇总为车体速度

[RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L549) 的 `Robot_Forwardkinematics(...)` 负责把各轮反馈转换成:

- `feedbackVx`
- `feedbackVy`
- `feedbackVz`

该函数在主控制任务里被周期性调用，见 [RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L160)。

如果当前车型是 `S200_OUTDOOR`，计算公式是:

- `feedbackVx = (A + B + C + D) / 4`
- `feedbackVy = 0`
- `feedbackVz = (-A - B + C + D) / 4 / (WheelSpacing + AxleSpacing)`

具体位置在 [RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L560)。

### 4.4 UART4 打包到 ROS

[data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L38) 的 `RobotDataTransmitTask(...)` 以 `20 Hz` 周期发送数据。

其中基础帧 `Byte2..7` 明确来自:

- `feedbackVx` -> `Byte2..3`
- `feedbackVy` -> `Byte4..5`
- `feedbackVz` -> `Byte6..7`

具体赋值见:

- [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L61)
- [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L62)
- [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L64)
- [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L66)

这意味着:

- 只要 `feedbackVx/Vz` 被算出来，UART4 基础帧就一定会带上非零值
- 当前串口抓包仍是全 0，说明问题出在打包之前

## 5. C63A 侧优先自查项

下面这些是建议按优先级执行的自查项。

### 5.1 先确认“车真的动了，但反馈仍为 0”

不要只看目标速度或遥控指令，要看“反馈值”。

建议在 C63A 侧同时观察:

- 电机是否实际转动
- 车轮是否实际转动
- `MotorA/B/C/D.feedback`
- `feedbackVx`
- `feedbackVz`
- UART4 发出的 `Byte2..7`

如果车确实动了，但上述反馈仍为 0，问题就在 C63A。

### 5.2 检查 CAN 驱动反馈是否真的收到

优先检查 [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L123) 到 [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L139)。

需要确认:

- 是否持续收到 `0x190/0x191/0x192`
- `tmpL/tmpR` 是否在车动时变成非零
- 轮速正负号是否合理
- `NodeId` 与当前车型的电机映射是否匹配

如果这里已经是 0，后面不用再怀疑 ROS。

### 5.3 检查电机反馈变量是否被正确写入

优先检查 [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L170) 到 [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L219)。

需要确认:

- 当前车型 `CarType` 是否正确
- 反馈是否写到了正确的 `Motor*.feedback`
- 是否存在某些轮子的 feedback 没有被更新
- 是否在别处又被覆盖回 0

特别注意:

- 如果车型配置不对，可能会导致反馈写到错误轮子变量
- 即使 CAN 帧来了，正运动学也可能因为使用了另一组轮子变量而算出 0

### 5.4 检查正运动学是否得到非零车体速度

优先检查 [RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L549)。

需要确认:

- `feedbackVx`
- `feedbackVy`
- `feedbackVz`

在车前进、后退、原地转向时是否符合预期。

最低验收标准:

- 前进时 `feedbackVx != 0`
- 后退时 `feedbackVx != 0`
- 原地转向时 `feedbackVz != 0`

### 5.5 检查 UART4 打包前的最终值

优先检查 [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L58) 到 [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L90)。

需要确认:

- `basebuffer[2..3]` 是否对应 `feedbackVx`
- `basebuffer[4..5]` 是否对应 `feedbackVy`
- `basebuffer[6..7]` 是否对应 `feedbackVz`
- 打包前这三个值是否已经非零

如果 `feedbackVx/Vz` 非零，但 `basebuffer[2..7]` 仍是 0，则是 UART4 打包代码问题。

### 5.6 检查驱动器在线状态和使能状态

[RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L403) 的 `robot_en_check()` 会根据:

- 驱动器在线状态
- 驱动器错误
- 急停/软件停
- 低压状态

决定 `en_flag`。

其中:

- 初始化默认 `en_flag = 1`，见 [robot_select_init.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/robot_select_init.c#L8)
- 若存在错误，`en_flag` 会被拉成 `0`，见 [RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L476)

这项虽然不直接决定 `feedbackVx` 是否打包，但会影响底盘是否真的运动。

建议同时确认:

- `ErrNum`
- `Enkeystate`
- `softwareEnflag`
- `g_ServoDriveList[i]->onlineFlag`
- `g_ServoDriveList[i]->errFlag`

## 6. 建议加入的临时日志

为避免盲查，建议 C63A 侧临时加 4 组日志。

### 日志点 1: CAN 轮速原始值

位置:

- [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L123)

建议打印:

- `StdId`
- `tmpL`
- `tmpR`

目的:

- 确认驱动器反馈是否真的到板子

### 日志点 2: 电机 feedback

位置:

- [can_callback.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/can_callback.c#L160)

建议打印:

- `CarType`
- `NodeId`
- `MotorA/B/C/D.feedback`

目的:

- 确认轮速是否写进了正确变量

### 日志点 3: 车体反馈速度

位置:

- [RobotControl_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/RobotControl_task.c#L549)

建议打印:

- `feedbackVx`
- `feedbackVy`
- `feedbackVz`

目的:

- 确认正运动学是否正常

### 日志点 4: UART4 最终发送值

位置:

- [data_task.c](/home/car/CodeWisdom-AutoRacer/reference/WHEELTEC_C63A/WHEELTEC_APP/data_task.c#L61)

建议打印:

- `feedbackVx`
- `feedbackVy`
- `feedbackVz`
- `basebuffer[2..7]`

目的:

- 确认发给 ROS 的最终字节是不是已经为 0

## 7. 建议的最短排查顺序

建议按下面顺序排查，不要跳步。

1. 在 C63A 上确认车实际运动时，`0x190/0x191/0x192` 是否到达
2. 确认 `tmpL/tmpR` 是否非零
3. 确认 `Motor*.feedback` 是否非零
4. 确认 `feedbackVx/Vz` 是否非零
5. 确认 UART4 `Byte2..7` 是否非零
6. 最后再回到 ROS 看 `/odom`

这条顺序的原则是:

- 先看最原始反馈
- 再看中间变量
- 最后看串口打包

## 8. 最终判断标准

C63A 侧修复完成后，应满足以下最小标准:

- 车前进时，UART4 基础帧 `Byte2..3` 非零
- 车后退时，UART4 基础帧 `Byte2..3` 非零且符号相反
- 原地转向时，UART4 基础帧 `Byte6..7` 非零
- ROS 侧 `/odom.twist.twist.linear.x` 跟随变化
- ROS 侧 `/odom.twist.twist.angular.z` 跟随变化
- ROS 侧 `/odom.pose.pose.position.x/y` 随运动累积

## 9. 一句话结论

本次排查已经证明:

- ROS 串口协议正常
- ROS 解析路径正常
- `/odom` 为 0 的根因是 UART4 基础帧里的速度反馈字段持续为 0

因此，C63A 侧当前最优先要查的是:

- 驱动器反馈是否进入板子
- 反馈是否正确写入 `Motor*.feedback`
- 正运动学是否正确生成 `feedbackVx/Vz`
- UART4 打包前是否已经是 0
