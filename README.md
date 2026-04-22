# WHEELTEC Ackermann PWM 控制说明

## 概述
当前工程已经收敛为单一 Ackermann PWM 控制平台，正式控制入口只有两条：

- `ROS 串口 -> Ackermann PWM`
- `RC -> Ackermann PWM / 抢占`

旧差速/多车型底盘控制、旧轮速反馈、旧 CAN 舵机协议已从正式控制主路径中移除，不再参与车辆实际运动控制与 ROS 速度回传。

## 当前正式控制链路
### 1. ROS 串口控制
- 入口文件：`WHEELTEC_APP/SerialControl_task.c`
- 输入协议：固定 11 字节速度帧
- 处理流程：
  - `HAL_UART_RxCpltCallback()` -> `g_xQueueROSserial`
  - `SerialControlTask()` 解析 `Vx/Vy/Vz`
  - `ServoBasic_UpdateFromOrin()` 映射为 ESC / Servo PWM

### 2. RC 接管控制
- 输入捕获：`TIM4`
- 引脚：
  - `PD12 -> TIM4_CH1`：RC throttle
  - `PD13 -> TIM4_CH2`：RC steering
  - `PD14 -> TIM4_CH3`：RC guard
- RC 仅在明显偏离中位时抢占自动驾驶，回中保持一段时间后自动恢复 autonomous。

### 3. PWM 输出
- 输出定时器：`TIM8`
- 引脚：
  - `PC6 -> TIM8_CH1`：ESC PWM
  - `PC7 -> TIM8_CH2`：Servo PWM
- `PC8/PC9` 当前不作为正式控制输出使用。

### 4. 霍尔轮速反馈（阶段一）
- 当前阶段新增双路霍尔轮速输入，仅用于替换 `ROS` 上行 `vx` 伪反馈，不参与 `ESC` 闭环控制。
- 接线规划：
  - `PE9`：`Hall A`
  - `PE11`：`Hall B`
- 计数方式固定为：
  - 保留当前 `A/B` 角色互换方案：`Hall B` 作为计数脚，`Hall A` 作为方向脚
  - 仅在 `Hall B` 的下降沿记录一次有效事件
  - 在该时刻读取 `Hall A` 电平判定方向
  - 当前默认 `HALL_COUNT_EVENTS_PER_REV = 1`
- 当前轮径按 `0.235 m` 计算，轮周长约 `0.738 m`
- 当前阶段不启用 `PID`，只做真实测速与反馈替换

#### 霍尔输入电气前提
- 双路霍尔信号均按 `3.3V` 上拉、低电平有效处理
- 霍尔传感器与主控必须共地
- `PE9/PE11` 输入电平不得超过 `3.3V`
- 若实测高电平可能高于 `3.3V`，必须先做电平转换或隔离
- `PA13/PA14` 为 `SWD` 下载调试口，不能占用
- `PC6/PC7` 已为正式 `ESC/Servo PWM` 输出，不能改作轮速输入

## Ackermann 几何参数
当前几何参数以 `EXTRINSICS.md` 为准：
- `wheelbase = 0.54 m`
- `track_width = 0.48 m`
- `wheel_radius = 0.11 m`
- `max_steering_angle = 0.393 rad`
- `base_link = rear axle center`

在代码中对应的运行时参数为：
- `g_orin_ackermann_wheelbase_mm`
- `g_orin_ackermann_track_width_mm`
- `g_orin_ackermann_wheel_radius_mm`
- `g_orin_ackermann_max_steering_millirad`


## 串口协议
### ROS -> STM32 下行控制帧
固定 11 字节：

```text
0x7B  CMD1  CMD2  XH  XL  YH  YL  ZH  ZL  BCC  0x7D
```

含义：
- `CMD1 = 0x00`：速度控制
- `CMD2 bit7`：`flag_stop`
- `X/Y`：单位 `0.001 m/s`
- `Z`：单位 `0.001 rad/s`
- `BCC`：前 9 字节异或

### STM32 -> ROS 上行反馈帧
固定 24 字节基础帧：

```text
0x7B flag vx_h vx_l vy_h vy_l wz_h wz_l ay_h ay_l -ax_h -ax_l az_h az_l gy_h gy_l -gx_h -gx_l gz_h gz_l bat_h bat_l bcc 0x7D
```

说明：
- `vx/vy/wz` 由 `ServoBasic_GetOrinFeedback()` 提供
- 阶段一启用霍尔轮速后，`vx` 优先使用真实轮速反馈
- `g_orin_feedback_scale` 只保留给旧的 PWM 估算反馈，不再作用于霍尔真轮速
- 默认值为 `1254`（按千分比），即 `1.254x`
- `battery` 单位为 mV
- 常规模式下 `UART4 -> ROS` 发送这 24 字节基础帧
- 不再向 ROS 这一路混发 `19` 字节超声波帧或 `8` 字节回充帧

### UART4 霍尔调试帧
为排查霍尔输入，当前默认额外在 `UART4` 基础帧后追加 1 帧固定 32 字节调试帧。
可在 Keil Watch 中把 `g_uart4_hall_debug_enable` 设为 `0` 关闭。

```text
0xEB hall_a hall_b dir speed_valid timeout_active cnt_3 cnt_2 cnt_1 cnt_0 period_3 period_2 period_1 period_0 fault_h fault_l speed_h speed_l irq_3 irq_2 irq_1 irq_0 cb_3 cb_2 cb_1 cb_0 edge_3 edge_2 edge_1 edge_0 bcc 0xED
```

说明：
- `hall_a/hall_b`：当前引脚低电平有效，`1` 表示当前为低电平
- `dir`：方向，`0x01` 为正向，`0xFF` 为反向，`0x00` 表示尚未形成有效方向
- `speed_valid`：霍尔速度当前是否有效
- `timeout_active`：霍尔速度当前是否超时
- `cnt_3..cnt_0`：`event_count_total`，32 位有符号计数，高字节在前
- `period_3..period_0`：`last_period_us`，单位 `us`，32 位无符号，高字节在前
- `fault_h/fault_l`：`fault_count` 低 16 位
- `speed_h/speed_l`：霍尔计算速度，单位 `mm/s`，16 位有符号，高字节在前
- `irq_3..irq_0`：当前计数脚对应 EXTI 处理函数的进入次数
- `cb_3..cb_0`：`HAL_GPIO_EXTI_Callback(HallB_Pin)` 命中次数
- `edge_3..edge_0`：被 `HallSpeed_OnCountEvent()` 接受为有效边沿的次数

## 核心接口
文件：`WHEELTEC_APP/Inc/servo_basic_control.h`

保留的正式接口：
- `ServoBasic_Init()`
- `ServoBasic_ProcessControl()`
- `ServoBasic_UpdateFromOrin(...)`
- `ServoBasic_GetState()`
- `ServoBasic_IsRcOverrideActive()`
- `ServoBasic_IsRcEmergencyActive()`
- `ServoBasic_GetOrinFeedback(...)`
- `ServoBasic_OutputEscPulse()`
- `ServoBasic_OutputServoPulse()`

已移出正式主路径：
- 旧差速/多车型底盘控制队列
- 旧 CAN 舵机协议入口
- 旧轮速反馈驱动 ROS 速度回传的逻辑

## RC 抢占与安全策略
- 默认控制模式：`AUTONOMOUS`
- RC 偏离中位超过阈值后自动进入 `RC_PASSTHROUGH`
- RC 回中并保持释放时间后恢复 `AUTONOMOUS`
- `PD14 guard` 可触发紧急覆盖
- 无有效自主目标且 RC 不可用时，输出退回：
  - `ESC neutral`
  - `Servo center`

## 常用调试变量
建议在 Keil Watch 中观察：
- `g_state.control_mode`
- `g_state.esc_pulse_us`
- `g_state.servo_pulse_us`
- `g_hall_speed_state.event_count_total`
- `g_hall_speed_state.last_period_us`
- `g_hall_speed_state.direction`
- `g_hall_speed_state.fault_count`
- `g_rc_override_active`
- `g_rc_guard_active`
- `g_orin_state.active`
- `g_orin_state.feedback_vx_mps`
- `g_orin_state.feedback_vz_rad_s`
- `g_orin_feedback_scale`
- `TIM8->CCR1`
- `TIM8->CCR2`

## 当前保留但不属于正式控制主路径
以下内容仍可作为调试或附属功能保留，但不再决定车辆正式控制行为：
- `USART1` 调试输出
- 超声波与回充附加数据打包
- 板载 OLED / RGB 显示
- 部分旧兼容接口空实现
