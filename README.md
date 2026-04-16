# WHEELTEC 舵机与 Ackermann PWM 控制说明

本文档说明当前 WHEELTEC 工程中 `servo_basic_control` 相关的 PWM 控制链路、串口输入、RC 抢占逻辑与调试方法。

## 功能概览
- 支持 CAN 扩展帧控制 ESC / Servo：
  - IDs：`0x1314`、`0x2568`
  - Commands：`0x53`、`0x50`、`0x45`
- 支持 RC 输入直通：
  - `PD12`：RC throttle
  - `PD13`：RC steering
  - `PD14`：RC guard / emergency input
- 支持 AGX Orin 串口速度控制：
  - `SerialControlTask` 解析 `Vx/Vy/Vz`
  - `ServoBasic_UpdateFromOrin()` 将 `Vx/Vz` 映射为 ESC / Servo PWM
- 支持 RC 抢占自动驾驶：
  - 默认 autonomous
  - 当 RC 油门或转向偏离中位达到阈值时，自动抢占串口控制
- 支持基于 `EXTRINSICS.md` 的 Ackermann 几何映射：
  - `wheelbase = 0.54 m`
  - `track_width = 0.48 m`
  - `wheel_radius = 0.11 m`
  - `max_steering_angle = 0.393 rad`
- 支持统一安全限速：
  - 自动驾驶和 RC 都受 ESC 上限约束
  - 默认前进 / 后退速度上限均为 `2.0 m/s`

## 代码入口
- PWM / 协议逻辑：
  - `WHEELTEC_APP/servo_basic_control.c`
  - `WHEELTEC_APP/Inc/servo_basic_control.h`
- PWM 硬件输出：
  - `WHEELTEC_APP/servo_basic_output.c`
- RC 输入捕获：
  - `WHEELTEC_APP/servo_rc_capture.c`
  - `WHEELTEC_APP/Inc/servo_rc_capture.h`
- 串口协议解析：
  - `WHEELTEC_APP/SerialControl_task.c`
  - `WHEELTEC_APP/uart_callback.c`
- 定时器与 GPIO：
  - `Core/Src/tim.c`
- 几何参数来源：
  - `EXTRINSICS.md`

## 定时器与引脚映射
RC 输入捕获（`TIM4`）：
- `PD12 -> TIM4_CH1`：RC throttle
- `PD13 -> TIM4_CH2`：RC steering
- `PD14 -> TIM4_CH3`：RC guard / emergency
- `PD15 -> TIM4_CH4`：unused

PWM 输出（`TIM8`）：
- `PC6 -> TIM8_CH1`：ESC PWM
- `PC7 -> TIM8_CH2`：Servo PWM
- `PC8 -> TIM8_CH3`：neutral output
- `PC9 -> TIM8_CH4`：neutral output

定时器参数：
- `TIM4`：`1 us` tick，输入捕获
- `TIM8`：`1 us` tick，`ARR = 2630`，周期约 `2.631 ms`，频率约 `380 Hz`

补充：
- `PD14` 当前在 `tim.c` 中配置为 `GPIO_PULLDOWN`，用于避免悬空误触发。

## 对外接口
`WHEELTEC_APP/Inc/servo_basic_control.h`：
- `void ServoBasic_Init(void);`
- `void ServoBasic_HandleCanMessage(uint32_t can_id, uint8_t is_extended_id, const uint8_t *payload, uint8_t payload_len);`
- `void ServoBasic_ProcessControl(void);`
- `const servo_basic_state_t *ServoBasic_GetState(void);`
- `uint8_t ServoBasic_IsRcOverrideActive(void);`
- `uint8_t ServoBasic_IsRcEmergencyActive(void);`
- `void ServoBasic_UpdateFromOrin(float vx_mps, float vy_mps, float vz_rad_s, uint8_t flag_stop);`
- `void ServoBasic_OutputEscPulse(uint16_t pulse_us);`
- `void ServoBasic_OutputServoPulse(uint16_t pulse_us);`

`WHEELTEC_APP/Inc/servo_rc_capture.h`：
- `void ServoRC_Capture_Init(void);`
- `void ServoRC_IC_CaptureCallback(TIM_HandleTypeDef *htim);`
- `uint16_t ServoRC_GetThrottlePulse(void);`
- `uint16_t ServoRC_GetSteeringPulse(void);`
- `uint16_t ServoRC_GetGuardPulse(void);`
- `uint8_t ServoRC_IsThrottleActive(uint32_t timeout_ms);`
- `uint8_t ServoRC_IsSteeringActive(uint32_t timeout_ms);`
- `uint8_t ServoRC_IsGuardActive(uint32_t timeout_ms);`

## CAN 协议
支持的扩展 ID：
- `0x1314`
- `0x2568`

支持的命令：
- `0x53`：设置舵机角度
- `0x50`：按步进设置舵机脉宽
- `0x45`：设置 ESC 脉宽

夹紧规则：
- Servo pulse：`1000..2000 us`
- ESC pulse：`1000..2000 us`

说明：
- CAN 目标值最终也会经过当前输出安全限制。

## Orin 串口控制协议
控制帧格式固定为 11 字节：

```text
0x7B  CMD1  CMD2  XH  XL  YH  YL  ZH  ZL  BCC  0x7D
```

解释：
- `CMD1 = 0x00`：正常速度控制
- `CMD2 bit7`：`flag_stop`
- `X/Y`：单位 `mm/s`
- `Z`：单位 `0.001 rad/s`
- `BCC`：前 9 字节按 XOR 校验

当前 PWM 链路使用：
- `Vx`：映射为 ESC PWM
- `Vz`：结合 `Vx` 通过 Ackermann 自行车模型映射为 Servo PWM
- `Vy`：当前不参与 PWM 映射

## Ackermann 几何映射
当前 PWM 自动驾驶路径不再使用简单的 `Vz -> Servo` 线性映射，而是使用：

```text
delta = atan(L * wz / vx)
```

其中：
- `L = wheelbase = 0.54 m`
- `wz = Vz (rad/s)`
- `vx = Vx (m/s)`
- `delta` 为等效前轮转角

限制规则：
- `|delta| <= 0.393 rad`
- `delta = 0` 对应 `g_orin_servo_center_us`
- `±0.393 rad` 对应 `g_orin_servo_center_us ± g_orin_servo_range_us`

低速保护：
- 当 `|vx| < 0.05 m/s` 时：
  - ESC 输出回中
  - Servo 输出回中
  - 不再继续根据 `wz / vx` 放大转向

倒车转向：
- 由于公式直接使用带符号的 `vx`，倒车时同样的 `Vz` 会自动得到相反符号的 `delta`。

## ESC 速度映射
当前 `Vx -> ESC PWM` 使用分段映射，不再使用单一线性满量程。

规则：
- `|vx| <= deadband`：`ESC = center`
- `vx > deadband`：映射到 `[forward_start_us, forward_max_us]`
- `vx < -deadband`：映射到 `[reverse_start_us, reverse_max_us]`

默认安全参数：
- `g_orin_vx_forward_cap_mmps = 2000`
- `g_orin_vx_reverse_cap_mmps = 2000`
- `g_orin_vx_deadband_mmps = 50`
- `g_orin_esc_forward_start_us = 1560`
- `g_orin_esc_reverse_start_us = 1440`
- `g_orin_esc_forward_max_us = 1650`
- `g_orin_esc_reverse_max_us = 1350`

说明：
- 这些默认值偏保守，目的是降低架起测试时的危险性。
- 后续如果你提供“指令速度 vs 实测速度”对照表，可优先标定：
  - `g_orin_esc_forward_start_us`
  - `g_orin_esc_reverse_start_us`
  - `g_orin_esc_forward_max_us`
  - `g_orin_esc_reverse_max_us`
  - `g_orin_vx_scale`

## RC 抢占与保障逻辑
当前工程不再使用 `PD14` 作为模式通道。

实际逻辑：
- 若当前没有有效串口 PWM 控制流，且 RC 接收机有效，则直接进入 `RC passthrough`
- 若当前存在有效串口 PWM 控制流，则 `PD12/PD13` 需要明显偏离中位，才会触发 RC 抢占
- `PD14` 作为保障输入，触发时立即进入 RC override / emergency

RC 抢占默认阈值：
- `g_rc_override_center_us = 1500`
- `g_rc_override_enter_threshold_us = 60`
- `g_rc_override_exit_threshold_us = 40`
- `g_rc_override_enter_samples = 2`
- `g_rc_override_release_hold_ms = 500`

含义：
- 无串口控制时，RC 正常直通，不需要先越过抢占阈值
- 有串口控制时，油门或转向离中位超过 `60 us`，并连续满足 2 个控制周期时，进入 RC 抢占
- 回到中位 `±40 us` 内，并保持 `500 ms` 后，退出 RC 抢占并恢复 autonomous

串口仲裁：
- `RC override == 1` 时，串口非零运动命令会被阻断
- 刚进入 RC 抢占时，会自动下发一次零速命令清掉旧自动驾驶目标

## 可在 Keil Watch 中调的关键参数
几何 / Ackermann：
- `g_orin_ackermann_wheelbase_mm`
- `g_orin_ackermann_track_width_mm`
- `g_orin_ackermann_wheel_radius_mm`
- `g_orin_ackermann_max_steering_millirad`
- `g_orin_ackermann_min_vx_mmps`

纵向速度映射：
- `g_orin_vx_scale`
- `g_orin_vx_forward_cap_mmps`
- `g_orin_vx_reverse_cap_mmps`
- `g_orin_vx_deadband_mmps`
- `g_orin_esc_forward_start_us`
- `g_orin_esc_reverse_start_us`
- `g_orin_esc_forward_max_us`
- `g_orin_esc_reverse_max_us`

舵机 / PWM：
- `g_orin_servo_center_us`
- `g_orin_servo_range_us`
- `g_orin_esc_center_us`
- `g_orin_pwm_timeout_ms`
- `g_orin_pwm_enable`

RC 抢占：
- `g_rc_override_enter_threshold_us`
- `g_rc_override_exit_threshold_us`
- `g_rc_override_enter_samples`
- `g_rc_override_release_hold_ms`
- `g_rc_guard_enable`
- `g_rc_guard_active_high`
- `g_rc_guard_active_low_threshold_us`
- `g_rc_guard_active_high_threshold_us`

调试触发：
- `g_debug_servo_can_id`
- `g_debug_servo_cmd`
- `g_debug_servo_value`
- `g_debug_servo_trigger`

## 调试方法
### 1. 逻辑分析仪
观察：
- `PC6`：ESC PWM
- `PC7`：Servo PWM

判定基准：
- 周期约 `2.631 ms`
- 脉宽在 `1000..2000 us` 之间

### 2. Keil Watch
建议同时观察：
- `TIM8->CCR1`
- `TIM8->CCR2`
- `g_state.control_mode`
- `g_rc_override_active`
- `g_rc_guard_active`
- `g_rc_throttle_current`
- `g_rc_steering_current`

### 3. 串口速度帧
示例：`Vx = 0.12 m/s, Vz = 0`

```text
7B 00 00 00 78 00 00 00 00 03 7D
```

说明：
- 自动驾驶是否真正生效，要结合 `TIM8->CCR1 / CCR2` 与 `g_rc_override_active` 一起看。

## 常见问题
1. 串口发送后 PWM 不变
   - 检查 `g_orin_pwm_enable`
   - 检查 `g_rc_override_active` 是否被 RC 覆盖
   - 检查 `g_orin_state.last_update_ms` 是否在更新

2. RC 明明接着，但自动驾驶没有被覆盖
   - 检查 `g_rc_override_enter_threshold_us`
   - 检查 `g_rc_throttle_current / g_rc_steering_current` 是否真实偏离中位

3. 架起时轮速过快
   - 优先降低：
     - `g_orin_esc_forward_max_us`
     - `g_orin_esc_reverse_max_us`
     - `g_orin_vx_forward_cap_mmps`
     - `g_orin_vx_reverse_cap_mmps`

4. `PD14` 悬空导致异常
   - 当前已配置下拉，且仅作为 guard 输入使用
   - 但仍建议硬件上不要长期悬空

## 安全提示
- 调试前建议先架空车轮或断开机械负载。
- 先用较小 `Vx` 和较小 `g_orin_esc_forward_max_us / reverse_max_us` 联调。
- 实车联调时优先确认舵机中心与机械极限，避免打角过头。
