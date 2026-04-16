# Servo Basic Control Interface Reference (WHEELTEC)

This document enumerates the interfaces, variables, and parameters for the
servo basic control integration. It is intended as a concise reference for
future hardware or protocol extensions.

## Scope and Source Files
- WHEELTEC_APP/Inc/servo_basic_control.h
- WHEELTEC_APP/servo_basic_control.c
- WHEELTEC_APP/servo_basic_output.c
- WHEELTEC_APP/Inc/servo_basic_task.h
- WHEELTEC_APP/Inc/servo_rc_capture.h
- WHEELTEC_APP/servo_rc_capture.c
- WHEELTEC_APP/can_callback.c
- WHEELTEC_APP/tim_capture_callback.c
- Core/Src/main.c
- Core/Src/freertos.c
- Core/Src/tim.c
- Core/Inc/tim.h

## Public Interfaces
From WHEELTEC_APP/Inc/servo_basic_control.h:
- void ServoBasic_Init(void);
- void ServoBasic_HandleCanMessage(uint32_t can_id, uint8_t is_extended_id,
  const uint8_t *payload, uint8_t payload_len);
- void ServoBasic_ProcessControl(void);
- const servo_basic_state_t *ServoBasic_GetState(void);
- void ServoBasic_OutputEscPulse(uint16_t pulse_us);   // weak hook
- void ServoBasic_OutputServoPulse(uint16_t pulse_us); // weak hook

From WHEELTEC_APP/Inc/servo_basic_task.h:
- void ServoBasic_TaskInit(void);
- void ServoBasic_Task(void *param);
- BaseType_t ServoBasic_EnqueueCanMessageFromISR(uint32_t can_id,
  const uint8_t *payload, uint8_t payload_len,
  BaseType_t *pxHigherPriorityTaskWoken);

From WHEELTEC_APP/Inc/servo_rc_capture.h:
- void ServoRC_Capture_Init(void);
- void ServoRC_IC_CaptureCallback(TIM_HandleTypeDef *htim);
- uint16_t ServoRC_GetThrottlePulse(void);
- uint16_t ServoRC_GetSteeringPulse(void);
- uint16_t ServoRC_GetModePulse(void);

From WHEELTEC_APP/can_callback.c (debug helper):
- void Debug_SendServoExt(uint32_t ext_id, uint8_t cmd, uint16_t value);

HAL ISR entry points (integration):
- void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
- void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
- void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

## Public Types
From WHEELTEC_APP/Inc/servo_basic_control.h:
```
typedef enum
{
    SERVO_CTRL_MODE_RC_PASSTHROUGH = 0U,
    SERVO_CTRL_MODE_AUTONOMOUS     = 1U,
} servo_control_mode_t;

typedef struct
{
    uint16_t esc_pulse_us;
    uint16_t servo_pulse_us;
    servo_control_mode_t control_mode;
    uint8_t rc_takeover_pending;
    uint8_t emergency_stop;
} servo_basic_state_t;
```

ServoBasic_GetState() returns a pointer to the internal servo_basic_state_t.

## Global and Module Variables
WHEELTEC_APP/servo_basic_control.c:
- static servo_basic_state_t g_state
- static uint8_t g_rc_neutral_sample_count
- static uint16_t g_rc_takeover_ref_throttle
- static QueueHandle_t g_servo_basic_queue
- volatile uint32_t g_debug_servo_trigger
- volatile uint32_t g_debug_servo_can_id
- volatile uint32_t g_debug_servo_cmd
- volatile uint32_t g_debug_servo_value
- volatile uint32_t g_rc_pwm_follow_raw
- volatile uint32_t g_rc_signal_timeout_ms
- volatile uint32_t g_rc_debounce_enable
- volatile uint32_t g_rc_debounce_deadband_us
- volatile uint32_t g_rc_debounce_smooth_div
- static servo_basic_msg_t (queue element type):
  - uint32_t can_id
  - uint8_t payload_len
  - uint8_t payload[8]

WHEELTEC_APP/servo_rc_capture.c:
- static servo_rc_channel_state_t g_rc_throttle
- static servo_rc_channel_state_t g_rc_steering
- static servo_rc_channel_state_t g_rc_mode
- servo_rc_channel_state_t fields: last_capture, pulse_ticks, waiting_for_falling

WHEELTEC_APP/servo_basic_output.c:
- static uint8_t started (in start_pwm_once)

Core/Src/main.c (symbol retention for debug helper):
- static volatile uint32_t g_debug_servo_keep
- static void (* volatile g_debug_servo_fn)(uint32_t, uint8_t, uint16_t)
- static void Keep_Debug_SendServoExt_Symbol(void)

Debug trigger behavior (WHEELTEC_APP/servo_basic_control.c):
- Setting g_debug_servo_trigger to non-zero injects a synthetic CAN command
  in ServoBasic_Task on the next 20 ms cycle and then clears the trigger.
- g_debug_servo_can_id, g_debug_servo_cmd, and g_debug_servo_value are used
  to build the synthetic payload.

## Parameters and Constants
Protocol and PWM limits (WHEELTEC_APP/Inc/servo_basic_control.h):
- SERVO_CAN_ID_0 = 0x1314
- SERVO_CAN_ID_1 = 0x2568
- SERVO_CMD_SET_SERVO_ANGLE = 0x53
- SERVO_CMD_SET_SERVO_PULSE = 0x50
- SERVO_CMD_SET_ESC_PULSE   = 0x45
- ESC_PWM_MIN_PULSE_US      = 1000
- ESC_PWM_NEUTRAL_PULSE_US  = 1500
- ESC_PWM_MAX_PULSE_US      = 2000
- ESC_PULSE_STEP_US         = 1
- SERVO_MIN_PULSE_US        = 1000
- SERVO_MAX_PULSE_US        = 2000
- SERVO_PULSE_STEP_US       = 5

RC mode selection (WHEELTEC_APP/servo_basic_control.c):
- RC_MODE_LOW_CENTER_US            = 1000
- RC_MODE_MID_CENTER_US            = 1500
- RC_MODE_HIGH_CENTER_US           = 2000
- RC_MODE_TOL_US                   = 150
- RC_TAKEOVER_NEUTRAL_TOL_US       = 50
- RC_TAKEOVER_NEUTRAL_SAMPLES_REQ  = 3

Task and queue (Core/Src/freertos.c, WHEELTEC_APP/servo_basic_control.c):
- ServoBasic_Task period: 20 ms (50 Hz) via vTaskDelayUntil
- ServoBasic queue length: 8 messages
- Task priority: osPriorityHigh
- Task stack size: 128 * 2

Initialization behavior (WHEELTEC_APP/servo_basic_control.c, Core/Src/main.c):
- ServoBasic_Init sets control_mode to autonomous, clears takeover state, and
  initializes RC capture.
- Initial outputs: ESC neutral (1500 us) and servo midpoint (1500 us).

## CAN Protocol (Extended IDs Only)
Accepted extended IDs:
- 0x1314
- 0x2568

Commands (payload[0]):
- 0x53: set servo angle (degrees 0-180). Payload: [cmd, angle]
- 0x50: set servo pulse by step (5 us per step). Payload: [cmd, step]
- 0x45: set ESC pulse in microseconds (16-bit little-endian). Payload: [cmd, lo, hi]

Clamping and mapping:
- Angle is clamped to 0..180, then mapped to 1000..2000 us.
- Servo step is clamped to (SERVO_MAX - SERVO_MIN) / SERVO_PULSE_STEP_US = 200 steps.
- ESC pulse is clamped to 1000..2000 us.

Dispatch path:
- WHEELTEC_APP/can_callback.c enqueues CAN1 FIFO0 and CAN2 FIFO1 EXT frames to
  ServoBasic_EnqueueCanMessageFromISR.
- ServoBasic_Task drains the queue and calls ServoBasic_HandleCanMessage.

## Control Mode Behavior
Autonomous mode:
- Servo/ESC outputs follow the target pulses stored in g_state.
- ServoBasic_ProcessControl clamps targets before output.

RC passthrough mode:
- Uses live RC pulses from TIM4 capture for throttle and steering.
- On RC entry, ESC is held at 1000 us until throttle changes by more than 50 us
  for 3 samples (takeover safety).
- When g_rc_pwm_follow_raw is non-zero, RC passthrough uses raw pulse widths
  (bypasses 1000..2000 us clamp). Output still clips to TIM8 ARR.
- When input pulses are inactive for g_rc_signal_timeout_ms, outputs are
  forced to 0 (PWM low).
- When mode pulse is 0 (no mode channel), control defaults to RC passthrough.
- When g_rc_debounce_enable is non-zero, RC pulses are debounced using a
  deadband and smoothing divisor before output.

RC emergency stop:
- When the RC mode channel is centered near 1500 us (±150 us), outputs are
  forced to ESC=1000 us and Servo=1500 us, overriding other control modes.

## Hardware Mapping
RC input capture (TIM4):
- PD12 -> TIM4_CH1 (RC throttle)
- PD13 -> TIM4_CH2 (RC steering)
- PD14 -> TIM4_CH3 (RC mode)
- PD15 -> TIM4_CH4 (unused in current RC flow)

PWM output (TIM8):
- PC6  -> TIM8_CH1 (ESC PWM)
- PC7  -> TIM8_CH2 (Servo PWM)
- PC8  -> TIM8_CH3 (neutral output)
- PC9  -> TIM8_CH4 (neutral output)

## PWM Output Binding
WHEELTEC_APP/servo_basic_output.c:
- ServoBasic_OutputEscPulse updates TIM8_CH1.
- ServoBasic_OutputServoPulse updates TIM8_CH2.
- TIM8 CH3/CH4 are started and held at ESC_PWM_NEUTRAL_PULSE_US.
- Output values are clamped to TIM8 ARR before writing to CCR.

## Timer Configuration Details
TIM4 (RC input capture, Core/Src/tim.c):
- Prescaler: 83 (1 us tick assuming 84 MHz timer clock)
- ARR: 9999 (10 ms period)
- Channels 1-4 configured as input capture, rising edge default
- HAL_TIM_IC_Start_IT called for CH1-CH3 in Core/Src/main.c

TIM8 (PWM outputs, Core/Src/tim.c):
- Prescaler: 167 (1 us tick assuming 168 MHz timer clock)
- ARR: 2630 (about 2.631 ms period, ~380 Hz)
- Channels 1-4 configured as PWM1 with default pulse 1500

## RC Capture Details
WHEELTEC_APP/servo_rc_capture.c:
- Each channel measures pulse high time by toggling capture polarity
  (rising edge then falling edge).
- Pulse values are stored in microseconds (TIM4 tick is 1 us).
- pulse_ticks is set to 0 when the measured pulse exceeds 0xFFFF.
- Default pulse value per channel after init is 1500 us.

## Extension Points
- Override ServoBasic_OutputEscPulse and ServoBasic_OutputServoPulse to move PWM
  outputs to another timer or pin.
- Update Core/Src/tim.c and Core/Inc/tim.h for timer and pin remapping.
- Update WHEELTEC_APP/servo_rc_capture.c and WHEELTEC_APP/tim_capture_callback.c
  if RC input capture needs to move to another timer or channel.
