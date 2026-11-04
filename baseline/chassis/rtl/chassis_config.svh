`ifndef CHASSIS_CONFIG_SVH
`define CHASSIS_CONFIG_SVH

// ACG720 onboard oscillator.
`define CLK_HZ              50_000_000
`define UART_BAUD           115_200

// STM32 source behavior: TIM1 ran at 1 MHz with ARR=1000.
`define PWM_HZ              1_000
`define PWM_MAX             1_000

// Q16.16 constants copied from the STM32 project.
`define Q16_ONE             32'sd65536
`define RAMP_STEP_Q16       32'sd32768       // 0.5 rev/s per 10 ms
`define STOP_EPS_Q16        32'sd6554        // 0.1 rev/s
`define INV_WHEEL_CIRC_Q16  32'sd219587      // 1/(pi*0.095)
`define L_OVER_CIRC_Q16     32'sd41722       // 0.190/(pi*0.095)
`define KEY_WHEEL_RPS_Q16   32'sd65876       // 0.3/(pi*0.095)

`define PID_KP              925
`define PID_KI              82
`define PID_KD              27

// Physical installation flags copied verbatim from Core/Inc/main.h.
`define MOTOR_A_REVERSE     0
`define MOTOR_B_REVERSE     0
`define MOTOR_C_REVERSE     0
`define MOTOR_D_REVERSE     0
`define ENCODER_A_REVERSE   0
`define ENCODER_B_REVERSE   1
`define ENCODER_C_REVERSE   1
`define ENCODER_D_REVERSE   0

`endif
