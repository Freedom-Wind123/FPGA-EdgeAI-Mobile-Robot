// ============================================================================
// 小车跟随主循环示例（FPGA 软核 CPU / STM32 / ESP32 均可移植）
//
// 核心流程：
//   UART 收字节 → follow_feed_byte() → follow_tick() → 4 路 PWM 输出
// ============================================================================

#include "follow_controller.h"
#include <stdint.h>

// ---- 平台相关：根据你的 MCU/FPGA 改这几个函数 ----

// 从 UART 接收缓冲区读一个字节，无数据返回 -1
static int uart_recv_byte(void) {
    // 例：STM32:  if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE)) return (uint8_t)(huart.Instance->DR);
    // 例：ESP32:  uint8_t b; if (uart_read_bytes(UART_NUM_1, &b, 1, 0) == 1) return b;
    // 例：FPGA软核: return uart_read();
    return -1;  // 无数据
}

// 设置单个轮子的 PWM 占空比，speed 范围 -1.0 ~ +1.0
static void set_wheel_pwm(int wheel, float speed) {
    // speed > 0 正转，< 0 反转
    // 例：int duty = (int)(speed * PWM_MAX);  motor_set(wheel, duty);
    (void)wheel; (void)speed;
}

// ---- 主循环 ----

int main(void) {
    FollowParams params;
    follow_init(&params);  // 用默认参数，或自行设置

    while (1) {
        // 1. 喂入所有收到的 UART 字节
        int b;
        while ((b = uart_recv_byte()) >= 0) {
            follow_feed_byte((uint8_t)b);
        }

        // 2. 计算本轮 4 个轮子速度
        float wheels[WHEEL_NUM];
        follow_tick(wheels);

        // 3. 输出到电机驱动
        for (int i = 0; i < WHEEL_NUM; ++i) {
            set_wheel_pwm(i, wheels[i]);
        }

        // 4. 适当延时，控制循环频率（如 50Hz = 20ms）
        // delay_ms(20);
    }
}
