#pragma once

#include <stdint.h>

// ============================================================================
// 小车自主跟随控制器 —— 接收 RK3588 的 0xAA56 语义帧，驱动麦克纳姆轮
//
// 用法：
//   1. UART 每收到一个字节，调用 follow_feed_byte(b)
//   2. 解析到完整帧后，内部自动更新目标状态
//   3. 主循环定时调用 follow_tick()，读取 4 个轮子的 PWM 值
//   4. 将 wheel_pwm[] 输出到电机驱动
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// 0xAA56 语义命令帧结构（必须与 RK3588 端 semantic_types.h 完全一致）
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;         // 0xAA56
    uint8_t  ver;           // 1
    uint8_t  mode;          // 0=IDLE 1=SEARCH 2=APPROACH 3=VERIFY 4=DONE 5=FAIL 6=STOP
    uint8_t  target_cls;    // 0xFF 无目标
    uint8_t  target_color;  // 0=任意 1=红 2=绿 3=蓝
    uint8_t  conf;          // 0~100
    uint8_t  flags;         // bit0=目标可见 bit1=丢失 bit2=到达 bit3=只转不进
    int16_t  target_id;     // 跟踪 ID
    int16_t  bearing;       // 相对画面中心，0.1 度，右为正
    uint16_t dist_mm;       // 距离 mm，0 无效
    uint32_t seq;
    uint32_t tick_ms;
    uint16_t crc;           // CRC-16/X25
} SemanticCmd;
#pragma pack(pop)

// 麦克纳姆轮编号
enum { WHEEL_FL = 0, WHEEL_FR = 1, WHEEL_RL = 2, WHEEL_RR = 3, WHEEL_NUM = 4 };

// 控制器参数（根据实际小车尺寸和电机调速范围调整）
typedef struct {
    float stop_dist_m;        // 到达距离阈值，默认 0.6m（与视觉端 APPROACH→VERIFY 一致）
    float turn_only_deg;      // 偏角超过此值只转不进，默认 12°（与视觉端 FLAG_TURN_ONLY 一致）
    float fwd_kp;             // 前进速度比例系数
    float turn_kp;            // 转向角速度比例系数
    float max_vx;             // 最大前进速度（PWM 或归一化 -1~1）
    float max_wz;             // 最大角速度
    float wheel_factor;       // 麦轮解算系数 = (半轮距 + 半轴距)，单位与速度一致
} FollowParams;

// 初始化，设置参数
void follow_init(const FollowParams* params);

// 喂入一个 UART 字节，内部状态机解析 0xAA56 帧
// 返回 1 表示刚解析到一帧有效命令，0 表示还在收
int follow_feed_byte(uint8_t b);

// 主循环调用：根据最新目标计算 4 个轮子的速度
// wheel_out[4] 输出范围 -1.0 ~ +1.0，由外层映射到实际 PWM
void follow_tick(float wheel_out[WHEEL_NUM]);

// 获取当前状态（调试用）
uint8_t follow_get_mode(void);
int16_t follow_get_bearing(void);   // 0.1 度
uint16_t follow_get_dist_mm(void);
uint8_t follow_get_flags(void);

#ifdef __cplusplus
}
#endif
