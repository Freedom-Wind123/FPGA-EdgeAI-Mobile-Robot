#include "follow_controller.h"

#include <string.h>

// ---------------------------------------------------------------------------
// CRC-16/X25 校验（必须与 RK3588 端一致）
// ---------------------------------------------------------------------------
static uint16_t crc16_x25(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
        }
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

// ---------------------------------------------------------------------------
// 帧解析状态机
// 注意：RK3588 为小端，uint16_t magic=0xAA56 在线上实际字节序为 0x56 0xAA
typedef enum {
    ST_MAGIC_LO,   // 等 0x56（magic 低字节，先到）
    ST_MAGIC_HI,   // 等 0xAA（magic 高字节，后到）
    ST_BODY,       // 收帧体
} ParseState;

static ParseState g_state = ST_MAGIC_LO;
static uint8_t    g_buf[sizeof(SemanticCmd)];
static int        g_idx = 0;

static SemanticCmd g_cmd;        // 最新一帧有效命令
static uint8_t     g_cmd_valid = 0;

static FollowParams g_p = {
    .stop_dist_m   = 0.6f,
    .turn_only_deg = 12.0f,
    .fwd_kp        = 0.5f,
    .turn_kp       = 0.03f,
    .max_vx        = 0.8f,
    .max_wz        = 0.6f,
    .wheel_factor  = 0.2f,   // (半轮距+半轴距)，按实际小车改
};

void follow_init(const FollowParams* params) {
    if (params) g_p = *params;
    g_state = ST_MAGIC_LO;
    g_idx = 0;
    g_cmd_valid = 0;
    memset(&g_cmd, 0, sizeof(g_cmd));
}

int follow_feed_byte(uint8_t b) {
    switch (g_state) {
        case ST_MAGIC_LO:
            if (b == 0x56) g_state = ST_MAGIC_HI;
            return 0;

        case ST_MAGIC_HI:
            if (b == 0xAA) {
                g_state = ST_BODY;
                g_idx = 0;
                g_buf[g_idx++] = 0x56;
                g_buf[g_idx++] = 0xAA;
            } else {
                g_state = ST_MAGIC_LO;
            }
            return 0;

        case ST_BODY:
            g_buf[g_idx++] = b;
            if (g_idx < (int)sizeof(SemanticCmd)) return 0;

            // 收满一帧，校验 CRC
            g_state = ST_MAGIC_LO;
            g_idx = 0;
            {
                SemanticCmd* c = (SemanticCmd*)g_buf;
                uint16_t calc = crc16_x25(g_buf, (int)sizeof(SemanticCmd) - 2);
                if (calc != c->crc) return 0;

                g_cmd = *c;
                g_cmd_valid = 1;
                return 1;
            }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// 限幅工具
// ---------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------------------
// 麦克纳姆轮逆运动学
//   Vx: 前进(+)后退(-)   Vy: 左移(+)右移(-)   Wz: 逆时针(+)顺时针(-)
//   FL = Vx - Vy - Wz*k
//   FR = Vx + Vy + Wz*k
//   RL = Vx + Vy - Wz*k
//   RR = Vx - Vy + Wz*k
//   k = 半轮距 + 半轴距
// ---------------------------------------------------------------------------
static void mecanum(float vx, float vy, float wz, float k, float out[4]) {
    out[WHEEL_FL] = vx - vy - wz * k;
    out[WHEEL_FR] = vx + vy + wz * k;
    out[WHEEL_RL] = vx + vy - wz * k;
    out[WHEEL_RR] = vx - vy + wz * k;
}

void follow_tick(float wheel_out[WHEEL_NUM]) {
    // 默认停车
    wheel_out[WHEEL_FL] = 0.0f;
    wheel_out[WHEEL_FR] = 0.0f;
    wheel_out[WHEEL_RL] = 0.0f;
    wheel_out[WHEEL_RR] = 0.0f;

    if (!g_cmd_valid) return;

    // 非跟随状态直接停车
    if (g_cmd.mode != 2 /*APPROACH*/) return;

    // 目标丢失或无效：停车
    if (!(g_cmd.flags & 0x01 /*FLAG_TARGET_VALID*/)) return;

    float bearing_deg = g_cmd.bearing / 10.0f;   // 右为正
    float dist_m = g_cmd.dist_mm / 1000.0f;

    // 到达目标距离内：停车
    if (dist_m > 0.0f && dist_m < g_p.stop_dist_m) return;

    // 角速度：向目标方向转
    //   bearing > 0（目标在右）→ 顺时针转 → Wz 为负
    float wz = -g_p.turn_kp * bearing_deg;

    // 前进速度：与剩余距离成正比
    float remain = dist_m - g_p.stop_dist_m;
    float vx = g_p.fwd_kp * remain;

    // 偏角太大时只转不进（与视觉端 FLAG_TURN_ONLY 一致）
    if (g_cmd.flags & 0x08 /*FLAG_TURN_ONLY*/) {
        vx = 0.0f;
    }

    // 限幅
    vx = clampf(vx, -g_p.max_vx, g_p.max_vx);
    wz = clampf(wz, -g_p.max_wz, g_p.max_wz);

    // 暂不使用横移（Vy=0），仅前进+转向；如需横向微调可在此加入 Vy
    mecanum(vx, 0.0f, wz, g_p.wheel_factor, wheel_out);

    // 四轮整体限幅到 [-1, 1]
    float maxv = 0.0f;
    for (int i = 0; i < WHEEL_NUM; ++i) {
        float a = wheel_out[i] < 0 ? -wheel_out[i] : wheel_out[i];
        if (a > maxv) maxv = a;
    }
    if (maxv > 1.0f) {
        float s = 1.0f / maxv;
        for (int i = 0; i < WHEEL_NUM; ++i) wheel_out[i] *= s;
    }
}

// ---------------------------------------------------------------------------
// 调试接口
// ---------------------------------------------------------------------------
uint8_t  follow_get_mode(void)      { return g_cmd.mode; }
int16_t  follow_get_bearing(void)   { return g_cmd.bearing; }
uint16_t follow_get_dist_mm(void)   { return g_cmd.dist_mm; }
uint8_t  follow_get_flags(void)     { return g_cmd.flags; }
