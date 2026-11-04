// 快速测试：模拟一帧 0xAA56 命令，验证跟随控制逻辑
#include "follow_controller.h"
#include <stdio.h>
#include <string.h>

static uint16_t crc16_x25(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

int main(void) {
    follow_init(NULL);

    // 构造一帧：APPROACH 模式，目标在右前方 30°，距离 2000mm
    SemanticCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.magic = 0xAA56;
    cmd.ver = 1;
    cmd.mode = 2;              // APPROACH
    cmd.target_cls = 39;       // bottle
    cmd.target_color = 1;      // red
    cmd.conf = 85;
    cmd.flags = 0x01;          // TARGET_VALID（偏角<25° 时不带 TURN_ONLY）
    cmd.target_id = 5;
    cmd.bearing = 300;         // 30.0° 右偏
    cmd.dist_mm = 2000;        // 2.0m
    cmd.crc = crc16_x25((uint8_t*)&cmd, sizeof(cmd) - 2);

    // 逐字节喂入
    uint8_t* p = (uint8_t*)&cmd;
    int got = 0;
    for (int i = 0; i < (int)sizeof(cmd); ++i) {
        got |= follow_feed_byte(p[i]);
    }
    printf("帧解析: %s\n", got ? "成功" : "失败");

    // 计算轮子输出
    float w[4];
    follow_tick(w);
    printf("目标: 右偏30°, 距离2.0m\n");
    printf("FL=%+.3f FR=%+.3f RL=%+.3f RR=%+.3f\n", w[0], w[1], w[2], w[3]);
    printf("解释: 右侧两轮较快 → 顺时针转向目标，同时前进\n\n");

    // 测试：偏角很大(35°) → 只转不进
    memset(&cmd, 0, sizeof(cmd));
    cmd.magic = 0xAA56; cmd.ver = 1; cmd.mode = 2;
    cmd.flags = 0x09;    // VALID | TURN_ONLY
    cmd.bearing = 350;   // 35°
    cmd.dist_mm = 2000;
    cmd.crc = crc16_x25((uint8_t*)&cmd, sizeof(cmd) - 2);
    for (int i = 0; i < (int)sizeof(cmd); ++i) follow_feed_byte(((uint8_t*)&cmd)[i]);
    follow_tick(w);
    printf("目标: 右偏35°(TURN_ONLY), 距离2.0m\n");
    printf("FL=%+.3f FR=%+.3f RL=%+.3f RR=%+.3f\n", w[0], w[1], w[2], w[3]);
    printf("解释: 左右轮反向 → 原地顺时针转向，不前进\n\n");

    // 测试：到达距离内(0.5m) → 停车
    memset(&cmd, 0, sizeof(cmd));
    cmd.magic = 0xAA56; cmd.ver = 1; cmd.mode = 2;
    cmd.flags = 0x01; cmd.bearing = 50; cmd.dist_mm = 500;
    cmd.crc = crc16_x25((uint8_t*)&cmd, sizeof(cmd) - 2);
    for (int i = 0; i < (int)sizeof(cmd); ++i) follow_feed_byte(((uint8_t*)&cmd)[i]);
    follow_tick(w);
    printf("目标: 距离0.5m(<0.6m阈值)\n");
    printf("FL=%+.3f FR=%+.3f RL=%+.3f RR=%+.3f\n", w[0], w[1], w[2], w[3]);
    printf("解释: 全零 → 已到达，停车\n");

    return 0;
}
