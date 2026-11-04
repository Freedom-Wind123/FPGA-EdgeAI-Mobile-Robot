#pragma once

#include <cstdint>

// 单个检测结果（图像坐标）
struct Detection {
    int cls = -1;
    float conf = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    int track_id = -1;
    int color = 0;   // 0=未知 1=红 2=绿 3=蓝
    float dist_m = 0.0f;   // 激光雷达测距，0 表示无回波，回退单目估计
    uint8_t source = 0;    // 产生该检测的模型序号（多模型并行时用于排查）
    int16_t parent = -1;   // 父类统一类别 ID，-1 表示无父类
};

#pragma pack(push, 1)

// 发给 ACG720 的单目标数据
struct TargetMsg {
    uint8_t  cls;
    uint8_t  color;
    uint8_t  conf;      // 0~100
    int16_t  cx;
    int16_t  cy;
    int16_t  w;
    int16_t  h;
    int16_t  bearing;   // 相对画面中心，0.1 度
    uint16_t dist_mm;   // 雷达优先，无回波时回退单目估计；0 表示无效
};

// 一帧视觉结果，30Hz 发送
struct VisionFrame {
    uint16_t magic;     // 0xAA55
    uint8_t  ver;
    uint8_t  count;
    uint32_t seq;
    uint32_t tick_ms;
    TargetMsg targets[4];
    uint16_t crc;       // CRC-16/X25，覆盖 crc 之前的全部字节
};

#pragma pack(pop)

static_assert(sizeof(TargetMsg) == 15, "TargetMsg must be packed");
static_assert(sizeof(VisionFrame) == 74, "VisionFrame must be packed");
