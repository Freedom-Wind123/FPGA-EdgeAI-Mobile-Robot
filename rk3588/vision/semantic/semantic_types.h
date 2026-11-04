#pragma once

#include <cstdint>

namespace sem {

// 与视觉层 Detection::color 保持一致
enum ColorId : uint8_t {
    COLOR_ANY = 0,
    COLOR_RED = 1,
    COLOR_GREEN = 2,
    COLOR_BLUE = 3,
};

// 语义任务状态，数值直接写入协议 mode 字段
enum class State : uint8_t {
    IDLE = 0,
    SEARCH = 1,
    APPROACH = 2,
    VERIFY = 3,
    DONE = 4,
    FAIL = 5,
    STOP = 6,
};

// 语义命令 flags 位
enum CommandFlag : uint8_t {
    FLAG_TARGET_VALID = 1u << 0,  // 本帧目标可见
    FLAG_TARGET_LOST = 1u << 1,   // 逼近过程中丢失，等待重搜
    FLAG_VERIFY_OK = 1u << 2,     // 到达且确认完成
    FLAG_TURN_ONLY = 1u << 3,      // 只允许原地转向，不允许前进
};

// 用户一句话里解析出的目标描述
struct TargetDesc {
    enum Relation { NEAREST = 0, LEFT = 1, RIGHT = 2, CENTER = 3 };

    int cls = -1;                   // COCO 类别下标，-1 表示任意
    uint8_t color = COLOR_ANY;      // 0 任意
    Relation relation = NEAREST;
    float max_distance_m = 5.0f;
};

enum class Action : uint8_t {
    FIND = 0,
    APPROACH = 1,
    STOP = 2,
    REPORT = 3,
};

struct Intent {
    Action action = Action::FIND;
    TargetDesc target;
    bool valid = false;
};

// 单帧观测，来自视觉层
struct ObjectObservation {
    int track_id = -1;
    int cls = -1;
    uint8_t color = COLOR_ANY;
    float conf = 0.0f;
    float bearing_deg = 0.0f;   // 右为正
    float distance_m = 0.0f;    // 0 表示无效
    int64_t timestamp_ms = 0;
};

// 世界模型里的稳定目标
struct SemanticObject {
    int track_id = -1;
    int cls = -1;
    uint8_t color = COLOR_ANY;
    float conf = 0.0f;
    float bearing_deg = 0.0f;
    float distance_m = 0.0f;
    int hits = 0;
    int64_t last_seen_ms = 0;
    bool valid = false;
};

// 当前任务选中的目标
struct SemanticTarget {
    int track_id = -1;
    int cls = -1;
    uint8_t color = COLOR_ANY;
    float conf = 0.0f;
    float bearing_deg = 0.0f;
    float distance_m = 0.0f;
    float score = 0.0f;
    bool valid = false;
};

#pragma pack(push, 1)

// RK3588 -> ACG720 语义命令帧，24 字节
struct SemanticCommand {
    uint16_t magic;         // 0xAA56
    uint8_t  ver;           // 1
    uint8_t  mode;          // State
    uint8_t  target_cls;    // 0xFF 表示无目标
    uint8_t  target_color;  // ColorId
    uint8_t  conf;          // 0~100
    uint8_t  flags;         // CommandFlag
    int16_t  target_id;     // 跟踪 ID，-1 无
    int16_t  bearing;       // 相对画面中心，0.1 度
    uint16_t dist_mm;       // 雷达优先，无回波时回退单目；0 无效
    uint32_t seq;
    uint32_t tick_ms;
    uint16_t crc;           // CRC-16/X25
};

#pragma pack(pop)

static_assert(sizeof(SemanticCommand) == 24, "SemanticCommand must be packed");

inline const char* stateName(State s) {
    switch (s) {
        case State::IDLE: return "IDLE";
        case State::SEARCH: return "SEARCH";
        case State::APPROACH: return "APPROACH";
        case State::VERIFY: return "VERIFY";
        case State::DONE: return "DONE";
        case State::FAIL: return "FAIL";
        case State::STOP: return "STOP";
    }
    return "UNKNOWN";
}

inline const char* actionName(Action a) {
    switch (a) {
        case Action::FIND: return "FIND";
        case Action::APPROACH: return "APPROACH";
        case Action::STOP: return "STOP";
        case Action::REPORT: return "REPORT";
    }
    return "UNKNOWN";
}

}  // namespace sem
