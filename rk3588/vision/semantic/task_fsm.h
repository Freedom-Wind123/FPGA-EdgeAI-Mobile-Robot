#pragma once

#include <cstdint>

#include "semantic_types.h"

namespace sem {

// FIND 在 SEARCH 命中后进入 DONE；APPROACH 才继续 APPROACH -> VERIFY。
class TaskFSM {
public:
    void setIntent(const Intent& intent, int64_t now_ms);

    // 搜索阶段的自转角速度（rad/s），用于按累计转角判定「自转两周」
    void setSearchAngularSpeed(float wz_rad_s) { search_wz_ = wz_rad_s; }

    // 到达判定距离（m），需与底盘停车阈值保持一致
    void setStopDistance(float dist_m) { stop_dist_m_ = dist_m; }

    // 底盘侧判定到位（如目标框占比达到停车阈值）时，通知状态机立即结束任务
    void markArrived(int64_t now_ms);

    // 每次视觉更新调用一次，返回本帧要发给 ACG720 的命令
    SemanticCommand tick(const SemanticTarget& target, int64_t now_ms);

    State state() const { return state_; }
    const Intent& intent() const { return intent_; }

private:
    void enter(State s, int64_t now_ms);

    State state_ = State::IDLE;
    Intent intent_{};
    SemanticTarget last_target_{};
    int64_t state_since_ = 0;
    int64_t last_valid_ms_ = 0;
    int64_t last_tick_ms_ = 0;
    int hit_streak_ = 0;
    int verify_streak_ = 0;
    float search_wz_ = 0.30f;
    float stop_dist_m_ = 0.6f;
    float search_rot_deg_ = 0.0f;
};

}  // namespace sem
