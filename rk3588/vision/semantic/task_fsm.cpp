#include "task_fsm.h"

#include <algorithm>
#include <cmath>

namespace sem {
namespace {

// 连续命中帧数达到该值即锁定目标（约 0.1s，兼顾误检与响应速度）
constexpr int kLockHits = 3;
// VERIFY 确认帧数
constexpr int kVerifyHits = 2;
// 搜索阶段最大自转圈数：两圈内没找到目标就结束本次任务
constexpr float kSearchTurns = 2.0f;
// 搜索阶段兜底超时，防止角速度配置异常时一直转
constexpr int64_t kSearchBackstopMs = 90000;
// 逼近阶段目标持续消失多久后结束任务
constexpr int64_t kApproachLostMs = 1500;
// 逼近阶段兜底超时
constexpr int64_t kApproachTimeoutMs = 60000;
// 目标偏离画面中心超过该角度时只转不进（先居中，再靠近）
constexpr float kTurnOnlyDeg = 12.0f;

constexpr float kRadToDeg = 57.2957795f;

}  // namespace

void TaskFSM::enter(State s, int64_t now_ms) {
    state_ = s;
    state_since_ = now_ms;
    last_tick_ms_ = now_ms;
    verify_streak_ = 0;
    hit_streak_ = 0;
    if (s == State::SEARCH) search_rot_deg_ = 0.0f;
}

void TaskFSM::markArrived(int64_t now_ms) {
    // 底盘判定已贴到目标：直接结束，不再反复校验
    if (state_ == State::APPROACH) enter(State::DONE, now_ms);
}

void TaskFSM::setIntent(const Intent& intent, int64_t now_ms) {
    intent_ = intent;
    last_target_ = SemanticTarget{};
    last_valid_ms_ = now_ms;
    hit_streak_ = 0;
    verify_streak_ = 0;
    search_rot_deg_ = 0.0f;

    switch (intent.action) {
        case Action::STOP:
            enter(State::STOP, now_ms);
            break;
        case Action::REPORT:
            enter(State::IDLE, now_ms);
            break;
        case Action::FIND:
        case Action::APPROACH:
        default:
            enter(State::SEARCH, now_ms);
            break;
    }
}

SemanticCommand TaskFSM::tick(const SemanticTarget& target, int64_t now_ms) {
    if (target.valid) {
        last_target_ = target;
        last_valid_ms_ = now_ms;
        ++hit_streak_;
    } else {
        hit_streak_ = 0;
    }

    // 按实际下发的搜索角速度累计转角；目标可见时底盘会停转，不计入
    if (last_tick_ms_ > 0 && now_ms > last_tick_ms_) {
        const int64_t dt_ms = std::min<int64_t>(now_ms - last_tick_ms_, 1000);
        if (state_ == State::SEARCH && !target.valid) {
            search_rot_deg_ += std::fabs(search_wz_) *
                               static_cast<float>(dt_ms) * 0.001f * kRadToDeg;
        }
    }
    last_tick_ms_ = now_ms;

    const int64_t since_state = now_ms - state_since_;
    const int64_t since_valid = now_ms - last_valid_ms_;

    switch (state_) {
        case State::SEARCH:
            if (hit_streak_ >= kLockHits) {
                // 找到目标立即停止搜索：FIND 直接完成任务，APPROACH 先居中再逼近
                enter(intent_.action == Action::APPROACH ? State::APPROACH
                                                         : State::DONE,
                      now_ms);
            } else if (search_rot_deg_ >= kSearchTurns * 360.0f) {
                // 缓慢自转满两圈仍没找到，结束本次任务
                enter(State::FAIL, now_ms);
            } else if (since_state > kSearchBackstopMs) {
                enter(State::FAIL, now_ms);
            }
            break;

        case State::APPROACH:
            if (target.valid && target.distance_m > 0.0f &&
                target.distance_m <= stop_dist_m_) {
                // 到位立即结束任务，不再原地旋转或反复搜索
                enter(State::VERIFY, now_ms);
            } else if (since_valid > kApproachLostMs) {
                // 逼近中目标持续消失（被遮挡/转出视野）：退回搜索，重新自转找目标
                enter(State::SEARCH, now_ms);
            } else if (since_state > kApproachTimeoutMs) {
                enter(State::FAIL, now_ms);
            }
            break;

        case State::VERIFY:
            if (target.valid) {
                ++verify_streak_;
            } else {
                verify_streak_ = 0;
            }
            if (verify_streak_ >= kVerifyHits) {
                enter(State::DONE, now_ms);
            } else if (since_valid > 1000) {
                // 已到位，丢失也不再回到旋转搜索
                enter(State::DONE, now_ms);
            }
            break;

        default:
            break;
    }

    SemanticCommand cmd{};
    cmd.target_cls = 0xFF;
    cmd.target_color = COLOR_ANY;
    cmd.target_id = -1;
    cmd.mode = static_cast<uint8_t>(state_);

    // 当前有目标优先用当前值，否则 DONE/FAIL 时保留最后一次目标信息
    const SemanticTarget& info = target.valid ? target : last_target_;
    const bool show_info =
        target.valid || state_ == State::DONE || state_ == State::FAIL;

    if (show_info && info.valid) {
        cmd.target_cls = static_cast<uint8_t>(std::clamp(info.cls, 0, 254));
        cmd.target_color = info.color;
        cmd.conf = static_cast<uint8_t>(std::clamp(
            static_cast<int>(std::lround(info.conf * 100.0f)), 0, 100));
        cmd.target_id = static_cast<int16_t>(info.track_id);
        cmd.bearing = static_cast<int16_t>(std::lround(info.bearing_deg * 10.0f));
        cmd.dist_mm = info.distance_m > 0.0f
                          ? static_cast<uint16_t>(std::lround(info.distance_m * 1000.0f))
                          : 0;
    }

    if (target.valid) cmd.flags |= FLAG_TARGET_VALID;
    if (!target.valid && (state_ == State::APPROACH || state_ == State::VERIFY)) {
        cmd.flags |= FLAG_TARGET_LOST;
    }
    if (state_ == State::APPROACH &&
        std::fabs(target.bearing_deg) > kTurnOnlyDeg) {
        cmd.flags |= FLAG_TURN_ONLY;
    }
    if (state_ == State::DONE && intent_.action == Action::APPROACH) {
        cmd.flags |= FLAG_VERIFY_OK;
    }

    return cmd;
}

}  // namespace sem
