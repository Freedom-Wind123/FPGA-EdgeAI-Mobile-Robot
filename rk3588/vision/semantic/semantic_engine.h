#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "types.h"

#include "action_interface.h"
#include "command_parser.h"
#include "perception_adapter.h"
#include "semantic_types.h"
#include "target_resolver.h"
#include "task_fsm.h"
#include "world_model.h"

class UartLink;

namespace sem {

// 语义层总入口：一句话指令 -> 目标表 -> 状态机 -> 串口命令
class SemanticEngine {
public:
    bool init(const std::vector<std::string>& classes,
              UartLink* uart,
              const CameraParams& cam);

    // 控制线程/主循环调用，解析并缓存指令
    bool submitText(const std::string& text);

    void setImageWidth(int img_w) { adapter_.setImageWidth(img_w); }

    // 搜索阶段自转角速度（rad/s），与主循环下发的 --search-wz 保持一致
    void setSearchAngularSpeed(float wz_rad_s) {
        fsm_.setSearchAngularSpeed(wz_rad_s);
    }

    // 到达判定距离（m），与主循环的 --stop-dist 保持一致
    void setStopDistance(float dist_m) { fsm_.setStopDistance(dist_m); }

    // 主循环判定已到位（框占比达到停车阈值）时通知状态机立即结束任务
    void notifyArrived(int64_t now_ms) { fsm_.markArrived(now_ms); }

    // 每帧视觉结果调用一次，内部完成状态机更新和 0xAA56 发送
    SemanticCommand update(const std::vector<Detection>& dets, int64_t now_ms);

    State state() const { return fsm_.state(); }
    const char* stateText() const { return stateName(fsm_.state()); }
    int selectedTrackId() const { return selected_track_id_; }
    SemanticCommand lastCommand() const { return last_cmd_; }

private:
    CommandParser parser_;
    PerceptionAdapter adapter_;
    WorldModel world_;
    TargetResolver resolver_;
    TaskFSM fsm_;
    ActionInterface action_;

    Intent pending_{};
    bool has_pending_ = false;

    SemanticCommand last_cmd_{};
    int selected_track_id_ = -1;
};

}  // namespace sem
