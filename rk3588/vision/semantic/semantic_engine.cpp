#include "semantic_engine.h"

#include <iostream>

namespace sem {

bool SemanticEngine::init(const std::vector<std::string>& classes,
                          UartLink* uart,
                          const CameraParams& cam) {
    parser_.setClasses(classes);
    adapter_.setParams(cam);
    action_.setUart(uart);
    world_ = WorldModel();
    fsm_ = TaskFSM();
    pending_ = Intent{};
    has_pending_ = false;
    last_cmd_ = SemanticCommand{};
    selected_track_id_ = -1;
    return true;
}

bool SemanticEngine::submitText(const std::string& text) {
    Intent intent;
    if (!parser_.parse(text, intent)) return false;
    pending_ = intent;
    has_pending_ = true;
    return true;
}

SemanticCommand SemanticEngine::update(const std::vector<Detection>& dets,
                                       int64_t now_ms) {
    if (has_pending_) {
        fsm_.setIntent(pending_, now_ms);
        has_pending_ = false;
        std::cout << "[semantic] 指令 " << actionName(pending_.action)
                  << " cls=" << pending_.target.cls
                  << " color=" << static_cast<int>(pending_.target.color)
                  << " -> " << stateName(fsm_.state()) << std::endl;
    }

    const std::vector<ObjectObservation> obs = adapter_.convert(dets, now_ms);
    world_.update(obs, now_ms);
    const std::vector<SemanticObject> objects = world_.snapshot();

    const SemanticTarget target = resolver_.resolve(fsm_.intent().target, objects);

    const State before = fsm_.state();
    SemanticCommand cmd = fsm_.tick(target, now_ms);
    if (fsm_.state() != before) {
        std::cout << "[semantic] " << stateName(before) << " -> "
                  << stateName(fsm_.state()) << std::endl;
    }

    action_.publish(cmd, now_ms);

    last_cmd_ = cmd;
    selected_track_id_ = (cmd.flags & FLAG_TARGET_VALID) ? cmd.target_id : -1;
    return cmd;
}

}  // namespace sem
