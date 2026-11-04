#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "semantic_types.h"

namespace sem {

// 按 track_id 维护短时目标表，屏蔽单帧漏检
class WorldModel {
public:
    WorldModel(int64_t ttl_ms = 1500, int64_t forget_ms = 5000)
        : ttl_ms_(ttl_ms), forget_ms_(forget_ms) {}

    void update(const std::vector<ObjectObservation>& obs, int64_t now_ms);

    // 只返回当前仍然有效的目标
    std::vector<SemanticObject> snapshot() const;

    size_t size() const { return objects_.size(); }

private:
    std::vector<SemanticObject> objects_;
    int64_t ttl_ms_;
    int64_t forget_ms_;
};

}  // namespace sem
