#pragma once

#include <vector>

#include "semantic_types.h"

namespace sem {

// 按类别、颜色、距离、方位、稳定度给候选目标打分，取最高分
class TargetResolver {
public:
    SemanticTarget resolve(const TargetDesc& desc,
                           const std::vector<SemanticObject>& objects) const;
};

}  // namespace sem
