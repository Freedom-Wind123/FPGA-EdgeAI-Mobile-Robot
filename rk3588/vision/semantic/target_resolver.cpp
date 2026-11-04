#include "target_resolver.h"

#include <algorithm>
#include <cmath>

namespace sem {

SemanticTarget TargetResolver::resolve(
        const TargetDesc& desc, const std::vector<SemanticObject>& objects) const {
    SemanticTarget best;
    const float max_dist = desc.max_distance_m > 0.0f ? desc.max_distance_m : 5.0f;

    for (const auto& o : objects) {
        if (!o.valid) continue;
        if (desc.cls >= 0 && o.cls != desc.cls) continue;

        // 颜色：匹配满分，未知降权，冲突直接排除
        float color_factor = 1.0f;
        if (desc.color != COLOR_ANY) {
            if (o.color == desc.color) {
                color_factor = 1.0f;
            } else if (o.color == COLOR_ANY) {
                color_factor = 0.55f;
            } else {
                continue;
            }
        }

        if (o.distance_m > 0.0f && o.distance_m > max_dist) continue;

        const float dist_score =
            o.distance_m > 0.0f ? 1.0f - std::min(1.0f, o.distance_m / max_dist) : 0.3f;

        float rel_score = 0.5f;
        switch (desc.relation) {
            case TargetDesc::NEAREST:
                rel_score = 1.0f - std::min(1.0f, std::fabs(o.bearing_deg) / 60.0f);
                break;
            case TargetDesc::LEFT:
                rel_score = o.bearing_deg < -5.0f ? 1.0f : 0.1f;
                break;
            case TargetDesc::RIGHT:
                rel_score = o.bearing_deg > 5.0f ? 1.0f : 0.1f;
                break;
            case TargetDesc::CENTER:
                rel_score = 1.0f - std::min(1.0f, std::fabs(o.bearing_deg) / 30.0f);
                break;
        }

        const float stable = std::min(1.0f, o.hits / 8.0f);
        const float score =
            (0.45f * dist_score + 0.25f * o.conf + 0.15f * rel_score + 0.15f * stable) *
            color_factor;

        if (score > best.score) {
            best.track_id = o.track_id;
            best.cls = o.cls;
            best.color = o.color;
            best.conf = o.conf;
            best.bearing_deg = o.bearing_deg;
            best.distance_m = o.distance_m;
            best.score = score;
            best.valid = true;
        }
    }

    return best;
}

}  // namespace sem
