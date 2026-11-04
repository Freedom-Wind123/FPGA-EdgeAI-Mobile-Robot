#include "world_model.h"

#include <algorithm>

namespace sem {

void WorldModel::update(const std::vector<ObjectObservation>& obs, int64_t now_ms) {
    constexpr float kAlpha = 0.4f;

    for (const auto& o : obs) {
        if (o.track_id < 0) continue;

        auto it = std::find_if(objects_.begin(), objects_.end(),
                               [&](const SemanticObject& s) {
                                   return s.track_id == o.track_id;
                               });
        if (it == objects_.end()) {
            SemanticObject s;
            s.track_id = o.track_id;
            s.cls = o.cls;
            s.color = o.color;
            s.conf = o.conf;
            s.bearing_deg = o.bearing_deg;
            s.distance_m = o.distance_m;
            s.hits = 1;
            s.last_seen_ms = now_ms;
            s.valid = true;
            objects_.push_back(s);
            continue;
        }

        it->bearing_deg = it->bearing_deg * (1.0f - kAlpha) + o.bearing_deg * kAlpha;
        it->conf = it->conf * (1.0f - kAlpha) + o.conf * kAlpha;
        if (o.distance_m > 0.0f) {
            it->distance_m = (it->distance_m > 0.0f)
                                 ? it->distance_m * (1.0f - kAlpha) + o.distance_m * kAlpha
                                 : o.distance_m;
        }
        if (o.color != COLOR_ANY) it->color = o.color;
        if (o.cls >= 0) it->cls = o.cls;
        it->hits += 1;
        it->last_seen_ms = now_ms;
    }

    for (auto& s : objects_) {
        s.valid = (now_ms - s.last_seen_ms) <= ttl_ms_;
    }

    objects_.erase(std::remove_if(objects_.begin(), objects_.end(),
                                  [&](const SemanticObject& s) {
                                      return (now_ms - s.last_seen_ms) > forget_ms_;
                                  }),
                   objects_.end());
}

std::vector<SemanticObject> WorldModel::snapshot() const {
    std::vector<SemanticObject> out;
    out.reserve(objects_.size());
    for (const auto& s : objects_) {
        if (s.valid) out.push_back(s);
    }
    return out;
}

}  // namespace sem
