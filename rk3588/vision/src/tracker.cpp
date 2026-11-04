#include "tracker.h"

#include <algorithm>

Tracker::Tracker(float iou_th, int max_age)
    : iou_th_(iou_th), max_age_(max_age) {}

void Tracker::update(std::vector<Detection>& dets) {
    for (auto& t : tracks_) t.age++;

    std::vector<bool> used(tracks_.size(), false);

    for (auto& d : dets) {
        int best = -1;
        float best_iou = iou_th_;

        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (used[i]) continue;
            if (tracks_[i].det.cls != d.cls) continue;
            float v = iou(tracks_[i].det, d);
            if (v > best_iou) {
                best_iou = v;
                best = (int)i;
            }
        }

        if (best >= 0) {
            d.track_id = tracks_[best].id;
            tracks_[best].det = d;
            tracks_[best].age = 0;
            used[best] = true;
        } else {
            Track t;
            t.id = next_id_++;
            t.det = d;
            t.age = 0;
            d.track_id = t.id;
            tracks_.push_back(t);
            used.push_back(true);
        }
    }

    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const Track& t) { return t.age > max_age_; }),
        tracks_.end());
}

float Tracker::iou(const Detection& a, const Detection& b) {
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;
    float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}
