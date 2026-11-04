#include "perception_adapter.h"

#include <algorithm>
#include <cmath>

namespace sem {
namespace {

// COCO 常见类别的近似物理高度，用于单目测距；自定义类别会退回默认值
float cocoHeight(int cls) {
    switch (cls) {
        case 0:  return 1.70f;   // person
        case 1:  return 1.00f;   // bicycle
        case 2:  return 1.50f;   // car
        case 15: return 0.30f;   // cat
        case 16: return 0.50f;   // dog
        case 24: return 0.45f;   // backpack
        case 32: return 0.22f;   // sports ball
        case 39: return 0.25f;   // bottle
        case 41: return 0.12f;   // cup
        case 45: return 0.08f;   // bowl
        case 46: return 0.20f;   // banana
        case 47: return 0.08f;   // apple
        case 49: return 0.08f;   // orange
        case 56: return 0.90f;   // chair
        case 57: return 0.80f;   // couch
        case 60: return 0.75f;   // dining table
        case 63: return 0.25f;   // laptop
        case 64: return 0.06f;   // mouse
        case 66: return 0.03f;   // keyboard
        case 67: return 0.15f;   // cell phone
        case 73: return 0.24f;   // book
        case 75: return 0.30f;   // vase
        case 77: return 0.30f;   // teddy bear
        default: return 0.0f;
    }
}

}  // namespace

PerceptionAdapter::PerceptionAdapter(const CameraParams& p) : params_(p) {
    if (params_.img_w <= 0) params_.img_w = 1280;
    if (params_.hfov_deg <= 0.0f) params_.hfov_deg = 50.0f;
    if (params_.fy_px <= 0.0f) params_.fy_px = 1000.0f;
    if (params_.default_obj_h_m <= 0.0f) params_.default_obj_h_m = 0.25f;
}

float PerceptionAdapter::objectHeight(int cls) const {
    if (params_.object_height_override_m > 0.0f) {
        return params_.object_height_override_m;
    }
    float h = cocoHeight(cls);
    return h > 0.0f ? h : params_.default_obj_h_m;
}

std::vector<ObjectObservation> PerceptionAdapter::convert(
        const std::vector<Detection>& dets, int64_t now_ms) const {
    std::vector<ObjectObservation> out;
    out.reserve(dets.size());

    for (const auto& d : dets) {
        if (d.track_id < 0) continue;

        ObjectObservation o;
        o.track_id = d.track_id;
        o.cls = d.cls;
        o.color = static_cast<uint8_t>(std::clamp(d.color, 0, 3));
        o.conf = std::clamp(d.conf, 0.0f, 1.0f);

        const float cx = (d.x1 + d.x2) * 0.5f;
        float norm = (cx - params_.img_w * 0.5f) / (params_.img_w * 0.5f);
        norm = std::clamp(norm, -1.0f, 1.0f);
        o.bearing_deg = norm * params_.hfov_deg * 0.5f;

        const float h_px = std::max(1.0f, d.y2 - d.y1);
        const float mono = params_.fy_px * objectHeight(d.cls) / h_px;
        o.distance_m = d.dist_m > 0.0f
                           ? d.dist_m
                           : ((mono >= 0.15f && mono <= 8.0f) ? mono : 0.0f);

        o.timestamp_ms = now_ms;
        out.push_back(o);
    }
    return out;
}

}  // namespace sem
