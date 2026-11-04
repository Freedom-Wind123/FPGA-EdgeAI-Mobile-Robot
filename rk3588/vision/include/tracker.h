#pragma once

#include <vector>

#include "types.h"

// 轻量 IoU 跟踪器：先保证 ID 稳定，后续可替换成 ByteTrack
class Tracker {
public:
    explicit Tracker(float iou_th = 0.3f, int max_age = 30);

    void update(std::vector<Detection>& dets);

private:
    struct Track {
        int id = -1;
        Detection det;
        int age = 0;
    };

    static float iou(const Detection& a, const Detection& b);

    std::vector<Track> tracks_;
    int next_id_ = 1;
    float iou_th_;
    int max_age_;
};
