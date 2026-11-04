#pragma once

#include <cstdint>
#include <vector>

#include "types.h"
#include "semantic_types.h"

namespace sem {

// 相机参数，标定后修改
struct CameraParams {
    int img_w = 1280;
    float hfov_deg = 50.0f;          // 水平视场角
    float fy_px = 1000.0f;           // 竖直方向焦距（像素）
    float default_obj_h_m = 0.25f;   // 未知类别的物理高度
    float object_height_override_m = 0.0f; // 大于 0 时统一覆盖类别高度表
};

// 把视觉层 Detection 转成语义层观测
class PerceptionAdapter {
public:
    PerceptionAdapter() = default;
    explicit PerceptionAdapter(const CameraParams& p);

    void setParams(const CameraParams& p) { params_ = p; }
    void setImageWidth(int img_w) { if (img_w > 0) params_.img_w = img_w; }
    const CameraParams& params() const { return params_; }

    std::vector<ObjectObservation> convert(const std::vector<Detection>& dets,
                                           int64_t now_ms) const;

private:
    float objectHeight(int cls) const;

    CameraParams params_{};
};

}  // namespace sem
