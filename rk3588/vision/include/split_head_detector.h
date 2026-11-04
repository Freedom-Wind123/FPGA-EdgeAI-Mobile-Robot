#pragma once

#include <opencv2/core.hpp>
#include <rknn_api.h>

#include <string>
#include <vector>

#include "detector_base.h"
#include "types.h"

// YOLO11 风格的双输出 RKNN 模型：
//   out0 = [1, 4, N]    归一化 cxcywh 框
//   out1 = [1, nc, N]   每类置信度（无 objectness）
// 社区场景模型（垃圾桶/红绿灯/消防/社区人员/停车位）都是这种布局
class SplitHeadDetector : public IDetector {
public:
    SplitHeadDetector();
    ~SplitHeadDetector() override;

    bool init(const std::string& model_path, float conf_th, float iou_th,
              int expected_class_count,
              const std::string& aux_path = std::string()) override;
    std::vector<Detection> infer(const cv::Mat& bgr) override;
    bool bindCore(int core_index) override;

private:
    bool load(const std::string& path);
    // 量化输出就地反量化到 float 缓冲
    const float* dequant(const rknn_tensor_attr& attr, const void* src,
                         int64_t elems, std::vector<float>& dst) const;

    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_{};
    rknn_tensor_attr input_attr_{};
    rknn_tensor_attr box_attr_{};
    rknn_tensor_attr score_attr_{};
    int box_index_ = 0;
    int score_index_ = 1;
    int input_w_ = 640;
    int input_h_ = 640;
    int class_count_ = 0;
    float conf_th_ = 0.25f;
    float iou_th_ = 0.45f;
    bool inited_ = false;
    bool out_quant_ = false;

    cv::Mat rgb_;
    std::vector<uint8_t> packed_;
    std::vector<float> box_buf_;
    std::vector<float> score_buf_;

    int dbg_frame_ = 0;
    double dbg_letterbox_ = 0.0;
    double dbg_rknn_ = 0.0;
    double dbg_post_ = 0.0;
};
