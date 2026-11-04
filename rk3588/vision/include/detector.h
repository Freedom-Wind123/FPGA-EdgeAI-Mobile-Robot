#pragma once

#include <opencv2/core.hpp>
#include <rknn_api.h>

#include <string>
#include <vector>

#include "detector_base.h"
#include "types.h"

// YOLOv8 单输入单输出 RKNN 推理
class Detector : public IDetector {
public:
    Detector();
    ~Detector() override;

    bool init(const std::string& model_path, float conf_th, float iou_th,
              int expected_class_count,
              const std::string& aux_path = std::string()) override;
    std::vector<Detection> infer(const cv::Mat& bgr) override;
    bool bindCore(int core_index) override;

private:
    bool load(const std::string& path);

    std::vector<Detection> postprocess(const float* out, int nc, int na,
                                       float scale, int pad_x, int pad_y,
                                       int src_w, int src_h,
                                       bool channel_major) const;

    // 三尺度 9 输出（box/cls/score 分离）的解码，用于瑞芯微 model zoo 导出的模型
    std::vector<Detection> postprocessMultiHead(const std::vector<const float*>& outs,
                                                float scale, int pad_x, int pad_y,
                                                int src_w, int src_h) const;

    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_{};
    rknn_tensor_attr input_attr_{};
    rknn_tensor_attr output_attr_{};
    std::vector<rknn_tensor_attr> out_attrs_;
    // true 表示 3 尺度 × (box/cls/score) 共 9 个输出的 model zoo 导出格式
    bool multi_head_ = false;
    int input_w_ = 640;
    int input_h_ = 640;
    float conf_th_ = 0.25f;
    float iou_th_ = 0.45f;
    bool inited_ = false;
    // 输出是否为量化类型：量化模型不请求 float 转换，自己做反量化
    bool out_quant_ = false;

    // 复用缓冲：避免每帧重建 640x640 RGB 图与输入打包内存
    cv::Mat rgb_;
    std::vector<uint8_t> packed_;
    std::vector<float> deq_buf_;
    // 后处理流式统计缓冲（每个上下文独占，无需加锁）
    mutable std::vector<float> score_buf_;
    mutable std::vector<uint16_t> cls_buf_;
    // 分段耗时统计（每 30 帧打印一次）
    int dbg_frame_ = 0;
    double dbg_letterbox_ = 0.0;
    double dbg_pack_ = 0.0;
    double dbg_rknn_ = 0.0;
    double dbg_post_ = 0.0;
};
