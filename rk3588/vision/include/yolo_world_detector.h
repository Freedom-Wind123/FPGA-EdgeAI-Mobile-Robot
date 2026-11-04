#pragma once

#include <rknn_api.h>

#include <string>
#include <vector>

#include "detector_base.h"

// YOLO-World（开放词表）检测器
// 模型结构：2 输入（图像 + 文本向量），6 输出（3 个尺度 × 分类/框）
class YoloWorldDetector : public IDetector {
public:
    enum class BoxMode {
        XYWH_PX,     // 框值已是输入图像像素坐标下的 cx,cy,w,h
        XYWH_NORM,   // 框值是 0~1 归一化的 cx,cy,w,h
        XYXY_PX,     // 框值是输入图像像素坐标下的 x1,y1,x2,y2
        XYXY_NORM,   // 框值是 0~1 归一化的 x1,y1,x2,y2
        LTRB_PX,     // 框值是相对 grid 中心的像素距离 l,t,r,b
        LTRB_STRIDE  // 框值是相对 grid 中心的 stride 单位距离
    };

    YoloWorldDetector() = default;
    ~YoloWorldDetector() override;

    // aux_path 为 texts.bin（80×512 float32 原始数据）
    bool init(const std::string& model_path, float conf_th, float iou_th,
              int expected_class_count, const std::string& aux_path) override;
    std::vector<Detection> infer(const cv::Mat& bgr) override;
    bool bindCore(int core_index) override;

    void setBoxMode(BoxMode mode) { box_mode_ = mode; }
    void setDebug(bool on) { debug_ = on; }
    void setApplySigmoid(bool on) { apply_sigmoid_ = on; }

private:
    struct Scale {
        int h = 0;
        int w = 0;
        int stride = 1;
        int cls_idx = -1;
        int box_idx = -1;
    };

    bool loadModel(const std::string& path);
    bool loadTexts(const std::string& path, int class_count, int dim);
    bool bindScales(int class_count);
    std::vector<Detection> decodeScale(const float* cls, const float* box,
                                       const Scale& s, float scale,
                                       int pad_x, int pad_y,
                                       int src_w, int src_h) const;

    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_{};
    rknn_tensor_attr image_attr_{};
    rknn_tensor_attr text_attr_{};
    int image_idx_ = -1;
    int text_idx_ = -1;
    int input_w_ = 640;
    int input_h_ = 640;
    int class_count_ = 80;
    std::vector<Scale> scales_;
    std::vector<float> texts_;

    float conf_th_ = 0.25f;
    float iou_th_ = 0.45f;
    BoxMode box_mode_ = BoxMode::XYWH_PX;
    bool apply_sigmoid_ = false;
    bool debug_ = false;
    bool debug_printed_ = false;
    mutable bool debug_box_printed_ = false;
    bool inited_ = false;

    // 复用缓冲：避免每帧重建 640x640 RGB 图与输入打包内存
    cv::Mat rgb_;
    std::vector<uint8_t> packed_;
};
