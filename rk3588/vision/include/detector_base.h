#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "types.h"

// 检测器统一接口：主流程只依赖这一层，模型路线可切换
class IDetector {
public:
    virtual ~IDetector() = default;

    // aux_path 由具体实现解释：YOLOv8 忽略，YOLO-World 用作 texts.bin
    virtual bool init(const std::string& model_path, float conf_th, float iou_th,
                      int expected_class_count,
                      const std::string& aux_path) = 0;

    virtual std::vector<Detection> infer(const cv::Mat& bgr) = 0;

    // 绑定到指定 NPU 核（0/1/2），实现不支持时返回 false 由驱动自行调度
    virtual bool bindCore(int core_index) {
        (void)core_index;
        return false;
    }
};
