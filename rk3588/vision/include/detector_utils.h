#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <vector>

#include "types.h"

// 两种检测器共用的预处理与后处理，避免行为分叉
namespace det {

// 等比缩放 + 居中填充，输出 RGB 图
void letterbox(const cv::Mat& src, cv::Mat& rgb, int dst_w, int dst_h,
               float& scale, int& pad_x, int& pad_y);

// 把 RGB 图打包成模型输入：nchw=true 为平面布局，false 为 HWC
void packImage(const cv::Mat& rgb, int dst_w, int dst_h, bool nchw,
               std::vector<uint8_t>& out);

float iou(const Detection& a, const Detection& b);

// 按类别做 NMS，并按置信度降序保留
void nms(std::vector<Detection>& dets, float iou_th, std::size_t max_keep = 64);

}  // namespace det
