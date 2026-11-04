#include "detector_utils.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace det {

void letterbox(const cv::Mat& src, cv::Mat& rgb, int dst_w, int dst_h,
               float& scale, int& pad_x, int& pad_y) {
    scale = std::min(dst_w / (float)src.cols, dst_h / (float)src.rows);
    int nw = (int)std::round(src.cols * scale);
    int nh = (int)std::round(src.rows * scale);
    pad_x = (dst_w - nw) / 2;
    pad_y = (dst_h - nh) / 2;

    // canvas 按线程复用：只在分辨率变化或填充区变化时重涂灰边，
    // 固定 1280x720 输入下每帧省掉一次 1.2MB 的全画布填充与内存分配
    static thread_local cv::Mat canvas;
    static thread_local cv::Rect last_roi;
    if (canvas.cols != dst_w || canvas.rows != dst_h ||
        canvas.type() != CV_8UC3) {
        canvas.create(dst_h, dst_w, CV_8UC3);
        last_roi = cv::Rect();
    }
    const cv::Rect roi(pad_x, pad_y, nw, nh);
    if (roi != last_roi) {
        canvas.setTo(cv::Scalar(114, 114, 114));
        last_roi = roi;
    }

    cv::resize(src, canvas(roi), roi.size());
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
}

void packImage(const cv::Mat& rgb, int dst_w, int dst_h, bool nchw,
               std::vector<uint8_t>& out) {
    if (!nchw) {
        // NHWC：直接 memcpy，最快路径
        out.assign(rgb.data, rgb.data + (std::size_t)rgb.total() * rgb.elemSize());
        return;
    }

    // NCHW：用 cv::split（NEON SIMD 加速）替代三重 for 循环
    const std::size_t plane = (std::size_t)dst_w * dst_h;
    out.resize(plane * 3);
    cv::Mat channels[3];
    cv::split(rgb, channels);
    for (int c = 0; c < 3; ++c) {
        std::memcpy(out.data() + (std::size_t)c * plane, channels[c].data, plane);
    }
}

float iou(const Detection& a, const Detection& b) {
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

void nms(std::vector<Detection>& dets, float iou_th, std::size_t max_keep) {
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) {
                  return a.conf > b.conf;
              });

    std::vector<Detection> keep;
    std::vector<bool> removed(dets.size(), false);

    for (std::size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        keep.push_back(dets[i]);
        for (std::size_t j = i + 1; j < dets.size(); ++j) {
            if (removed[j]) continue;
            if (dets[i].cls != dets[j].cls) continue;
            if (iou(dets[i], dets[j]) > iou_th) removed[j] = true;
        }
        if (keep.size() >= max_keep) break;
    }
    dets.swap(keep);
}

}  // namespace det
