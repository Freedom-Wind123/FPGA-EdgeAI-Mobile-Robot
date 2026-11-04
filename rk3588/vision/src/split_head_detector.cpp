#include "split_head_detector.h"
#include "detector_utils.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

SplitHeadDetector::SplitHeadDetector() = default;

SplitHeadDetector::~SplitHeadDetector() {
    if (ctx_) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

bool SplitHeadDetector::load(const std::string& path) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) {
        std::cerr << "open model failed: " << path << std::endl;
        return false;
    }
    std::fseek(fp, 0, SEEK_END);
    long size = std::ftell(fp);
    std::rewind(fp);
    if (size <= 0) {
        std::fclose(fp);
        return false;
    }
    std::vector<uint8_t> model((size_t)size);
    const size_t n = std::fread(model.data(), 1, model.size(), fp);
    std::fclose(fp);
    if (n != model.size()) {
        std::cerr << "read model failed: " << path << std::endl;
        return false;
    }
    int ret = rknn_init(&ctx_, model.data(), (uint32_t)model.size(), 0, nullptr);
    if (ret != 0) {
        std::cerr << "rknn_init failed: " << ret
                  << " (check librknnrt version / NPU driver)" << std::endl;
        return false;
    }
    return true;
}

bool SplitHeadDetector::init(const std::string& model_path, float conf_th,
                             float iou_th, int expected_class_count,
                             const std::string& aux_path) {
    (void)aux_path;
    conf_th_ = conf_th;
    iou_th_ = iou_th;

    if (!load(model_path)) return false;

    int ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != 0) {
        std::cerr << "rknn_query in/out num failed: " << ret << std::endl;
        return false;
    }
    if (io_num_.n_input != 1 || io_num_.n_output != 2) {
        std::cerr << "split-head 模型要求 1 输入 2 输出，实际 " << io_num_.n_input
                  << " 输入 " << io_num_.n_output << " 输出" << std::endl;
        return false;
    }

    std::memset(&input_attr_, 0, sizeof(input_attr_));
    input_attr_.index = 0;
    ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_));
    if (ret != 0 || input_attr_.n_dims != 4 || input_attr_.dims[0] != 1) {
        std::cerr << "split-head 模型输入属性不支持（需要 batch=1 的 4 维图像输入）"
                  << std::endl;
        return false;
    }
    if (input_attr_.fmt == RKNN_TENSOR_NHWC) {
        input_h_ = (int)input_attr_.dims[1];
        input_w_ = (int)input_attr_.dims[2];
        if (input_attr_.dims[3] != 3) {
            std::cerr << "split-head 模型输入通道不是 3" << std::endl;
            return false;
        }
    } else if (input_attr_.fmt == RKNN_TENSOR_NCHW) {
        if (input_attr_.dims[1] != 3) {
            std::cerr << "split-head 模型输入通道不是 3" << std::endl;
            return false;
        }
        input_h_ = (int)input_attr_.dims[2];
        input_w_ = (int)input_attr_.dims[3];
    } else {
        std::cerr << "split-head 模型输入 tensor 布局不支持" << std::endl;
        return false;
    }

    rknn_tensor_attr attrs[2];
    for (int i = 0; i < 2; ++i) {
        std::memset(&attrs[i], 0, sizeof(attrs[i]));
        attrs[i].index = (uint32_t)i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attrs[i], sizeof(attrs[i]));
        if (ret != 0) {
            std::cerr << "query output attr " << i << " failed: " << ret << std::endl;
            return false;
        }
    }

    // 4 通道的那个是 boxes，另一个是 scores
    if (attrs[0].n_dims == 3 && attrs[0].dims[1] == 4) {
        box_index_ = 0;
        score_index_ = 1;
    } else if (attrs[1].n_dims == 3 && attrs[1].dims[1] == 4) {
        box_index_ = 1;
        score_index_ = 0;
    } else {
        std::cerr << "split-head 模型输出布局不认识：找不到 4 通道框张量" << std::endl;
        return false;
    }
    box_attr_ = attrs[box_index_];
    score_attr_ = attrs[score_index_];

    if (score_attr_.n_dims != 3) {
        std::cerr << "split-head 分类输出维数不是 3" << std::endl;
        return false;
    }
    class_count_ = (int)score_attr_.dims[1];
    if (expected_class_count > 0 && class_count_ != expected_class_count) {
        std::cerr << "模型输出 " << class_count_ << " 类，但类别表有 "
                  << expected_class_count << " 行；请换匹配的类别表" << std::endl;
        return false;
    }
    if ((int)box_attr_.dims[2] != (int)score_attr_.dims[2]) {
        std::cerr << "split-head 框与分类的 anchor 数不一致" << std::endl;
        return false;
    }

    out_quant_ = (box_attr_.type == RKNN_TENSOR_INT8 ||
                  box_attr_.type == RKNN_TENSOR_UINT8 ||
                  box_attr_.type == RKNN_TENSOR_INT16);

    std::cout << "split-head 模型: " << input_w_ << "x" << input_h_
              << ", classes=" << class_count_
              << ", anchors=" << box_attr_.dims[2]
              << ", box_out type=" << (int)box_attr_.type
              << ", score_out type=" << (int)score_attr_.type << std::endl;

    inited_ = true;
    return true;
}

bool SplitHeadDetector::bindCore(int core_index) {
    if (!inited_ || ctx_ == 0) return false;
    rknn_core_mask mask = RKNN_NPU_CORE_AUTO;
    switch (core_index) {
        case 0: mask = RKNN_NPU_CORE_0; break;
        case 1: mask = RKNN_NPU_CORE_1; break;
        case 2: mask = RKNN_NPU_CORE_2; break;
        default: return false;
    }
    return rknn_set_core_mask(ctx_, mask) == RKNN_SUCC;
}

const float* SplitHeadDetector::dequant(const rknn_tensor_attr& attr, const void* src,
                                       int64_t elems, std::vector<float>& dst) const {
    if (attr.type == RKNN_TENSOR_FLOAT32) {
        return static_cast<const float*>(src);
    }
    dst.resize((size_t)elems);
    const float sc = attr.scale;
    const int32_t zp = attr.zp;
    if (attr.type == RKNN_TENSOR_INT8) {
        const int8_t* p = static_cast<const int8_t*>(src);
        for (int64_t i = 0; i < elems; ++i)
            dst[(size_t)i] = (float)((int32_t)p[i] - zp) * sc;
    } else if (attr.type == RKNN_TENSOR_UINT8) {
        const uint8_t* p = static_cast<const uint8_t*>(src);
        for (int64_t i = 0; i < elems; ++i)
            dst[(size_t)i] = (float)((int32_t)p[i] - zp) * sc;
    } else {
        const int16_t* p = static_cast<const int16_t*>(src);
        for (int64_t i = 0; i < elems; ++i)
            dst[(size_t)i] = (float)((int32_t)p[i] - zp) * sc;
    }
    return dst.data();
}

std::vector<Detection> SplitHeadDetector::infer(const cv::Mat& bgr) {
    if (!inited_ || bgr.empty()) return {};

    using namespace std::chrono;
    ++dbg_frame_;

    auto t0 = steady_clock::now();
    float scale = 1.0f;
    int pad_x = 0, pad_y = 0;
    det::letterbox(bgr, rgb_, input_w_, input_h_, scale, pad_x, pad_y);
    auto t1 = steady_clock::now();

    const bool nchw = (input_attr_.fmt == RKNN_TENSOR_NCHW);
    const uint8_t* input_data = rgb_.data;
    std::size_t input_bytes = rgb_.total() * rgb_.elemSize();
    if (nchw) {
        det::packImage(rgb_, input_w_, input_h_, true, packed_);
        input_data = packed_.data();
        input_bytes = packed_.size();
    }

    rknn_input input{};
    input.index = 0;
    input.buf = const_cast<uint8_t*>(input_data);
    input.size = (uint32_t)input_bytes;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = input_attr_.fmt;
    input.pass_through = 0;
    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != 0) {
        std::cerr << "rknn_inputs_set failed: " << ret << std::endl;
        return {};
    }
    ret = rknn_run(ctx_, nullptr);
    if (ret != 0) {
        std::cerr << "rknn_run failed: " << ret << std::endl;
        return {};
    }

    rknn_output outputs[2];
    std::memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < 2; ++i) {
        outputs[i].index = (uint32_t)i;
        outputs[i].want_float = out_quant_ ? 0 : 1;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(ctx_, 2, outputs, nullptr);
    if (ret != 0) {
        std::cerr << "rknn_outputs_get failed: " << ret << std::endl;
        return {};
    }
    auto t2 = steady_clock::now();

    const int64_t box_elems = (int64_t)box_attr_.n_elems;
    const int64_t score_elems = (int64_t)score_attr_.n_elems;
    const float* boxes = dequant(box_attr_, outputs[box_index_].buf, box_elems, box_buf_);
    const float* scores = dequant(score_attr_, outputs[score_index_].buf, score_elems,
                                  score_buf_);
    rknn_outputs_release(ctx_, 2, outputs);

    // YOLO11 的 2 输出模型不带 objectness；若分数是 logits（最大绝对值 > 1）则补 sigmoid
    const int na = (int)(box_elems / 4);
    float max_score = -1e9f;
    for (int64_t i = 0; i < score_elems; ++i) max_score = std::max(max_score, scores[i]);
    const bool use_sigmoid = (max_score > 1.0f);

    auto sigmoid = [](float v) { return 1.0f / (1.0f + std::exp(-std::min(std::max(v, -80.0f), 80.0f))); };

    // 框是归一化坐标还是像素坐标：YOLO11 导出通常是 0~1
    float box_scale = 1.0f;
    float max_abs = 0.0f;
    for (int64_t i = 0; i < box_elems; ++i) max_abs = std::max(max_abs, std::fabs(boxes[i]));
    const bool normalized = (max_abs <= 2.0f);
    if (normalized) box_scale = (float)input_w_;

    const float inv_scale = (scale > 0.0f) ? (1.0f / scale) : 1.0f;
    const int src_w = bgr.cols;
    const int src_h = bgr.rows;

    std::vector<Detection> dets;
    dets.reserve(32);
    for (int i = 0; i < na; ++i) {
        float best = -1e9f;
        int best_cls = -1;
        const float* row = scores + (int64_t)i;
        for (int c = 0; c < class_count_; ++c) {
            const float v = row[(int64_t)c * na];
            if (v > best) {
                best = v;
                best_cls = c;
            }
        }
        float conf = use_sigmoid ? sigmoid(best) : best;
        if (conf < conf_th_) continue;

        // boxes 是 [1,4,N] 通道优先布局，同一 anchor 的 4 个值分散在 4 个通道里
        const float cx = boxes[(int64_t)0 * na + i] * box_scale;
        const float cy = boxes[(int64_t)1 * na + i] * box_scale;
        const float bw = boxes[(int64_t)2 * na + i] * box_scale;
        const float bh = boxes[(int64_t)3 * na + i] * box_scale;

        float x1 = (cx - bw * 0.5f - pad_x) * inv_scale;
        float y1 = (cy - bh * 0.5f - pad_y) * inv_scale;
        float x2 = (cx + bw * 0.5f - pad_x) * inv_scale;
        float y2 = (cy + bh * 0.5f - pad_y) * inv_scale;
        x1 = std::min(std::max(x1, 0.0f), (float)(src_w - 1));
        y1 = std::min(std::max(y1, 0.0f), (float)(src_h - 1));
        x2 = std::min(std::max(x2, 0.0f), (float)(src_w - 1));
        y2 = std::min(std::max(y2, 0.0f), (float)(src_h - 1));
        if (x2 <= x1 || y2 <= y1) continue;

        Detection d;
        d.cls = best_cls;
        d.conf = conf;
        d.x1 = x1;
        d.y1 = y1;
        d.x2 = x2;
        d.y2 = y2;
        dets.push_back(d);
    }

    det::nms(dets, iou_th_);
    auto t3 = steady_clock::now();

    if (dbg_frame_ % 30 == 0) {
        auto ms = [](auto a, auto b) {
            return duration_cast<microseconds>(b - a).count() / 1000.0;
        };
        std::printf("[split] letterbox=%.1fms rknn=%.1fms post=%.1fms dets=%zu\n",
                    ms(t0, t1), ms(t1, t2), ms(t2, t3), dets.size());
        dbg_letterbox_ = ms(t0, t1);
        dbg_rknn_ = ms(t1, t2);
        dbg_post_ = ms(t2, t3);
        std::fflush(stdout);
    }
    return dets;
}
