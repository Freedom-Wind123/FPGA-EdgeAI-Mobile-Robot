#include "detector.h"
#include "detector_utils.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>

Detector::Detector() = default;

Detector::~Detector() {
    if (ctx_) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

bool Detector::init(const std::string& model_path, float conf_th, float iou_th,
                    int expected_class_count, const std::string& aux_path) {
    (void)aux_path;  // YOLOv8 路线不需要额外输入
    conf_th_ = conf_th;
    iou_th_ = iou_th;

    if (!load(model_path)) return false;

    int ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != 0) {
        std::cerr << "rknn_query in/out num failed: " << ret << std::endl;
        return false;
    }
    // 支持两种 YOLOv8 导出格式：
    //   1) 单输出 [1, 4+nc, anchors]（ultralytics 默认导出）
    //   2) 三尺度共 9 个输出（瑞芯微 model zoo 导出，box/cls/score 分离）
    multi_head_ = (io_num_.n_output == 9);
    if (io_num_.n_input != 1 || (!multi_head_ && io_num_.n_output != 1)) {
        std::cerr << "unsupported RKNN model I/O: this detector requires 1 input and "
                  << "either 1 or 9 outputs, got " << io_num_.n_input << " input(s) and "
                  << io_num_.n_output << " output(s)" << std::endl;
        return false;
    }

    std::memset(&input_attr_, 0, sizeof(input_attr_));
    input_attr_.index = 0;
    ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_));
    if (ret != 0) {
        std::cerr << "rknn_query input attr failed: " << ret << std::endl;
        return false;
    }
    if (input_attr_.n_dims != 4 || input_attr_.dims[0] != 1) {
        std::cerr << "unsupported RKNN input shape: expected batch size 1 and a "
                  << "4-D image tensor" << std::endl;
        return false;
    }

    // 查询全部输出张量属性：单输出取 0 号，多输出需要逐个确认尺度关系
    out_attrs_.assign(io_num_.n_output, rknn_tensor_attr{});
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        out_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attrs_[i],
                         sizeof(rknn_tensor_attr));
        if (ret != 0) {
            std::cerr << "rknn_query output attr " << i << " failed: " << ret << std::endl;
            return false;
        }
    }
    output_attr_ = out_attrs_[0];

    int model_class_count = 0;
    if (multi_head_) {
        // 9 输出布局固定为 3 个尺度 × (box 64ch, cls nc, score 1ch)
        if ((int)out_attrs_[1].n_dims != 4 || (int)out_attrs_[1].dims[0] != 1) {
            std::cerr << "unsupported multi-head output shape" << std::endl;
            return false;
        }
        model_class_count = static_cast<int>(out_attrs_[1].dims[1]);
    } else {
        if (output_attr_.n_dims != 3 || output_attr_.dims[0] != 1) {
            std::cerr << "unsupported RKNN output shape: expected a decoded single-output "
                      << "YOLO tensor [1, 4+classes, anchors] or [1, anchors, 4+classes]"
                      << std::endl;
            return false;
        }

        const int dim_a = static_cast<int>(output_attr_.dims[1]);
        const int dim_b = static_cast<int>(output_attr_.dims[2]);
        const int channel_dim = std::min(dim_a, dim_b);
        if (channel_dim <= 4 || std::max(dim_a, dim_b) <= channel_dim) {
            std::cerr << "unsupported RKNN output dimensions: expected box and class "
                      << "channels followed by anchor predictions" << std::endl;
            return false;
        }
        model_class_count = channel_dim - 4;
    }
    if (expected_class_count > 0 && model_class_count != expected_class_count) {
        std::cerr << "RKNN output has " << model_class_count
                  << " classes, but classes file contains " << expected_class_count
                  << "; use a matching classes file or model" << std::endl;
        return false;
    }

    if (input_attr_.fmt == RKNN_TENSOR_NHWC) {
        input_h_ = input_attr_.dims[1];
        input_w_ = input_attr_.dims[2];
        if (input_attr_.dims[3] != 3) {
            std::cerr << "unsupported RKNN input channels: expected RGB (3)" << std::endl;
            return false;
        }
    } else if (input_attr_.fmt == RKNN_TENSOR_NCHW) {
        if (input_attr_.dims[1] != 3) {
            std::cerr << "unsupported RKNN input channels: expected RGB (3)" << std::endl;
            return false;
        }
        input_h_ = input_attr_.dims[2];
        input_w_ = input_attr_.dims[3];
    } else {
        std::cerr << "unsupported RKNN input tensor format" << std::endl;
        return false;
    }
    if (input_w_ <= 0 || input_h_ <= 0) {
        std::cerr << "invalid RKNN input dimensions" << std::endl;
        return false;
    }

    // 多输出（model zoo 三尺度）模式下统一让 runtime 转 float，解码逻辑最简单
    out_quant_ = !multi_head_ && (output_attr_.type == RKNN_TENSOR_INT8 ||
                                  output_attr_.type == RKNN_TENSOR_UINT8 ||
                                  output_attr_.type == RKNN_TENSOR_INT16);

    std::cout << "RKNN input: " << input_w_ << "x" << input_h_
              << ", in_type=" << (int)input_attr_.type
              << ", out_type=" << (int)output_attr_.type
              << ", classes: " << model_class_count
              << ", mode: " << (multi_head_ ? "multi-head(9)" : "single")
              << ", outputs: " << io_num_.n_output
              << ", output dims:";
    if (multi_head_) {
        std::cout << " box0=" << out_attrs_[0].dims[1] << "x" << out_attrs_[0].dims[2]
                  << "x" << out_attrs_[0].dims[3];
    } else {
        for (uint32_t i = 0; i < output_attr_.n_dims; ++i) {
            std::cout << " " << output_attr_.dims[i];
        }
    }
    std::cout << std::endl;

    inited_ = true;
    return true;
}

bool Detector::bindCore(int core_index) {
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

bool Detector::load(const std::string& path) {
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
    size_t n = std::fread(model.data(), 1, model.size(), fp);
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

std::vector<Detection> Detector::infer(const cv::Mat& bgr) {
    if (!inited_ || bgr.empty()) return {};

    using namespace std::chrono;
    ++dbg_frame_;

    auto ts0 = steady_clock::now();
    // ---- 预处理 ----
    float scale = 1.0f;
    int pad_x = 0, pad_y = 0;
    det::letterbox(bgr, rgb_, input_w_, input_h_, scale, pad_x, pad_y);
    auto ts1 = steady_clock::now();

    // NHWC 输入直接复用 rgb_ 的内存，省掉一次整图拷贝；NCHW 才需要重排
    const bool nchw = (input_attr_.fmt == RKNN_TENSOR_NCHW);
    const uint8_t* input_data = rgb_.data;
    std::size_t input_bytes = rgb_.total() * rgb_.elemSize();
    if (nchw) {
        det::packImage(rgb_, input_w_, input_h_, true, packed_);
        input_data = packed_.data();
        input_bytes = packed_.size();
    }
    auto ts2 = steady_clock::now();

    // ---- RKNN 推理 ----
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

    std::vector<rknn_output> outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        outputs[i].index = i;
        // 量化输出不请求 float 转换，取回原始整型数据后自己做反量化，
        // 省掉 runtime 每帧一次整块类型转换与临时分配
        outputs[i].want_float = out_quant_ ? 0 : 1;
        outputs[i].is_prealloc = 0;
    }

    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret != 0) {
        std::cerr << "rknn_outputs_get failed: " << ret << std::endl;
        return {};
    }
    auto ts3 = steady_clock::now();

    // ---- 后处理 ----
    std::vector<Detection> dets;
    if (multi_head_) {
        std::vector<const float*> ptrs(io_num_.n_output);
        for (uint32_t i = 0; i < io_num_.n_output; ++i) {
            ptrs[i] = static_cast<const float*>(outputs[i].buf);
        }
        dets = postprocessMultiHead(ptrs, scale, pad_x, pad_y, bgr.cols, bgr.rows);
    } else if (output_attr_.n_dims == 3) {
        int d1 = output_attr_.dims[1];
        int d2 = output_attr_.dims[2];
        bool channel_major = d1 < d2;
        int nc = (channel_major ? d1 : d2) - 4;
        int na = channel_major ? d2 : d1;

        const float* out = nullptr;
        if (out_quant_) {
            const int64_t n = (int64_t)output_attr_.n_elems;
            const float sc = output_attr_.scale;
            const int32_t zp = output_attr_.zp;
            if (n > 0 && sc != 0.0f) {
                deq_buf_.resize((size_t)n);
                if (output_attr_.type == RKNN_TENSOR_INT8) {
                    const int8_t* src = static_cast<const int8_t*>(outputs[0].buf);
                    for (int64_t i = 0; i < n; ++i)
                        deq_buf_[(size_t)i] = (float)((int32_t)src[i] - zp) * sc;
                } else if (output_attr_.type == RKNN_TENSOR_UINT8) {
                    const uint8_t* src = static_cast<const uint8_t*>(outputs[0].buf);
                    for (int64_t i = 0; i < n; ++i)
                        deq_buf_[(size_t)i] = (float)((int32_t)src[i] - zp) * sc;
                } else {
                    const int16_t* src = static_cast<const int16_t*>(outputs[0].buf);
                    for (int64_t i = 0; i < n; ++i)
                        deq_buf_[(size_t)i] = (float)((int32_t)src[i] - zp) * sc;
                }
                out = deq_buf_.data();
            } else {
                out = static_cast<const float*>(outputs[0].buf);
            }
        } else {
            out = static_cast<const float*>(outputs[0].buf);
        }
        if (nc > 0 && na > 0 && out) {
            dets = postprocess(out, nc, na, scale, pad_x, pad_y,
                               bgr.cols, bgr.rows, channel_major);
        }
    } else {
        std::cerr << "unsupported output dims: " << output_attr_.n_dims << std::endl;
    }
    auto ts4 = steady_clock::now();

    rknn_outputs_release(ctx_, io_num_.n_output, outputs.data());

    // ---- 累计并打印 ----
    dbg_letterbox_ += duration<double, std::milli>(ts1 - ts0).count();
    dbg_pack_      += duration<double, std::milli>(ts2 - ts1).count();
    dbg_rknn_      += duration<double, std::milli>(ts3 - ts2).count();
    dbg_post_      += duration<double, std::milli>(ts4 - ts3).count();

    if (dbg_frame_ % 30 == 0) {
        // 多上下文并行时统一串行输出，避免行内容交错
        static std::mutex print_mtx;
        std::lock_guard<std::mutex> lk(print_mtx);
        std::cout << "  [detector] letterbox=" << dbg_letterbox_ / 30.0
                  << "ms pack=" << dbg_pack_ / 30.0
                  << "ms rknn=" << dbg_rknn_ / 30.0
                  << "ms post=" << dbg_post_ / 30.0 << "ms" << std::endl;
        dbg_letterbox_ = dbg_pack_ = dbg_rknn_ = dbg_post_ = 0;
    }

    return dets;
}

std::vector<Detection> Detector::postprocess(const float* out, int nc, int na,
                                             float scale, int pad_x, int pad_y,
                                             int src_w, int src_h,
                                             bool channel_major) const {
    std::vector<Detection> dets;
    dets.reserve(128);

    const float inv_scale = 1.0f / scale;
    const float src_w_max = (float)src_w - 1;
    const float src_h_max = (float)src_h - 1;
    const int total_ch = 4 + nc;

    // 框解码 + 裁剪 + 收录，两种布局共用
    auto push = [&](int best, float score, float cx, float cy, float w, float h) {
        float x1 = (cx - w * 0.5f - pad_x) * inv_scale;
        float y1 = (cy - h * 0.5f - pad_y) * inv_scale;
        float x2 = (cx + w * 0.5f - pad_x) * inv_scale;
        float y2 = (cy + h * 0.5f - pad_y) * inv_scale;

        x1 = std::max(0.0f, std::min(x1, src_w_max));
        y1 = std::max(0.0f, std::min(y1, src_h_max));
        x2 = std::max(0.0f, std::min(x2, src_w_max));
        y2 = std::max(0.0f, std::min(y2, src_h_max));
        if (x2 <= x1 || y2 <= y1) return;

        Detection d;
        d.cls = best;
        d.conf = score;
        d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;
        dets.push_back(d);
    };

    if (channel_major) {
        // 通道优先布局 [1, 4+nc, na]：按通道连续扫描取最大类别分数。
        // 原写法每个 anchor 都要跨 na=8400 读取 80 个类别，缓存命中率极差；
        // 改成逐通道流式比较后整段内存只需顺序扫一遍。
        if ((int)score_buf_.size() != na) {
            score_buf_.resize((std::size_t)na);
            cls_buf_.resize((std::size_t)na);
        }
        const float* p0 = out + 4 * na;
        for (int i = 0; i < na; ++i) {
            score_buf_[i] = p0[i];
            cls_buf_[i] = 0;
        }
        for (int c = 1; c < nc; ++c) {
            const float* pc = out + (4 + c) * na;
            for (int i = 0; i < na; ++i) {
                const float s = pc[i];
                if (s > score_buf_[i]) {
                    score_buf_[i] = s;
                    cls_buf_[i] = (uint16_t)c;
                }
            }
        }
        for (int i = 0; i < na; ++i) {
            const float score = score_buf_[i];
            if (score < conf_th_) continue;
            push((int)cls_buf_[i], score,
                 out[i], out[na + i], out[2 * na + i], out[3 * na + i]);
            if (dets.size() >= 128) break;
        }
    } else {
        // 通道在后布局 [1, na, 4+nc]：每行连续，直接扫描
        for (int i = 0; i < na; ++i) {
            const float* row = out + i * total_ch;
            int best = 0;
            float score = row[4];
            for (int c = 1; c < nc; ++c) {
                const float s = row[4 + c];
                if (s > score) { score = s; best = c; }
            }
            if (score < conf_th_) continue;

            push(best, score, row[0], row[1], row[2], row[3]);
            if (dets.size() >= 128) break;
        }
    }

    det::nms(dets, iou_th_);
    return dets;
}

std::vector<Detection> Detector::postprocessMultiHead(const std::vector<const float*>& outs,
                                                      float scale, int pad_x, int pad_y,
                                                      int src_w, int src_h) const {
    std::vector<Detection> dets;
    dets.reserve(128);
    if (outs.size() < 9 || out_attrs_.size() < 9) return dets;

    const int nc = static_cast<int>(out_attrs_[1].dims[1]);
    const float inv_scale = 1.0f / scale;
    const float src_w_max = (float)src_w - 1;
    const float src_h_max = (float)src_h - 1;

    for (int b = 0; b < 3; ++b) {
        const int h = static_cast<int>(out_attrs_[3 * b].dims[2]);
        const int w = static_cast<int>(out_attrs_[3 * b].dims[3]);
        if (h <= 0 || w <= 0) continue;
        const int stride = input_w_ / w;   // 80/40/20 -> 8/16/32
        if (stride <= 0) continue;
        const int hw = h * w;
        const float* box = outs[3 * b];
        const float* cls = outs[3 * b + 1];
        const float* obj_p = outs[3 * b + 2];

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const int idx = y * w + x;
                // 模型内已做 sigmoid，这里拿到的是 0~1 的概率
                const float obj = obj_p[idx];
                if (obj < conf_th_ * 0.5f) continue;

                int best = 0;
                float bestv = cls[idx];
                for (int c = 1; c < nc; ++c) {
                    const float v = cls[c * hw + idx];
                    if (v > bestv) { bestv = v; best = c; }
                }
                const float prob = bestv * obj;
                if (prob < conf_th_) continue;

                // DFL：对每条边的 16 个 bin 做 softmax 后加权求期望距离（单位：格）
                float dist[4];
                for (int s = 0; s < 4; ++s) {
                    const float* p = box + (s * 16) * hw + idx;
                    float mx = p[0];
                    for (int k = 1; k < 16; ++k) {
                        const float v = p[k * hw];
                        if (v > mx) mx = v;
                    }
                    float sum = 0.0f;
                    float acc = 0.0f;
                    for (int k = 0; k < 16; ++k) {
                        const float e = std::exp(p[k * hw] - mx);
                        sum += e;
                        acc += e * (float)k;
                    }
                    dist[s] = (sum > 0.0f) ? acc / sum : 0.0f;
                }

                const float cx = ((float)x + 0.5f) * (float)stride;
                const float cy = ((float)y + 0.5f) * (float)stride;
                float x1 = (cx - dist[0] * stride - pad_x) * inv_scale;
                float y1 = (cy - dist[1] * stride - pad_y) * inv_scale;
                float x2 = (cx + dist[2] * stride - pad_x) * inv_scale;
                float y2 = (cy + dist[3] * stride - pad_y) * inv_scale;

                x1 = std::max(0.0f, std::min(x1, src_w_max));
                y1 = std::max(0.0f, std::min(y1, src_h_max));
                x2 = std::max(0.0f, std::min(x2, src_w_max));
                y2 = std::max(0.0f, std::min(y2, src_h_max));
                if (x2 <= x1 || y2 <= y1) continue;

                Detection d;
                d.cls = best;
                d.conf = prob;
                d.x1 = x1; d.y1 = y1; d.x2 = x2; d.y2 = y2;
                dets.push_back(d);
                if (dets.size() >= 128) break;
            }
            if (dets.size() >= 128) break;
        }
        if (dets.size() >= 128) break;
    }

    det::nms(dets, iou_th_);
    return dets;
}
