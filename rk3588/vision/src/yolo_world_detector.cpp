#include "yolo_world_detector.h"

#include "detector_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

YoloWorldDetector::~YoloWorldDetector() {
    if (ctx_) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

bool YoloWorldDetector::loadModel(const std::string& path) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) {
        std::cerr << "open world model failed: " << path << std::endl;
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
        std::cerr << "read world model failed: " << path << std::endl;
        return false;
    }

    int ret = rknn_init(&ctx_, model.data(), (uint32_t)model.size(), 0, nullptr);
    if (ret != 0) {
        std::cerr << "rknn_init world model failed: " << ret
                  << " (check librknnrt version / NPU driver)" << std::endl;
        return false;
    }
    return true;
}

bool YoloWorldDetector::loadTexts(const std::string& path, int class_count,
                                  int dim) {
    const std::size_t expected =
        (std::size_t)class_count * (std::size_t)dim * sizeof(float);
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) {
        std::cerr << "open texts file failed: " << path
                  << "\n  YOLO-World 需要文本向量文件，请先在板子上运行:"
                  << "\n  python3 tools/gen_texts_bin.py --clip-model <clip_text_fp16.rknn>"
                  << " --classes classes.txt --out " << path << std::endl;
        return false;
    }
    std::fseek(fp, 0, SEEK_END);
    long size = std::ftell(fp);
    std::rewind(fp);
    if (size <= 0 || (std::size_t)size != expected) {
        std::fclose(fp);
        std::cerr << "texts file size mismatch: " << path << " is " << size
                  << " bytes, expected " << expected << " (" << class_count
                  << "x" << dim << " float32)" << std::endl;
        return false;
    }

    texts_.resize((std::size_t)class_count * dim);
    size_t n = std::fread(texts_.data(), sizeof(float), texts_.size(), fp);
    std::fclose(fp);
    if (n != texts_.size()) {
        std::cerr << "read texts file failed: " << path << std::endl;
        return false;
    }
    return true;
}

bool YoloWorldDetector::init(const std::string& model_path, float conf_th,
                             float iou_th, int expected_class_count,
                             const std::string& aux_path) {
    conf_th_ = conf_th;
    iou_th_ = iou_th;

    if (aux_path.empty()) {
        std::cerr << "YOLO-World 需要指定 texts.bin 路径" << std::endl;
        return false;
    }
    if (!loadModel(model_path)) return false;

    int ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != 0) {
        std::cerr << "rknn_query in/out num failed: " << ret << std::endl;
        return false;
    }
    if (io_num_.n_input != 2) {
        std::cerr << "unsupported world model: expected 2 inputs (image + texts), got "
                  << io_num_.n_input << std::endl;
        return false;
    }

    for (uint32_t i = 0; i < io_num_.n_input; ++i) {
        rknn_tensor_attr attr{};
        attr.index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (ret != 0) {
            std::cerr << "rknn_query input attr failed: " << ret << std::endl;
            return false;
        }
        if (attr.n_dims == 4) {
            image_attr_ = attr;
            image_idx_ = (int)i;
        } else if (attr.n_dims == 3) {
            text_attr_ = attr;
            text_idx_ = (int)i;
        }
    }
    if (image_idx_ < 0 || text_idx_ < 0) {
        std::cerr << "unsupported world model inputs: need one 4-D image tensor "
                  << "and one 3-D text tensor" << std::endl;
        return false;
    }

    if (image_attr_.fmt == RKNN_TENSOR_NHWC) {
        input_h_ = image_attr_.dims[1];
        input_w_ = image_attr_.dims[2];
        if (image_attr_.dims[3] != 3) {
            std::cerr << "unsupported world image channels" << std::endl;
            return false;
        }
    } else if (image_attr_.fmt == RKNN_TENSOR_NCHW) {
        if (image_attr_.dims[1] != 3) {
            std::cerr << "unsupported world image channels" << std::endl;
            return false;
        }
        input_h_ = image_attr_.dims[2];
        input_w_ = image_attr_.dims[3];
    } else {
        std::cerr << "unsupported world image tensor format" << std::endl;
        return false;
    }
    if (input_w_ <= 0 || input_h_ <= 0) {
        std::cerr << "invalid world image dimensions" << std::endl;
        return false;
    }

    if (text_attr_.dims[0] != 1) {
        std::cerr << "unsupported world text batch size" << std::endl;
        return false;
    }
    class_count_ = (int)text_attr_.dims[1];
    const int text_dim = (int)text_attr_.dims[2];
    if (expected_class_count > 0 && class_count_ != expected_class_count) {
        std::cerr << "world text input has " << class_count_
                  << " classes, but classes file contains "
                  << expected_class_count << std::endl;
        return false;
    }

    if (!loadTexts(aux_path, class_count_, text_dim)) return false;
    if (!bindScales(class_count_)) return false;

    std::cout << "YOLO-World input: " << input_w_ << "x" << input_h_
              << ", classes: " << class_count_ << ", text dim: " << text_dim
              << ", scales: " << scales_.size() << std::endl;
    inited_ = true;
    return true;
}

bool YoloWorldDetector::bindScales(int class_count) {
    std::vector<rknn_tensor_attr> outs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        outs[i] = rknn_tensor_attr{};
        outs[i].index = i;
        int ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &outs[i],
                             sizeof(outs[i]));
        if (ret != 0) {
            std::cerr << "rknn_query output attr failed: " << ret << std::endl;
            return false;
        }
    }

    std::vector<bool> used(outs.size(), false);
    for (std::size_t i = 0; i < outs.size(); ++i) {
        const auto& cls = outs[i];
        if (used[i] || cls.n_dims != 4 || (int)cls.dims[1] != class_count) continue;

        int box_idx = -1;
        for (std::size_t j = 0; j < outs.size(); ++j) {
            const auto& box = outs[j];
            if (used[j] || box.n_dims != 4 || (int)box.dims[1] != 4) continue;
            if (box.dims[2] != cls.dims[2] || box.dims[3] != cls.dims[3]) continue;
            box_idx = (int)j;
            break;
        }
        if (box_idx < 0) continue;

        if (cls.fmt != RKNN_TENSOR_NCHW || outs[box_idx].fmt != RKNN_TENSOR_NCHW) {
            std::cerr << "unsupported world output layout: expected NCHW" << std::endl;
            return false;
        }

        Scale s;
        s.h = (int)cls.dims[2];
        s.w = (int)cls.dims[3];
        s.stride = input_w_ / s.w;
        s.cls_idx = (int)i;
        s.box_idx = box_idx;
        if (s.stride <= 0 || input_w_ % s.w != 0 ||
            input_h_ / s.h != s.stride) {
            std::cerr << "unexpected world output scale: " << s.w << "x" << s.h
                      << std::endl;
            return false;
        }

        used[i] = true;
        used[box_idx] = true;
        scales_.push_back(s);
    }

    if (scales_.empty()) {
        std::cerr << "no valid detection scales found in world model outputs"
                  << std::endl;
        return false;
    }
    std::sort(scales_.begin(), scales_.end(),
              [](const Scale& a, const Scale& b) { return a.stride < b.stride; });
    return true;
}

std::vector<Detection> YoloWorldDetector::decodeScale(
    const float* cls, const float* box, const Scale& s, float scale,
    int pad_x, int pad_y, int src_w, int src_h) const {
    std::vector<Detection> dets;
    const int HW = s.h * s.w;

    for (int y = 0; y < s.h; ++y) {
        for (int x = 0; x < s.w; ++x) {
            const int idx = y * s.w + x;

            float raw = cls[idx];
            int best = 0;
            for (int c = 1; c < class_count_; ++c) {
                const float v = cls[(std::size_t)c * HW + idx];
                if (v > raw) {
                    raw = v;
                    best = c;
                }
            }
            const float score = apply_sigmoid_
                                    ? 1.0f / (1.0f + std::exp(-raw))
                                    : raw;
            if (score < conf_th_) continue;

            const float b0 = box[idx];
            const float b1 = box[(std::size_t)HW + idx];
            const float b2 = box[(std::size_t)2 * HW + idx];
            const float b3 = box[(std::size_t)3 * HW + idx];

            if (debug_ && !debug_box_printed_) {
                debug_box_printed_ = true;
                std::cout << "world raw box @stride " << s.stride << ": "
                          << b0 << " " << b1 << " " << b2 << " " << b3
                          << " (grid " << x << "," << y << ", raw score " << raw
                          << ")" << std::endl;
            }

            float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
            const float cx_grid = (x + 0.5f) * s.stride;
            const float cy_grid = (y + 0.5f) * s.stride;

            switch (box_mode_) {
                case BoxMode::XYWH_PX: {
                    x1 = b0 - b2 * 0.5f;
                    y1 = b1 - b3 * 0.5f;
                    x2 = b0 + b2 * 0.5f;
                    y2 = b1 + b3 * 0.5f;
                    break;
                }
                case BoxMode::XYWH_NORM: {
                    const float cx = b0 * input_w_;
                    const float cy = b1 * input_h_;
                    const float w = b2 * input_w_;
                    const float h = b3 * input_h_;
                    x1 = cx - w * 0.5f;
                    y1 = cy - h * 0.5f;
                    x2 = cx + w * 0.5f;
                    y2 = cy + h * 0.5f;
                    break;
                }
                case BoxMode::XYXY_PX: {
                    x1 = b0;
                    y1 = b1;
                    x2 = b2;
                    y2 = b3;
                    break;
                }
                case BoxMode::XYXY_NORM: {
                    x1 = b0 * input_w_;
                    y1 = b1 * input_h_;
                    x2 = b2 * input_w_;
                    y2 = b3 * input_h_;
                    break;
                }
                case BoxMode::LTRB_PX: {
                    x1 = cx_grid - b0;
                    y1 = cy_grid - b1;
                    x2 = cx_grid + b2;
                    y2 = cy_grid + b3;
                    break;
                }
                case BoxMode::LTRB_STRIDE: {
                    x1 = cx_grid - b0 * s.stride;
                    y1 = cy_grid - b1 * s.stride;
                    x2 = cx_grid + b2 * s.stride;
                    y2 = cy_grid + b3 * s.stride;
                    break;
                }
            }

            Detection d;
            d.cls = best;
            d.conf = score;
            d.x1 = (x1 - pad_x) / scale;
            d.y1 = (y1 - pad_y) / scale;
            d.x2 = (x2 - pad_x) / scale;
            d.y2 = (y2 - pad_y) / scale;

            d.x1 = std::max(0.0f, std::min(d.x1, (float)src_w - 1));
            d.y1 = std::max(0.0f, std::min(d.y1, (float)src_h - 1));
            d.x2 = std::max(0.0f, std::min(d.x2, (float)src_w - 1));
            d.y2 = std::max(0.0f, std::min(d.y2, (float)src_h - 1));
            if (d.x2 <= d.x1 || d.y2 <= d.y1) continue;

            dets.push_back(d);
            if (dets.size() >= 512) return dets;
        }
    }
    return dets;
}

bool YoloWorldDetector::bindCore(int core_index) {
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

std::vector<Detection> YoloWorldDetector::infer(const cv::Mat& bgr) {
    if (!inited_ || bgr.empty()) return {};

    float scale = 1.0f;
    int pad_x = 0, pad_y = 0;
    det::letterbox(bgr, rgb_, input_w_, input_h_, scale, pad_x, pad_y);

    // NHWC 直接复用 rgb_ 内存；NCHW 才需要重排，缓冲同样复用
    const bool nchw = (image_attr_.fmt == RKNN_TENSOR_NCHW);
    const uint8_t* image_data = rgb_.data;
    std::size_t image_bytes = rgb_.total() * rgb_.elemSize();
    if (nchw) {
        det::packImage(rgb_, input_w_, input_h_, true, packed_);
        image_data = packed_.data();
        image_bytes = packed_.size();
    }

    rknn_input inputs[2]{};
    inputs[0].index = (uint32_t)image_idx_;
    inputs[0].buf = const_cast<uint8_t*>(image_data);
    inputs[0].size = (uint32_t)image_bytes;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = image_attr_.fmt;
    inputs[0].pass_through = 0;

    inputs[1].index = (uint32_t)text_idx_;
    inputs[1].buf = texts_.data();
    inputs[1].size = (uint32_t)(texts_.size() * sizeof(float));
    inputs[1].type = RKNN_TENSOR_FLOAT32;
    inputs[1].fmt = text_attr_.fmt;
    inputs[1].pass_through = 0;

    int ret = rknn_inputs_set(ctx_, 2, inputs);
    if (ret != 0) {
        std::cerr << "rknn_inputs_set(world) failed: " << ret << std::endl;
        return {};
    }

    ret = rknn_run(ctx_, nullptr);
    if (ret != 0) {
        std::cerr << "rknn_run(world) failed: " << ret << std::endl;
        return {};
    }

    std::vector<rknn_output> outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
        outputs[i].index = i;
        outputs[i].want_float = 1;
        outputs[i].is_prealloc = 0;
    }

    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret != 0) {
        std::cerr << "rknn_outputs_get(world) failed: " << ret << std::endl;
        return {};
    }

    std::vector<Detection> dets;
    dets.reserve(256);
    for (const auto& s : scales_) {
        const float* cls = static_cast<const float*>(outputs[s.cls_idx].buf);
        const float* box = static_cast<const float*>(outputs[s.box_idx].buf);
        if (!cls || !box) continue;

        if (debug_ && !debug_printed_) {
            float cls_min = cls[0], cls_max = cls[0];
            float box_min = box[0], box_max = box[0];
            const int HW = s.h * s.w;
            for (int i = 1; i < HW * class_count_; ++i) {
                cls_min = std::min(cls_min, cls[i]);
                cls_max = std::max(cls_max, cls[i]);
            }
            for (int i = 1; i < HW * 4; ++i) {
                box_min = std::min(box_min, box[i]);
                box_max = std::max(box_max, box[i]);
            }
            std::cout << "world debug stride=" << s.stride
                      << " cls=[" << cls_min << ", " << cls_max << "]"
                      << " box=[" << box_min << ", " << box_max << "]"
                      << std::endl;
        }

        std::vector<Detection> part = decodeScale(cls, box, s, scale, pad_x,
                                                  pad_y, bgr.cols, bgr.rows);
        dets.insert(dets.end(), part.begin(), part.end());
    }
    debug_printed_ = true;

    rknn_outputs_release(ctx_, io_num_.n_output, outputs.data());
    det::nms(dets, iou_th_);
    return dets;
}
