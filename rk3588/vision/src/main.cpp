#include "observation_export.h"
#include "camera.h"
#include "detector.h"
#include "detector_pool.h"
#include "multi_model.h"
#include "split_head_detector.h"
#include "yolo_world_detector.h"
#include "lidar.h"
#include "tracker.h"
#include "uart_link.h"
#include "vision_qa.h"
#include "voice_uart.h"
#include "types.h"
#include "semantic_engine.h"
#include "shell_quote.h"

#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool nextArg(int argc, char** argv, int& i, std::string& out) {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
}

std::vector<std::string> loadClasses(const std::string& path) {
    std::vector<std::string> names;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        names.push_back(line);
    }
    return names;
}

std::string className(int id, const std::vector<std::string>& names) {
    if (id >= 0 && id < (int)names.size()) return names[id];
    return std::to_string(id);
}

// 红/绿/蓝占比判定，连续帧确认由上层状态机负责
int classifyColor(const cv::Mat& bgr, const Detection& d) {
    int x1 = std::max(0, (int)std::lround(d.x1));
    int y1 = std::max(0, (int)std::lround(d.y1));
    int x2 = std::min(bgr.cols - 1, (int)std::lround(d.x2));
    int y2 = std::min(bgr.rows - 1, (int)std::lround(d.y2));
    if (x2 - x1 < 8 || y2 - y1 < 8) return 0;

    // 只取框中心 50%，减少背景颜色干扰
    int cw = x2 - x1;
    int ch = y2 - y1;
    cv::Rect roi(x1 + cw / 4, y1 + ch / 4,
                 std::max(1, cw / 2), std::max(1, ch / 2));
    cv::Mat hsv;
    cv::cvtColor(bgr(roi), hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1, red2, red, green, blue;
    // 红色区间放宽 + 降低饱和/亮度门槛：室内暖光下红色饱和度会掉到 80 以下
    cv::inRange(hsv, cv::Scalar(0, 70, 40), cv::Scalar(12, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(166, 70, 40), cv::Scalar(180, 255, 255), red2);
    red = red1 | red2;
    cv::inRange(hsv, cv::Scalar(35, 70, 40), cv::Scalar(85, 255, 255), green);
    cv::inRange(hsv, cv::Scalar(100, 70, 40), cv::Scalar(130, 255, 255), blue);

    double total = (double)hsv.total();
    double r = cv::countNonZero(red) / total;
    double g = cv::countNonZero(green) / total;
    double b = cv::countNonZero(blue) / total;

    double best = std::max({r, g, b});
    if (best < 0.06) return 0;
    if (best == r) return 1;
    if (best == g) return 2;
    return 3;
}

// 单目粗测距：只用框高，最终接近段应由超声/TOF 兜底
uint16_t estimateDistanceMm(const Detection& d, float fy_px, float obj_h_m) {
    const float h_px = d.y2 - d.y1;
    if (!std::isfinite(h_px) || h_px <= 0.0f ||
        !std::isfinite(fy_px) || fy_px <= 0.0f ||
        !std::isfinite(obj_h_m) || obj_h_m <= 0.0f) {
        return 0;
    }

    float dist_m = fy_px * obj_h_m / h_px;
    if (dist_m < 0.1f || dist_m > 8.0f) return 0;
    return (uint16_t)std::lround(dist_m * 1000.0f);
}

// 相对画面中心的方位角，0.1 度
int16_t estimateBearing(const Detection& d, int img_w, float hfov_deg) {
    float cx = (d.x1 + d.x2) * 0.5f;
    float norm = (cx - img_w * 0.5f) / (img_w * 0.5f);
    norm = std::max(-1.0f, std::min(1.0f, norm));
    return (int16_t)std::lround(norm * (hfov_deg * 0.5f) * 10.0f);
}

void drawDetections(cv::Mat& img, const std::vector<Detection>& dets,
                    const std::vector<std::string>& names) {
    int person_cls = -1;
    for (size_t i = 0; i < names.size(); ++i) {
        if (names[i] == "person") {
            person_cls = (int)i;
            break;
        }
    }

    auto box_iou = [](const Detection& a, const Detection& b) {
        const float xx1 = std::max(a.x1, b.x1);
        const float yy1 = std::max(a.y1, b.y1);
        const float xx2 = std::min(a.x2, b.x2);
        const float yy2 = std::min(a.y2, b.y2);
        const float inter = std::max(0.0f, xx2 - xx1) * std::max(0.0f, yy2 - yy1);
        const float uni = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1) +
                          std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1) - inter;
        return uni > 0.0f ? inter / uni : 0.0f;
    };

    for (const auto& d : dets) {
        // community / non-community 不再单独画框，直接并入 person 的标签
        if (person_cls >= 0 && d.cls != person_cls && d.parent == person_cls) continue;

        cv::Scalar color(0, 255, 0);
        if (d.color == 1) color = cv::Scalar(0, 0, 255);
        else if (d.color == 2) color = cv::Scalar(0, 255, 0);
        else if (d.color == 3) color = cv::Scalar(255, 0, 0);

        cv::rectangle(img,
                      cv::Point((int)d.x1, (int)d.y1),
                      cv::Point((int)d.x2, (int)d.y2),
                      color, 2);

        std::string label = className(d.cls, names) + " " +
                            std::to_string((int)(d.conf * 100)) + "%";
        if (person_cls >= 0 && d.cls == person_cls) {
            const Detection* attr = nullptr;
            float best_conf = -1.0f;
            for (const auto& a : dets) {
                if (a.cls == person_cls || a.parent != person_cls) continue;
                if (box_iou(d, a) >= 0.3f && a.conf > best_conf) {
                    best_conf = a.conf;
                    attr = &a;
                }
            }
            if (attr) label += " " + className(attr->cls, names);
        }
        if (d.track_id >= 0) label += " #" + std::to_string(d.track_id);

        cv::putText(img, label,
                    cv::Point((int)d.x1, std::max(15, (int)d.y1 - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
}

void drawSemanticState(cv::Mat& img, const sem::SemanticCommand& cmd) {
    std::string text = std::string("SEM: ") +
                       sem::stateName(static_cast<sem::State>(cmd.mode));
    if (cmd.flags & sem::FLAG_TARGET_VALID) {
        text += " id=" + std::to_string(cmd.target_id) +
                " bearing=" + std::to_string(cmd.bearing / 10.0) + "deg" +
                " dist=" + std::to_string(cmd.dist_mm) + "mm";
    }
    if (cmd.flags & sem::FLAG_TARGET_LOST) text += " LOST";
    if (cmd.flags & sem::FLAG_TURN_ONLY) text += " TURN";
    if (cmd.flags & sem::FLAG_VERIFY_OK) text += " OK";

    cv::putText(img, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

// 非阻塞按行读取 FIFO，保留半行缓冲；没有新数据时返回 false
class LineReader {
public:
    explicit LineReader(int fd) : fd_(fd) {}

    bool readLine(std::string& out) {
        for (;;) {
            const size_t pos = buf_.find('\n');
            if (pos != std::string::npos) {
                out = buf_.substr(0, pos);
                buf_.erase(0, pos + 1);
                if (!out.empty() && out.back() == '\r') out.pop_back();
                return true;
            }
            char chunk[256];
            const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
            if (n <= 0) return false;
            buf_.append(chunk, static_cast<size_t>(n));
            if (buf_.size() > 4096) buf_.clear();   // 脏数据不清行，直接丢弃
        }
    }

private:
    int fd_;
    std::string buf_;
};

}  // namespace

int main(int argc, char** argv) {
    std::string model = "models/yolov8n.rknn";
    std::string model_type = "yolov8";
    std::string world_model = "models/yolo_world_v2s_i8.rknn";
    std::string world_texts = "models/texts.bin";
    std::string world_box_mode = "ltrb_stride";
    bool world_debug = false;
    bool world_sigmoid = false;
    bool conf_set = false;
    std::string device = "/dev/video0";
    std::string uart_dev = "";
    std::string voice_uart_dev = "";
    std::string voice_fifo_path;   // ROS 语音话题桥接写入的 FIFO
    std::string cmdvel_fifo_path;  // 语义速度输出到 cmd_vel 桥接的 FIFO
    std::string observations_path;
    std::string preview_path;     // Optional atomic JPEG for local web remote.
    int voice_baud = 115200;
    std::string voice_encoding = "gbk";
    std::string tts_topic;   // 可选：问答结果同时发布到该 ROS2 话题（语音节点播报）
    std::string lidar_dev = "";
    std::string classes_path = "classes.txt";
    std::string model_config;
    int width = 1280;
    int height = 720;
    int fps = 80;
    int threads = 3;   // 并行推理上下文数量（RK3588 三核 NPU）
    int baud = 921600;
    std::string crc_mode = "x25";
    int lidar_baud = 460800;
    float lidar_offset = 0.0f;
    bool lidar_invert = false;
    float conf = 0.25f;
    float iou = 0.45f;
    float hfov = 50.0f;
    float fy_px = 1000.0f;
    float obj_h_m = 0.25f;
    bool obj_h_override = false;
    bool display = false;
    bool semantic = false;
    bool uart_text = false;
    float search_wz = 0.30f;     // 搜索自转角速度 rad/s（缓慢，两圈约 42s）
    float stop_ratio = 0.55f;    // 目标框高度占画面比例达到该值即停车
    float approach_vmax = 0.20f; // 逼近阶段最大线速度 m/s（缓慢靠近）
    float stop_dist = 0.6f;      // 单目测距报值超过该距离即停车（报值约为实际 2 倍）
    float center_tol = 10.0f;    // 目标偏离画面中心小于该角度才允许前进
    bool lidar_test = false;
    bool preview_only = false;
    std::string cmd_text;
    std::string image_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        std::string v;
        if (a == "--model" && nextArg(argc, argv, i, v)) model = v;
        else if (a == "--model-type" && nextArg(argc, argv, i, v)) model_type = v;
        else if (a == "--world-model" && nextArg(argc, argv, i, v)) world_model = v;
        else if (a == "--world-texts" && nextArg(argc, argv, i, v)) world_texts = v;
        else if (a == "--world-box-mode" && nextArg(argc, argv, i, v)) world_box_mode = v;
        else if (a == "--world-debug" && nextArg(argc, argv, i, v)) world_debug = (std::stoi(v) != 0);
        else if (a == "--world-sigmoid" && nextArg(argc, argv, i, v)) world_sigmoid = (std::stoi(v) != 0);
        else if (a == "--device" && nextArg(argc, argv, i, v)) device = v;
        else if (a == "--uart" && nextArg(argc, argv, i, v)) uart_dev = v;
        else if (a == "--voice-uart" && nextArg(argc, argv, i, v)) voice_uart_dev = v;
        else if (a == "--voice-fifo" && nextArg(argc, argv, i, v)) voice_fifo_path = v;
        else if (a == "--cmdvel-fifo" && nextArg(argc, argv, i, v)) cmdvel_fifo_path = v;
        else if (a == "--fpga-observations" && nextArg(argc, argv, i, v)) observations_path = v;
        else if (a == "--preview-file" && nextArg(argc, argv, i, v)) preview_path = v;
        else if (a == "--voice-baud" && nextArg(argc, argv, i, v)) voice_baud = std::stoi(v);
        else if (a == "--voice-encoding" && nextArg(argc, argv, i, v)) voice_encoding = v;
        else if (a == "--tts-topic" && nextArg(argc, argv, i, v)) tts_topic = v;
        else if (a == "--lidar" && nextArg(argc, argv, i, v)) lidar_dev = v;
        else if (a == "--lidar-baud" && nextArg(argc, argv, i, v)) lidar_baud = std::stoi(v);
        else if (a == "--lidar-offset" && nextArg(argc, argv, i, v)) lidar_offset = std::stof(v);
        else if (a == "--lidar-invert" && nextArg(argc, argv, i, v)) lidar_invert = (std::stoi(v) != 0);
        else if (a == "--lidar-test" && nextArg(argc, argv, i, v)) lidar_test = (std::stoi(v) != 0);
        else if (a == "--classes" && nextArg(argc, argv, i, v)) classes_path = v;
        else if (a == "--model-config" && nextArg(argc, argv, i, v)) model_config = v;
        else if (a == "--w" && nextArg(argc, argv, i, v)) width = std::stoi(v);
        else if (a == "--h" && nextArg(argc, argv, i, v)) height = std::stoi(v);
        else if (a == "--fps" && nextArg(argc, argv, i, v)) fps = std::stoi(v);
        else if (a == "--threads" && nextArg(argc, argv, i, v)) threads = std::stoi(v);
        else if (a == "--baud" && nextArg(argc, argv, i, v)) baud = std::stoi(v);
        else if (a == "--crc-mode" && nextArg(argc, argv, i, v)) crc_mode = v;
        else if (a == "--conf" && nextArg(argc, argv, i, v)) {
            conf = std::stof(v);
            conf_set = true;
        }
        else if (a == "--iou" && nextArg(argc, argv, i, v)) iou = std::stof(v);
        else if (a == "--hfov" && nextArg(argc, argv, i, v)) hfov = std::stof(v);
        else if (a == "--fy" && nextArg(argc, argv, i, v)) fy_px = std::stof(v);
        else if (a == "--objh" && nextArg(argc, argv, i, v)) {
            obj_h_m = std::stof(v);
            obj_h_override = true;
        }
        else if (a == "--semantic" && nextArg(argc, argv, i, v)) semantic = (std::stoi(v) != 0);
        else if (a == "--uart-text" && nextArg(argc, argv, i, v)) uart_text = (std::stoi(v) != 0);
        else if (a == "--search-wz" && nextArg(argc, argv, i, v)) search_wz = std::stof(v);
        else if (a == "--stop-ratio" && nextArg(argc, argv, i, v)) stop_ratio = std::stof(v);
        else if (a == "--approach-vmax" && nextArg(argc, argv, i, v)) approach_vmax = std::stof(v);
        else if (a == "--stop-dist" && nextArg(argc, argv, i, v)) stop_dist = std::stof(v);
        else if (a == "--center-tol" && nextArg(argc, argv, i, v)) center_tol = std::stof(v);
        else if (a == "--preview-only" && nextArg(argc, argv, i, v)) preview_only = (std::stoi(v) != 0);
        else if (a == "--cmd" && nextArg(argc, argv, i, v)) cmd_text = v;
        else if (a == "--image" && nextArg(argc, argv, i, v)) image_path = v;
        else if (a == "--display" && nextArg(argc, argv, i, v)) display = (std::stoi(v) != 0);
        else if (a == "--help") {
            std::cout
                << "用法: ./vision [选项]\n"
                << "  --model   模型路径，默认 models/yolov8n.rknn\n"
                << "  --model-type 检测模型路线：yolov8（默认）/ world / split\n"
                << "  --model-config 多模型并行配置(models.txt)，指定后忽略 --model-type，\n"
                << "                 所有模型同时推理，类别合并到一张统一表\n"
                << "  --world-model YOLO-World 模型路径，默认 models/yolo_world_v2s_i8.rknn\n"
                << "  --world-texts 文本向量文件，默认 models/texts.bin（world 模式必需）\n"
                << "  --world-box-mode world 框解码：xywh_px/xywh_norm/xyxy_px/xyxy_norm/ltrb_px/ltrb_stride，默认 ltrb_stride\n"
                << "  --world-debug 1 打印 world 输出数值范围，便于确认解码模式\n"
                << "  --world-sigmoid 1 若 world 分类输出是 logits，则先做 sigmoid\n"
                << "  --device  摄像头设备，默认 /dev/video0\n"
                << "  --uart    串口设备，例如 /dev/ttyS3；不填则只跑视觉\n"
                << "  --baud    串口波特率，默认 921600\n"
                << "  --crc-mode CRC 算法：x25（默认）或 mcrf4xx\n"
                << "  --voice-uart 离线语音模块串口，例如 /dev/ttyS9；不填则仅用终端输入\n"
                << "  --voice-fifo ROS 语音桥接 FIFO，例如 /tmp/voice_cmd.fifo；由 asr_text_bridge 写入\n"
                << "  --cmdvel-fifo 语义速度输出 FIFO，例如 /tmp/cmd_vel.fifo；由 cmd_vel_bridge 读并转成 /cmd_vel\n"
                << "  --preview-file 可选网页预览 JPEG 路径，例如 /tmp/rk_preview.jpg\n"
                << "  --voice-baud 语音模块波特率，默认 115200\n"
                << "  --voice-encoding 语音模块输出编码：gbk（默认）或 utf8\n"
                << "  --tts-topic 视觉问答结果同时发布到该 ROS2 String 话题（如 /smart_community/tts_text）\n"
                << "  --lidar   激光雷达串口，例如 /dev/ttyUSB0；不填则用单目估距\n"
                << "  --lidar-baud 雷达波特率，默认 460800（RPLIDAR C1）\n"
                << "  --lidar-offset 雷达 0 度与相机光轴的夹角(度)，默认 0\n"
                << "  --lidar-invert 1 表示雷达角度反向，默认 0\n"
                << "  --lidar-test 1 只测雷达，不打开摄像头和模型\n"
                << "  --w/--h   采集分辨率，默认 1280x720\n"
                << "  --fps     请求帧率，默认 80\n"
                << "  --threads 并行推理上下文数量，默认 3（RK3588 三核 NPU）\n"
                << "  --conf    置信度阈值，默认 0.25\n"
                << "  --iou     NMS IoU 阈值，默认 0.45\n"
                << "  --hfov    相机水平视场角，默认 50 度\n"
                << "  --fy      标定后的竖直焦距像素，默认 1000\n"
                << "  --objh    目标实际高度(米)，默认 0.25；语义模式下显式设置时覆盖类别表\n"
                << "  --semantic 1 启用语义层，默认 0\n"
                << "  --search-wz 搜索阶段原地自转角速度(rad/s)，默认 0.30，转满两圈未找到即结束任务\n"
                << "  --stop-dist 逼近停车距离(米)，默认 0.6，到达后立即结束任务\n"
                << "  --approach-vmax 逼近阶段最大线速度(m/s)，默认 0.20\n"
                << "  --center-tol 目标居中容差(度)，默认 10，超出先原地转向再前进\n"
                << "  --preview-only 1 只预览相机画面，不加载模型\n"
                << "  --cmd     启动时执行的指令，例如 \"找红色瓶子\"\n"
                << "  --image   只跑一张图片并打印检测结果，用于验证模型\n"
                << "  --display 1 显示窗口，0 无界面\n"
                << "\n交互指令示例（回车发送）：找红色瓶子 / 靠近最近的人 / 停止 / 报告\n"
                << "视觉问答：语音或终端输入「这是什么/看到了什么」类问句时，\n"
                << "          调 DeepSeek 组织回答并由语音模块播报（需设置 DEEPSEEK_API_KEY，\n"
                << "          无 key 或无网络时退回本地模板播报）\n";
            return 0;
        }
    }

    if (width <= 0 || height <= 0 || fps <= 0 || threads < 1 || threads > 8 ||
        !std::isfinite(conf) || conf < 0.0f || conf > 1.0f ||
        !std::isfinite(iou) || iou < 0.0f || iou > 1.0f ||
        !std::isfinite(hfov) || hfov <= 0.0f || hfov >= 179.0f ||
        !std::isfinite(fy_px) || fy_px <= 0.0f ||
        !std::isfinite(obj_h_m) || obj_h_m <= 0.0f) {
        std::cerr << "参数无效：分辨率/FPS 必须为正数，threads 须在 1~8，"
                  << "阈值须在 0~1，HFOV 须在 0~179 度，fy 和目标高度须为正数"
                  << std::endl;
        return 2;
    }
    if (crc_mode != "x25" && crc_mode != "mcrf4xx") {
        std::cerr << "CRC 模式无效：仅支持 x25 或 mcrf4xx" << std::endl;
        return 2;
    }
    if (model_type != "yolov8" && model_type != "world" && model_type != "split") {
        std::cerr << "模型类型无效：仅支持 yolov8 / world / split" << std::endl;
        return 2;
    }
    // 三条路线的分数分布不同，未显式指定时各用各的默认阈值
    if (!conf_set) {
        if (model_type == "world") {
            conf = 0.10f;
        } else if (model_type == "split") {
            conf = 0.30f;
        } else {
            conf = 0.25f;
        }
    }

    YoloWorldDetector::BoxMode box_mode = YoloWorldDetector::BoxMode::XYWH_PX;
    if (model_type == "world") {
        if (world_box_mode == "xywh_px") {
            box_mode = YoloWorldDetector::BoxMode::XYWH_PX;
        } else if (world_box_mode == "xywh_norm") {
            box_mode = YoloWorldDetector::BoxMode::XYWH_NORM;
        } else if (world_box_mode == "xyxy_px") {
            box_mode = YoloWorldDetector::BoxMode::XYXY_PX;
        } else if (world_box_mode == "xyxy_norm") {
            box_mode = YoloWorldDetector::BoxMode::XYXY_NORM;
        } else if (world_box_mode == "ltrb_px") {
            box_mode = YoloWorldDetector::BoxMode::LTRB_PX;
        } else if (world_box_mode == "ltrb_stride") {
            box_mode = YoloWorldDetector::BoxMode::LTRB_STRIDE;
        } else {
            std::cerr << "框解码模式无效：仅支持 xywh_px / xywh_norm / "
                      << "xyxy_px / xyxy_norm / ltrb_px / ltrb_stride" << std::endl;
            return 2;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    // 写 FIFO 时桥接端若已退出，write 会返回 EPIPE；忽略 SIGPIPE 避免整个进程被打断
    std::signal(SIGPIPE, SIG_IGN);

    // 雷达自检：只验证串口、波特率、角度和测距，不依赖相机/RKNN
    if (lidar_test) {
        if (lidar_dev.empty()) {
            std::cerr << "--lidar-test 需要同时指定 --lidar <设备>" << std::endl;
            return 2;
        }

        Lidar lidar;
        if (!lidar.open(lidar_dev, lidar_baud)) return 1;
        lidar.start();

        std::cout << "雷达自检中，Ctrl+C 退出；每 500ms 打印各角度最近回波"
                  << std::endl;

        int64_t last_print = 0;
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const int64_t now = nowMs();
            if (now - last_print < 500) continue;
            last_print = now;

            std::cout << "scans=" << lidar.scanCount();
            for (int deg = 0; deg < 360; deg += 45) {
                float d = 0.0f;
                std::cout << " " << deg << "deg=";
                if (lidar.distanceInWindow((float)deg, 5.0f, d)) {
                    std::cout << d << "m";
                } else {
                    std::cout << "--";
                }
            }
            std::cout << std::endl;
        }

        lidar.stop();
        return 0;
    }

    std::vector<std::string> classes = loadClasses(classes_path);
    if (classes.empty()) {
        std::cerr << "warning: classes.txt 为空或不存在，显示时使用类别 ID" << std::endl;
    }

    // 单张图片自检模式：不打开相机、不建线程池，用来验证模型与解码是否正确
    if (!image_path.empty()) {
        cv::Mat img = cv::imread(image_path);
        if (img.empty()) {
            std::cerr << "读取图片失败: " << image_path << std::endl;
            return 1;
        }
        // 多模型配置下让每个模型各跑一次同一张图，便于逐模型核对类别表
        if (!model_config.empty()) {
            std::vector<ModelSpec> specs;
            if (!loadModelSpecs(model_config, specs)) return 1;
            for (const auto& s : specs) {
                std::unique_ptr<IDetector> det =
                    (s.route == "split") ? std::unique_ptr<IDetector>(
                                               std::make_unique<SplitHeadDetector>())
                                         : std::unique_ptr<IDetector>(
                                               std::make_unique<Detector>());
                const float use_conf = conf_set ? conf : s.conf;
                if (!det->init(s.model_path, use_conf, iou,
                               (int)s.classes.size(), std::string())) {
                    std::cerr << "[" << s.name << "] 初始化失败，跳过" << std::endl;
                    continue;
                }
                const auto dets = det->infer(img);
                std::cout << "[" << s.name << "] " << img.cols << "x" << img.rows
                          << " 检测到 " << dets.size() << " 个目标" << std::endl;
                for (const auto& d : dets) {
                    const std::string name =
                        (d.cls >= 0 && d.cls < (int)s.classes.size())
                            ? s.classes[(size_t)d.cls]
                            : std::to_string(d.cls);
                    std::cout << "  " << name << " conf=" << d.conf
                              << " box=(" << d.x1 << "," << d.y1 << ")-("
                              << d.x2 << "," << d.y2 << ")" << std::endl;
                }
            }
            return 0;
        }

        // 单图模式走和正式运行完全一样的检测器与解码，避免验证结果和实际链路不一致
        std::string img_model = (model_type == "world") ? world_model : model;
        std::string img_aux = (model_type == "world") ? world_texts : std::string();
        std::unique_ptr<IDetector> det;
        if (model_type == "world") {
            auto world = std::make_unique<YoloWorldDetector>();
            world->setBoxMode(box_mode);
            world->setDebug(world_debug);
            world->setApplySigmoid(world_sigmoid);
            det = std::move(world);
        } else if (model_type == "split") {
            det = std::make_unique<SplitHeadDetector>();
        } else {
            det = std::make_unique<Detector>();
        }
        if (!det->init(img_model, conf, iou, (int)classes.size(), img_aux)) {
            std::cerr << "初始化 RKNN 失败: " << img_model << std::endl;
            return 1;
        }
        const auto dets = det->infer(img);
        std::cout << "图片 " << image_path << " " << img.cols << "x" << img.rows
                  << " 检测到 " << dets.size() << " 个目标" << std::endl;
        for (const auto& d : dets) {
            const std::string name = (d.cls >= 0 && d.cls < (int)classes.size())
                                         ? classes[(size_t)d.cls]
                                         : std::to_string(d.cls);
            std::cout << "  " << name << " conf=" << d.conf
                      << " box=(" << d.x1 << "," << d.y1 << ")-("
                      << d.x2 << "," << d.y2 << ")" << std::endl;
        }
        return 0;
    }

    Camera camera(device, width, height, fps);
    if (!camera.open()) {
        std::cerr << "打开摄像头失败: " << device << std::endl;
        return 1;
    }
    camera.start();

    // 相机预览：不加载模型，先确认设备、分辨率、帧率是否正常
    if (preview_only) {
        std::cout << "预览模式: " << device << " " << width << "x" << height
                  << " @" << fps << "fps, display=" << (display ? "on" : "off")
                  << std::endl;
        if (!display) {
            std::cout << "未开启显示窗口，加 --display 1 才能看到画面" << std::endl;
        }

        int64_t last_report = nowMs();
        int frames_since = 0;
        while (!g_stop) {
            Frame frame;
            if (!camera.getLatest(frame, 200)) continue;

            ++frames_since;
            const int64_t now = nowMs();
            if (now - last_report >= 1000) {
                std::cout << "实际画面=" << frame.image.cols << "x" << frame.image.rows
                          << " 实测帧率="
                          << frames_since * 1000.0 / (double)(now - last_report)
                          << "fps" << std::endl;
                last_report = now;
                frames_since = 0;
            }

            if (display) {
                cv::imshow("rk3588_vision", frame.image);
                int key = cv::waitKey(1);
                if (key == 27 || key == 'q') break;
            }
        }

        camera.stop();
        std::cout << "退出" << std::endl;
        return 0;
    }

    std::string active_model = model;
    std::string aux_path;
    std::vector<std::string> active_classes = classes;
    DetectorPool::Factory detector_factory;
    DetectorPool detector_pool;
    std::unique_ptr<MultiModelRunner> multi_runner;

    if (!model_config.empty()) {
        // 多模型并行：每个模型一个独立推理池，结果合并到统一类别表
        std::vector<ModelSpec> specs;
        if (!loadModelSpecs(model_config, specs)) {
            camera.stop();
            return 2;
        }
        if (conf_set) {
            // 用户显式 --conf 时统一下发到所有模型
            for (auto& s : specs) s.conf = conf;
        }
        multi_runner = std::make_unique<MultiModelRunner>();
        if (!multi_runner->init(specs, iou)) {
            std::cerr << "多模型初始化失败" << std::endl;
            camera.stop();
            return 1;
        }
        active_model = model_config;
        active_classes = multi_runner->classNames();
    } else if (model_type == "world") {
        active_model = world_model;
        aux_path = world_texts;
        detector_factory = [box_mode, world_debug, world_sigmoid]()
            -> std::unique_ptr<IDetector> {
            auto world = std::make_unique<YoloWorldDetector>();
            world->setBoxMode(box_mode);
            world->setDebug(world_debug);
            world->setApplySigmoid(world_sigmoid);
            return world;
        };
        std::cout << "world 配置: box_mode=" << world_box_mode
                  << ", sigmoid=" << (world_sigmoid ? 1 : 0)
                  << ", conf=" << conf << std::endl;
    } else if (model_type == "split") {
        detector_factory = []() -> std::unique_ptr<IDetector> {
            return std::make_unique<SplitHeadDetector>();
        };
    } else {
        detector_factory = []() -> std::unique_ptr<IDetector> {
            return std::make_unique<Detector>();
        };
    }

    if (!multi_runner) {
        if (!detector_pool.start(threads, detector_factory, active_model, conf, iou,
                                 static_cast<int>(classes.size()), aux_path)) {
            std::cerr << "初始化 RKNN 失败: " << active_model << std::endl;
            camera.stop();
            return 1;
        }
    }
    const int pool_workers =
        multi_runner ? multi_runner->modelCount() : detector_pool.workers();

    Tracker tracker(0.3f, 30);
    UartLink uart;
    uart.setCrcMode(crc_mode == "x25" ? UartLink::CrcMode::X25
                                       : UartLink::CrcMode::MCRF4XX);
    if (!uart_dev.empty() || !cmdvel_fifo_path.empty() || !lidar_dev.empty()) {
        std::cerr << "FPGA architecture: do not use --uart, --cmdvel-fifo or --lidar in RK vision.\n";
        return 2;
    }
    if (!uart_dev.empty() && !uart.open(uart_dev, baud)) {
        std::cerr << "串口打开失败，继续以无串口模式运行" << std::endl;
    }

    VoiceUart voice_uart;
    if (!voice_uart_dev.empty()) {
        VoiceUart::Encoding enc =
            (voice_encoding == "utf8" || voice_encoding == "utf-8")
                ? VoiceUart::Encoding::UTF8
                : VoiceUart::Encoding::GBK;
        if (!voice_uart.open(voice_uart_dev, voice_baud, enc)) {
            std::cerr << "语音模块串口打开失败，仅保留终端输入" << std::endl;
        }
    }

    // ROS 语音话题桥接：asr_text_bridge 订阅识别文本并写入 FIFO，这里非阻塞读
    int voice_fifo_fd = -1;
    std::unique_ptr<LineReader> fifo_reader;
    if (!voice_fifo_path.empty()) {
        if (::access(voice_fifo_path.c_str(), F_OK) != 0 &&
            ::mkfifo(voice_fifo_path.c_str(), 0666) != 0) {
            std::cerr << "创建语音 FIFO 失败: " << voice_fifo_path << std::endl;
        }
        voice_fifo_fd = ::open(voice_fifo_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (voice_fifo_fd < 0) {
            std::cerr << "语音 FIFO 打开失败: " << voice_fifo_path
                      << "，仅保留语音串口/终端输入" << std::endl;
        } else {
            // 丢弃启动前残留的旧指令，避免一上线就执行过期命令
            char drain[256];
            while (::read(voice_fifo_fd, drain, sizeof(drain)) > 0) {}
            fifo_reader = std::make_unique<LineReader>(voice_fifo_fd);
        }
    }

    // ROS 语义速度桥接：vision 把 $vx,vy,wz 写进 FIFO，cmd_vel_bridge 转成 geometry_msgs/Twist
    int cmdvel_fifo_fd = -1;
    if (!cmdvel_fifo_path.empty()) {
        if (::access(cmdvel_fifo_path.c_str(), F_OK) != 0 &&
            ::mkfifo(cmdvel_fifo_path.c_str(), 0666) != 0) {
            std::cerr << "创建速度 FIFO 失败: " << cmdvel_fifo_path << std::endl;
        }
        // 桥接端以 O_RDWR 打开，保证这里 O_WRONLY 非阻塞打开不会因暂时无读者而失败
        cmdvel_fifo_fd = ::open(cmdvel_fifo_path.c_str(), O_WRONLY | O_NONBLOCK);
        if (cmdvel_fifo_fd < 0) {
            std::cerr << "速度 FIFO 暂未就绪（等待 cmd_vel_bridge），将自动重试: "
                      << cmdvel_fifo_path << std::endl;
        }
    }

    Lidar lidar;
    if (!lidar_dev.empty()) {
        if (lidar.open(lidar_dev, lidar_baud)) {
            lidar.start();
        } else {
            std::cerr << "激光雷达打开失败，距离回退为单目估计" << std::endl;
        }
    }

    sem::SemanticEngine semantic_engine;
    if (semantic) {
        sem::CameraParams cam_params;
        cam_params.img_w = width;
        cam_params.hfov_deg = hfov;
        cam_params.fy_px = fy_px;
        cam_params.default_obj_h_m = obj_h_m;
        cam_params.object_height_override_m = obj_h_override ? obj_h_m : 0.0f;
        // Text chassis protocol must never be interleaved with 0xAA56 binary frames.
        semantic_engine.init(active_classes, uart_text ? nullptr : &uart, cam_params);
        // 状态机与底盘共用同一组搜索/停车参数，避免「车已停、状态机还在逼近」
        semantic_engine.setSearchAngularSpeed(search_wz);
        semantic_engine.setStopDistance(stop_dist);

        if (!cmd_text.empty() && !semantic_engine.submitText(cmd_text)) {
            std::cerr << "无法解析启动指令: " << cmd_text << std::endl;
        }
    }

    std::string uart_state = uart.isOpen() ? uart_dev : std::string("none");
    std::string voice_state = voice_uart.isOpen() ? voice_uart_dev : std::string("none");
    std::string lidar_state = lidar.isOpen() ? lidar_dev : std::string("none");
    std::cout << "运行中: " << device << " " << width << "x" << height
              << " @" << fps << "fps"
              << ", model=" << active_model
              << (multi_runner ? ", models=" + std::to_string(multi_runner->modelCount())
                               : ", model_type=" + model_type)
              << ", uart=" << uart_state
              << ", voice=" << voice_state
              << ", vfifo=" << (voice_fifo_fd >= 0 ? voice_fifo_path : std::string("none"))
              << ", cvfifo=" << (cmdvel_fifo_fd >= 0 ? cmdvel_fifo_path : std::string("none"))
              << ", lidar=" << lidar_state
              << ", semantic=" << (semantic ? "on" : "off")
              << ", workers=" << pool_workers
              << std::endl;

    uint64_t seq = 0;
    auto last_send = std::chrono::steady_clock::now();
    int frame_count = 0;       // 主循环帧数（采集/显示节奏）
    int det_count = 0;         // 每秒完成的推理结果数
    int no_frame_count = 0;
    int64_t last_det_ts = 0;
    std::vector<Detection> latest_dets;   // 最近一次推理结果，供串口与显示复用
    int64_t report_start = nowMs();

    // 视觉问答：「这是什么」类问句 → 快照当前画面 → 后台调 DeepSeek → 语音播报。
    // 网络请求在后台线程执行，主循环不阻塞；DeepSeek 不可用时退回本地模板
    std::atomic<bool> qa_busy{false};
    std::thread qa_worker;
    std::mutex speak_mutex;
    auto publishText = [&](const std::string& text) {
        std::lock_guard<std::mutex> lock(speak_mutex);
        if (voice_uart.isOpen() && !voice_uart.sendText(text)) {
            std::cerr << "[voice] 语音串口发送失败" << std::endl;
        }
        if (!tts_topic.empty()) {
            // ROS2 语音节点走话题播报；YAML 双引号字符串需转义 \ 和 "
            std::string esc;
            for (char c : text) {
                if (c == '\n') { esc += "\\n"; continue; }
                if (c == '\r') { esc += "\\r"; continue; }
                if (c == '\\' || c == '"') esc += '\\';
                esc += c;
            }
            const std::string cmd =
                "timeout 5s ros2 topic pub --once " + shellQuote(tts_topic) +
                " std_msgs/msg/String " + shellQuote("{data: \"" + esc + "\"}") +
                " >/dev/null 2>&1";
            int rc = std::system(cmd.c_str());
            if (rc != 0) {
                std::cerr << "[qa] 发布 " << tts_topic << " 失败 rc=" << rc
                          << std::endl;
            }
        }
    };
    auto answerSceneQuestion = [&](const std::string& question) {
        if (qa_busy.exchange(true)) {
            std::cout << "[qa] 上一次问答未完成，本次跳过" << std::endl;
            return;
        }
        if (qa_worker.joinable()) qa_worker.join();
        const bool fresh = last_det_ts > 0 && nowMs() - last_det_ts < 2000;
        const std::vector<Detection> snapshot = fresh ? latest_dets : std::vector<Detection>{};
        const std::string scene = fresh ?
            buildSceneDescription(snapshot, active_classes, width) : "当前没有新鲜视觉数据，不能判断场景";
        const std::string fallback =
            fresh ? localAnswerForQuestion(question, snapshot, active_classes, width) :
                    "当前视觉数据已过期，请检查摄像头。";
        const char* env = std::getenv("DEEPSEEK_API_KEY");
        const std::string key = env ? env : "";
        qa_worker = std::thread([question, scene, fallback, key,
                     &publishText, &qa_busy]() {
            std::string ans;
            if (!key.empty()) ans = askDeepSeek(key, question, scene);
            if (ans.empty()) ans = fallback;
            std::cout << "[qa] 播报: " << ans << std::endl;
            publishText(ans);
            qa_busy = false;
        });
    };
    // 三路文本输入（语音串口 / FIFO 桥接 / 终端）统一走这里
    auto handleVoiceLine = [&](const std::string& line) {
        if (line.empty()) return;
        // Explicit read-only route from the new dialogue gate; never parse as motion.
        if (line.rfind("@question:", 0) == 0) {
            answerSceneQuestion(line.substr(10));
            return;
        }
        if (isSceneQuestion(line)) {
            answerSceneQuestion(line);
            return;
        }
        if (!semantic_engine.submitText(line)) {
            std::cerr << "[voice] 无法解析指令: " << line << std::endl;
            if (sem::hasTargetActionVerb(line)) {
                publishText("要找什么？");
            }
        }
    };

    double post_ms_sum = 0.0;     // color + lidar + tracker + semantic
    double disp_ms_sum = 0.0;     // clone + draw + imshow
    double total_ms_sum = 0.0;    // 单帧总耗时

    while (!g_stop) {
        auto loop_start = std::chrono::steady_clock::now();

        Frame frame;
        if (!camera.getLatest(frame, 200)) {
            if (++no_frame_count >= 50) {
                std::cerr << "连续 10 秒没有相机帧，退出" << std::endl;
                break;
            }
            continue;
        }
        no_frame_count = 0;

        // ---- 投递最新帧，推理在后台线程池进行，主循环不被拖慢 ----
        auto frame_img = std::make_shared<cv::Mat>(std::move(frame.image));
        if (!preview_path.empty()) {
            static int64_t last_preview = 0;
            if (nowMs() - last_preview >= 200) {
                last_preview = nowMs();
                try {
                    cv::Mat small;
                    const double scale = std::min(1.0, 640.0 / frame_img->cols);
                    cv::resize(*frame_img, small, cv::Size(), scale, scale);
                    const std::string temp = preview_path + ".tmp.jpg";
                    if (cv::imwrite(temp, small, {cv::IMWRITE_JPEG_QUALITY, 65}))
                        std::rename(temp.c_str(), preview_path.c_str());
                } catch (const cv::Exception& e) {
                    std::cerr << "preview: " << e.what() << std::endl;
                }
            }
        }
        std::vector<Detection> dets;
        std::shared_ptr<const cv::Mat> det_image;
        bool has_new = false;
        if (multi_runner) {
            multi_runner->submit(frame_img, frame.seq, frame.timestamp_ms);
            MultiModelRunner::Result r;
            if (multi_runner->takeLatest(r)) {
                has_new = true;
                dets = std::move(r.dets);
                det_image = r.image ? r.image : frame_img;
                last_det_ts = r.timestamp_ms;
            }
        } else {
            detector_pool.submit(frame_img, frame.seq, frame.timestamp_ms);
            DetectorPool::Result result;
            if (detector_pool.takeLatest(result)) {
                has_new = true;
                dets = std::move(result.dets);
                det_image = result.image ? result.image : frame_img;
                last_det_ts = result.timestamp_ms;
            }
        }

        // ---- 后处理：只在新结果到达时执行，坐标对应检测那一帧 ----
        auto t_post0 = std::chrono::steady_clock::now();
        if (has_new) {
            ++det_count;

            const cv::Mat& det_img = *det_image;
            for (auto& d : dets) {
                d.color = classifyColor(det_img, d);

                // 距离优先取雷达：用检测框方位角查同一方向上的最近回波
                if (lidar.isOpen()) {
                    const float bearing_deg =
                        estimateBearing(d, det_img.cols, hfov) / 10.0f;
                    const float angle = lidar_invert
                                            ? lidar_offset - bearing_deg
                                            : lidar_offset + bearing_deg;
                    float half_span = 0.5f * ((d.x2 - d.x1) / det_img.cols) * hfov;
                    half_span = std::clamp(half_span, 0.5f, 10.0f);

                    float dist_m = 0.0f;
                    if (lidar.distanceInWindow(angle, half_span, dist_m)) {
                        d.dist_m = dist_m;
                    }
                }
                // 无雷达回波时用单目估距兜底，保证语音播报能带上距离
                if (d.dist_m <= 0.0f) {
                    const float h_px = d.y2 - d.y1;
                    if (h_px > 1.0f && fy_px > 0.0f) {
                        const float est = fy_px * obj_h_m / h_px;
                        if (est >= 0.15f && est <= 8.0f) d.dist_m = est;
                    }
                }
            }
            tracker.update(dets);
            // Use actual capture timestamp, never refresh stale inference results.
            if (!observations_path.empty())
                exportObservations(observations_path, dets, last_det_ts, det_img.cols, hfov);


            if (semantic) {
                semantic_engine.setImageWidth(det_img.cols);
                semantic_engine.update(dets, nowMs());
            }
            latest_dets = std::move(dets);
        }
        auto t_post1 = std::chrono::steady_clock::now();

        if (semantic) {
            // 1) 离线语音模块串口输入
            if (voice_uart.isOpen()) {
                std::string vline;
                while (voice_uart.readLine(vline)) {
                    std::cout << "[voice] \"" << vline << "\"" << std::endl;
                    handleVoiceLine(vline);
                }
            }
            // 2) ROS 语音话题桥接 FIFO（asr_text_bridge 写入）
            if (fifo_reader) {
                std::string fline;
                while (fifo_reader->readLine(fline)) {
                    std::cout << "[voice] \"" << fline << "\"" << std::endl;
                    handleVoiceLine(fline);
                }
            }
            // 3) 终端手动输入（回车生效）；EOF 后不永久关闭，管道重连仍可继续读
            struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
            if (::poll(&pfd, 1, 0) > 0) {
                std::string line;
                if (std::getline(std::cin, line)) handleVoiceLine(line);
            }
        }

        // ---- 串口发送 ----
        auto now = std::chrono::steady_clock::now();
        const int vision_period_ms = semantic ? 100 : 33;
        if (now - last_send >= std::chrono::milliseconds(vision_period_ms)) {
            VisionFrame vf{};
            vf.magic = 0xAA55;
            vf.ver = 1;
            vf.seq = (uint32_t)(seq++);
            vf.tick_ms = (uint32_t)(nowMs() & 0xFFFFFFFF);
            vf.count = 0;

            for (const auto& d : latest_dets) {
                if (vf.count >= 4) break;
                TargetMsg& t = vf.targets[vf.count++];
                t.cls = (uint8_t)std::max(0, std::min(255, d.cls));
                t.color = (uint8_t)std::max(0, std::min(3, d.color));
                t.conf = (uint8_t)std::max(0, std::min(100,
                            (int)std::lround(d.conf * 100.0f)));
                t.cx = (int16_t)std::lround((d.x1 + d.x2) * 0.5f);
                t.cy = (int16_t)std::lround((d.y1 + d.y2) * 0.5f);
                t.w = (int16_t)std::lround(d.x2 - d.x1);
                t.h = (int16_t)std::lround(d.y2 - d.y1);
                t.bearing = estimateBearing(d, frame_img->cols, hfov);
                if (d.dist_m > 0.0f) {
                    t.dist_mm = (uint16_t)std::lround(
                        std::min(65.535f, d.dist_m) * 1000.0f);
                } else {
                    t.dist_mm = estimateDistanceMm(d, fy_px, obj_h_m);
                }
            }

            // 只要配置了速度 FIFO 就进入本块：里面会按帧重试打开，
            // 避免 vision 先于 cmd_vel_bridge 启动时 fd 一直打不开、速度永远发不出去
            if (semantic && (uart_text || !cmdvel_fifo_path.empty())) {
                // 文本速度输出：写串口（FPGA 文本协议）和/或 FIFO（转 /cmd_vel）
                const sem::SemanticCommand sc = semantic_engine.lastCommand();
                const bool target_ok =
                    (sc.flags & (uint8_t)sem::CommandFlag::FLAG_TARGET_VALID) != 0;
                const float dist_m = sc.dist_mm / 1000.0f;
                const float bearing_deg = sc.bearing / 10.0f;

                // 近距到位判据用「目标框高度占画面比例」，与焦距无关：
                // 框越接近占满画面，单目测距越饱和，此时只能靠框占比判定距离，继续前压就会顶到目标
                // 到位判据只看“当前跟踪的那个目标”，不能按类别取最大框：
                // 画面里同类目标的另一个实例（近处瓶子）会把框占比抬上去，导致误判到位
                float max_box_ratio = 0.0f;
                for (const auto& d : latest_dets) {
                    if (sc.target_cls != 0xFF && d.cls != (int)sc.target_cls) continue;
                    if (sc.target_id >= 0 && d.track_id != (int)sc.target_id) continue;
                    max_box_ratio = std::max(
                        max_box_ratio, (d.y2 - d.y1) / (float)frame_img->rows);
                }

                // 检测丢帧时 latest_dets 为空、占比会瞬间变 0，导致「停—再蹭」抖动；
                // 用带衰减的峰值保持，让停车判据不会因为单帧丢检而失效
                static float peak_box_ratio = 0.0f;
                static int64_t centered_since = 0;
                if (sc.mode != (uint8_t)sem::State::APPROACH) {
                    // 不在逼近阶段（新任务/搜索/收尾）清零，避免上次任务的状态残留
                    peak_box_ratio = 0.0f;
                    centered_since = 0;
                } else {
                    peak_box_ratio = std::max(max_box_ratio, peak_box_ratio * 0.96f);
                }

                float vx = 0.0f;
                float wz = 0.0f;
                if (sc.mode == (uint8_t)sem::State::APPROACH) {
                    if (target_ok && max_box_ratio >= stop_ratio) {
                        // 已贴到目标：通知状态机立即结束任务，本帧直接停车
                        semantic_engine.notifyArrived(nowMs());
                        vx = 0.0f;
                        wz = 0.0f;
                    } else if (target_ok) {
                        // 先居中：偏离超过容差只原地转向，居中稳定一小段时间后才起步前进
                        const float abs_bearing = std::fabs(bearing_deg);
                        wz = abs_bearing <= 2.0f
                                 ? 0.0f
                                 : std::clamp(-0.03f * bearing_deg, -0.6f, 0.6f);

                        if (abs_bearing > center_tol) {
                            centered_since = 0;
                        } else if (centered_since == 0) {
                            centered_since = nowMs();
                        }
                        const bool aligned =
                            abs_bearing <= center_tol &&
                            (nowMs() - centered_since >= 200);

                        if (aligned && dist_m > stop_dist &&
                            peak_box_ratio < stop_ratio) {
                            // 缓慢靠近：距离项给速度上限，框占比项做末端线性减速，到 stop_ratio 归零停车
                            const float vx_dist = 0.4f * (dist_m - stop_dist);
                            const float vx_ratio =
                                approach_vmax * (stop_ratio - peak_box_ratio) / stop_ratio;
                            const float vx_ratio_cur =
                                approach_vmax * (stop_ratio - max_box_ratio) / stop_ratio;
                            const float vx_raw = std::min({vx_dist, vx_ratio, vx_ratio_cur});
                            if (vx_raw > 0.0f) {
                                // 留一点爬行速度克服电机死区，接近停车距离时再归零
                                const float vx_min = dist_m > stop_dist + 0.10f ? 0.08f : 0.0f;
                                vx = std::clamp(vx_raw, vx_min, approach_vmax);
                            }
                        }
                    }
                } else if (sc.mode == (uint8_t)sem::State::SEARCH) {
                    // 视野内没有目标：原地缓慢自转搜索；一旦出现目标立即停转，等状态机锁定
                    vx = 0.0f;
                    wz = target_ok ? 0.0f : search_wz;
                }
                // VERIFY / DONE / FAIL / STOP 时 vx=wz=0，立即停车结束任务

                // 视觉链路中断（2s 无新结果）时一律停车，避免按上一帧状态持续自转
                if (last_det_ts > 0 && (nowMs() - last_det_ts) > 2000) {
                    vx = 0.0f;
                    wz = 0.0f;
                }

                char txt[64];
                const int txt_len = std::snprintf(txt, sizeof(txt), "$%.2f,%.2f,%.2f\n",
                                                  vx, 0.0f, wz);
                if (uart_text && uart.isOpen()) uart.sendText(txt);
                if (!cmdvel_fifo_path.empty()) {
                    if (cmdvel_fifo_fd < 0) {
                        // 桥接节点可能晚于 vision 启动，这里按帧重试打开
                        cmdvel_fifo_fd = ::open(cmdvel_fifo_path.c_str(),
                                                O_WRONLY | O_NONBLOCK);
                        if (cmdvel_fifo_fd >= 0) {
                            std::cout << "[ctl] 速度 FIFO 已连接: "
                                      << cmdvel_fifo_path << std::endl;
                        }
                    }
                    if (cmdvel_fifo_fd >= 0 && txt_len > 0) {
                        const ssize_t written = ::write(cmdvel_fifo_fd, txt, (size_t)txt_len);
                        if (written != txt_len) {
                            ::close(cmdvel_fifo_fd);
                            cmdvel_fifo_fd = -1;
                        }
                    }
                }

                static auto last_dbg = std::chrono::steady_clock::time_point{};
                if (now - last_dbg >= std::chrono::seconds(1)) {
                    last_dbg = now;
                    std::cout << "[ctl] state="
                              << sem::stateName(static_cast<sem::State>(sc.mode))
                              << " cls=" << (int)sc.target_cls
                              << " color=" << (int)sc.target_color
                              << " conf=" << (int)sc.conf
                              << " id=" << sc.target_id
                              << " dist=" << dist_m << "m bearing=" << bearing_deg
                              << "deg box=" << max_box_ratio
                              << "/" << peak_box_ratio
                              << " -> vx=" << vx << " wz=" << wz << std::endl;
                }
            } else if (uart.isOpen()) {
                uart.send(vf);
            }
            last_send = now;
        }

        // ---- 显示：始终显示最新采集帧，框沿用最近一次检测结果 ----
        auto t_disp0 = std::chrono::steady_clock::now();
        if (display) {
            cv::Mat show = frame_img->clone();
            drawDetections(show, latest_dets, active_classes);
            if (semantic) drawSemanticState(show, semantic_engine.lastCommand());
            cv::imshow("rk3588_vision", show);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') break;
        }
        auto t_disp1 = std::chrono::steady_clock::now();

        auto loop_end = std::chrono::steady_clock::now();
        double post_ms   = std::chrono::duration<double, std::milli>(t_post1   - t_post0).count();
        double disp_ms   = std::chrono::duration<double, std::milli>(t_disp1   - t_disp0).count();
        double total_ms  = std::chrono::duration<double, std::milli>(loop_end  - loop_start).count();

        post_ms_sum   += post_ms;
        disp_ms_sum   += disp_ms;
        total_ms_sum  += total_ms;
        frame_count++;

        // 每秒统计一次：loop_fps 是采集/显示节奏，det_fps 是实际推理吞吐
        const int64_t now_wall = nowMs();
        if (now_wall - report_start >= 1000) {
            const double secs = (now_wall - report_start) / 1000.0;
            const double avg_total = total_ms_sum / std::max(1, frame_count);
            const double latency =
                last_det_ts > 0 ? (double)(now_wall - last_det_ts) : 0.0;

            std::cout << "[perf] loop_fps=" << (frame_count / secs)
                      << " det_fps=" << (det_count / secs)
                      << " avg_ms=" << avg_total
                      << " (post=" << post_ms_sum / std::max(1, frame_count)
                      << " disp=" << disp_ms_sum / std::max(1, frame_count) << ")"
                      << " latency=" << latency << "ms"
                      << " dets=" << latest_dets.size()
                      << " workers=" << pool_workers;
            if (semantic) std::cout << " state=" << semantic_engine.stateText();
            if (lidar.isOpen()) std::cout << " lidar_scans=" << lidar.scanCount();
            std::cout << std::endl;

            report_start = now_wall;
            frame_count = 0;
            det_count = 0;
            post_ms_sum = 0.0;
            disp_ms_sum = 0.0;
            total_ms_sum = 0.0;
        }
    }

    camera.stop();
    lidar.stop();
    if (uart_text && uart.isOpen()) uart.sendText("$0.000,0.000,0.000\r\n");
    if (cmdvel_fifo_fd >= 0) {
        const char stop[] = "$0.000,0.000,0.000\n";
        (void)::write(cmdvel_fifo_fd, stop, sizeof(stop) - 1);
        ::close(cmdvel_fifo_fd);
    }
    if (voice_fifo_fd >= 0) ::close(voice_fifo_fd);
    if (qa_worker.joinable()) qa_worker.join();
    uart.close();
    std::cout << "退出" << std::endl;
    return 0;
}
