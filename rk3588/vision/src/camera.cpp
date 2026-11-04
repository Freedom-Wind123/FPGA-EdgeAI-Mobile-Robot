#include "camera.h"

#include <chrono>
#include <utility>

Camera::Camera(std::string device, int width, int height, int fps)
    : device_(std::move(device)), width_(width), height_(height), fps_(fps) {}

Camera::~Camera() {
    stop();
}

bool Camera::open() {
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        cap_.open(device_, cv::CAP_ANY);
    }
    if (!cap_.isOpened()) {
        opened_ = false;
        return false;
    }

    // UVC 相机优先用 MJPG，720p/1080p 下 USB 带宽更稳
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);
    cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);

    opened_ = true;
    return true;
}

void Camera::start() {
    if (!opened_) return;
    running_ = true;
    thread_ = std::thread(&Camera::captureLoop, this);
}

void Camera::stop() {
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
    opened_ = false;
}

void Camera::captureLoop() {
    cv::Mat raw;
    while (running_) {
        if (!cap_.read(raw) || raw.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 主线程还没消费上一帧 → 直接跳过 MJPG 解码，避免 CPU 浪费
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (has_new_) continue;
        }

        cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
        if (frame.empty()) continue;

        Frame f;
        f.image = std::move(frame);
        f.seq = ++seq_;
        f.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_ = std::move(f);
            has_new_ = true;
        }
        cv_.notify_one();
    }
}

bool Camera::getLatest(Frame& out, int timeout_ms) {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                      [this] { return has_new_ || !running_; })) {
        return false;
    }
    if (!has_new_) return false;

    out = std::move(latest_);
    has_new_ = false;
    return true;
}
