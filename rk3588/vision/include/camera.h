#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

struct Frame {
    cv::Mat image;
    uint64_t seq = 0;
    int64_t timestamp_ms = 0;
};

// UVC 相机采集，只保留最新一帧，避免推理慢时延迟累积
class Camera {
public:
    Camera(std::string device, int width, int height, int fps);
    ~Camera();

    bool open();
    void start();
    void stop();

    // 取最新一帧；超时没有新帧返回 false
    bool getLatest(Frame& out, int timeout_ms = 200);
    bool isOpened() const { return opened_.load(); }

private:
    void captureLoop();

    std::string device_;
    int width_;
    int height_;
    int fps_;
    cv::VideoCapture cap_;
    std::thread thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    Frame latest_;
    uint64_t seq_ = 0;
    bool has_new_ = false;
    std::atomic<bool> running_{false};
    std::atomic<bool> opened_{false};
};
