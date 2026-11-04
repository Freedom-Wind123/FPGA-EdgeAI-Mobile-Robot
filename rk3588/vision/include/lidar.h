#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// RPLIDAR C1/A1/A2 等 2D 扫描雷达
// 独立线程读串口，只保留最新一圈扫描结果；按角度窗口查询距离
class Lidar {
public:
    Lidar() = default;
    ~Lidar();

    bool open(const std::string& device, int baud);
    void start();
    void stop();
    bool isOpen() const { return fd_ >= 0; }

    // 以 center_deg 为中心、半角 half_span_deg 内最近的返回点，单位米
    // 没有有效回波或数据过期返回 false
    bool distanceInWindow(float center_deg, float half_span_deg,
                          float& dist_m) const;

    uint64_t scanCount() const { return scan_count_; }

private:
    bool sendScan();
    void readLoop();
    void publishScan(const std::vector<float>& bins);

    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> scan_count_{0};
    std::thread thread_;

    mutable std::mutex mtx_;
    std::vector<float> bins_;   // 720 格，每格 0.5 度，0 表示无回波
    int64_t last_scan_ms_ = 0;
};
