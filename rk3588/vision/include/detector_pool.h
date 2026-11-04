#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "detector_base.h"
#include "types.h"

// 多上下文异步推理池：每个 worker 独占一个 RKNN 上下文，
// 主线程只投递最新帧、取最新结果，采集与显示不再被推理耗时拖住。
class DetectorPool {
public:
    struct Result {
        std::vector<Detection> dets;
        std::shared_ptr<const cv::Mat> image;  // 产生这批检测结果的那一帧
        uint64_t seq = 0;
        int64_t timestamp_ms = 0;
    };

    using Factory = std::function<std::unique_ptr<IDetector>()>;

    DetectorPool() = default;
    ~DetectorPool();
    DetectorPool(const DetectorPool&) = delete;
    DetectorPool& operator=(const DetectorPool&) = delete;

    // workers 为请求的并行上下文数；初始化失败的上下文会被跳过，
    // 只要有至少一个成功就返回 true
    // core_base 为起始 NPU 核，worker i 绑到 (core_base + i) % 3
    bool start(int workers, const Factory& factory,
               const std::string& model_path, float conf_th, float iou_th,
               int expected_class_count, const std::string& aux_path,
               int core_base = 0);
    void stop();

    // 投递最新帧；还没被 worker 取走的旧帧直接丢弃，避免延迟累积
    void submit(std::shared_ptr<const cv::Mat> image, uint64_t seq,
                int64_t timestamp_ms);

    // 取最新一次推理结果，没有新结果返回 false
    bool takeLatest(Result& out);

    int workers() const { return worker_count_; }

private:
    struct Job {
        std::shared_ptr<const cv::Mat> image;
        uint64_t seq = 0;
        int64_t timestamp_ms = 0;
    };

    void workerLoop(int index);

    std::vector<std::unique_ptr<IDetector>> detectors_;
    std::vector<std::thread> threads_;
    int worker_count_ = 0;

    std::mutex job_mtx_;
    std::condition_variable job_cv_;
    Job pending_;
    bool has_pending_ = false;

    std::mutex res_mtx_;
    Result latest_;
    bool has_result_ = false;

    std::atomic<bool> running_{false};
};
