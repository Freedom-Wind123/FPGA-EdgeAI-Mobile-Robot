#include "detector_pool.h"

#include <iostream>
#include <utility>

DetectorPool::~DetectorPool() {
    stop();
}

bool DetectorPool::start(int workers, const Factory& factory,
                         const std::string& model_path, float conf_th,
                         float iou_th, int expected_class_count,
                         const std::string& aux_path, int core_base) {
    stop();
    if (workers < 1) workers = 1;
    if (core_base < 0 || core_base > 2) core_base = 0;

    for (int i = 0; i < workers; ++i) {
        std::unique_ptr<IDetector> det = factory();
        if (!det ||
            !det->init(model_path, conf_th, iou_th, expected_class_count, aux_path)) {
            std::cerr << "推理上下文 " << i << " 初始化失败" << std::endl;
            break;
        }
        detectors_.push_back(std::move(det));
    }
    if (detectors_.empty()) return false;

    worker_count_ = (int)detectors_.size();
    if (worker_count_ < workers) {
        std::cerr << "警告: 仅 " << worker_count_ << " 个推理上下文可用，"
                  << "并行度低于请求的 " << workers << std::endl;
    }

    // 每个上下文固定绑到一颗 NPU 核，避免驱动把并发推理挤到同一核上
    for (int i = 0; i < worker_count_; ++i) {
        const int core = (core_base + i) % 3;
        if (detectors_[i]->bindCore(core)) {
            std::cout << "  worker " << i << " -> NPU core " << core << std::endl;
        } else {
            std::cerr << "  警告: worker " << i << " 绑定 NPU core " << core
                      << " 失败，交由驱动调度" << std::endl;
        }
    }

    running_ = true;
    threads_.reserve(worker_count_);
    for (int i = 0; i < worker_count_; ++i) {
        threads_.emplace_back(&DetectorPool::workerLoop, this, i);
    }
    return true;
}

void DetectorPool::stop() {
    running_ = false;
    job_cv_.notify_all();
    for (auto& t : threads_) {
        if (t.joinable()) t.join();
    }
    threads_.clear();

    {
        std::lock_guard<std::mutex> lk(job_mtx_);
        has_pending_ = false;
        pending_ = Job{};
    }
    {
        std::lock_guard<std::mutex> lk(res_mtx_);
        has_result_ = false;
        latest_ = Result{};
    }

    detectors_.clear();
    worker_count_ = 0;
}

void DetectorPool::submit(std::shared_ptr<const cv::Mat> image, uint64_t seq,
                          int64_t timestamp_ms) {
    {
        std::lock_guard<std::mutex> lk(job_mtx_);
        pending_.image = std::move(image);
        pending_.seq = seq;
        pending_.timestamp_ms = timestamp_ms;
        has_pending_ = true;
    }
    job_cv_.notify_one();
}

bool DetectorPool::takeLatest(Result& out) {
    std::lock_guard<std::mutex> lk(res_mtx_);
    if (!has_result_) return false;
    out = std::move(latest_);
    latest_ = Result{};
    has_result_ = false;
    return true;
}

void DetectorPool::workerLoop(int index) {
    while (true) {
        Job job;
        {
            std::unique_lock<std::mutex> lk(job_mtx_);
            job_cv_.wait(lk, [this] { return has_pending_ || !running_; });
            if (!running_ && !has_pending_) return;
            job = std::move(pending_);
            pending_ = Job{};
            has_pending_ = false;
        }

        std::vector<Detection> dets;
        if (index < (int)detectors_.size() && job.image &&
            !job.image->empty()) {
            dets = detectors_[index]->infer(*job.image);
        }

        {
            std::lock_guard<std::mutex> lk(res_mtx_);
            if (!has_result_ || job.seq >= latest_.seq) {
                latest_.dets = std::move(dets);
                latest_.image = job.image;
                latest_.seq = job.seq;
                latest_.timestamp_ms = job.timestamp_ms;
                has_result_ = true;
            }
        }
    }
}
