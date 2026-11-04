#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "detector_pool.h"
#include "types.h"

// 一个模型配置行（models.txt）
struct ModelSpec {
    std::string name;
    std::string route;        // yolov8 / split
    std::string model_path;
    std::string classes_path;
    int core = 0;             // 绑定的 NPU 核
    int every = 1;            // 每 N 帧投递一次
    float conf = 0.25f;       // 该模型自己的置信度阈值
    std::string parent_name;  // 父类名，空表示无父类
    bool suppress_parent = false;  // true 时子类同框覆盖父类框（状态类模型用）
    std::vector<std::string> require_names;  // 必须同框存在的锚类名（如车位需有自行车）
    std::vector<std::string> classes;
};

// 解析 models.txt：跳过注释行与空行，字段不合法只告警并跳过该行
bool loadModelSpecs(const std::string& path, std::vector<ModelSpec>& out);

// 统一类别表：按名字去重；base 先注册以保持 COCO 0~79 下标不变
class UnifiedClassRegistry {
public:
    // 已存在则复用旧 ID，否则追加；超过 255 类返回 -1
    int add(const std::string& name);
    int find(const std::string& name) const;
    const std::vector<std::string>& names() const { return names_; }
    int size() const { return (int)names_.size(); }

private:
    std::vector<std::string> names_;
};

// 多模型并行：每个模型一个独立推理池，按 every 降频投递，
// 各模型结果重映射到统一类别 ID 后合并去重，主循环只看到一份检测列表
class MultiModelRunner {
public:
    struct Result {
        std::vector<Detection> dets;
        std::shared_ptr<const cv::Mat> image;  // 最新一帧结果对应画面
        uint64_t seq = 0;
        int64_t timestamp_ms = 0;
    };

    MultiModelRunner() = default;
    ~MultiModelRunner();
    MultiModelRunner(const MultiModelRunner&) = delete;
    MultiModelRunner& operator=(const MultiModelRunner&) = delete;

    // 单个模型初始化失败只跳过该模型；全部失败返回 false
    bool init(const std::vector<ModelSpec>& specs, float iou_th);
    void stop();

    void submit(std::shared_ptr<const cv::Mat> image, uint64_t seq,
                int64_t timestamp_ms);

    // 任一模型有新结果时返回 true；合并各模型最近一次结果
    // 规则：split 更细，状态类模型（suppress_parent）的父类框被同框子类覆盖时丢弃；
    //       人物属性框（community / non-community）只保留与 person 框同框的，
    //       每个 person 最多一个，避免背景区域出现大量无主属性框；
    //       带 require 的模型（如车位）只保留与锚类（如 bicycle）同框的框；
    //       同统一类别 IoU≥0.6 视为重复，保留置信度高的
    bool takeLatest(Result& out);

    const std::vector<std::string>& classNames() const { return registry_.names(); }
    int modelCount() const { return (int)models_.size(); }

private:
    struct Entry {
        ModelSpec spec;
        std::vector<int> local_to_global;  // 模型本地类别 -> 统一类别
        int parent_id = -1;
        std::vector<int> require_ids;      // 锚类在统一类别表中的 ID
        int index = 0;                     // 供 Detection.source 使用
        std::unique_ptr<DetectorPool> pool;
        int frames_since = 0;
        std::vector<Detection> last_dets;  // 已重映射的最近一次结果
        std::shared_ptr<const cv::Mat> last_image;
        uint64_t last_seq = 0;
        int64_t last_ts = 0;
        bool has_result = false;
    };

    std::vector<std::unique_ptr<Entry>> models_;
    UnifiedClassRegistry registry_;
};

// 合并规则：同统一类别且 IoU≥0.6 视为重复，保留置信度高的；其余全保留
std::vector<Detection> mergeDetections(std::vector<Detection> dets);
