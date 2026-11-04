#include "multi_model.h"

#include "detector.h"
#include "split_head_detector.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

namespace {

std::vector<std::string> readClassNames(const std::string& path) {
    std::vector<std::string> names;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        names.push_back(line);
    }
    return names;
}

std::unique_ptr<IDetector> makeDetector(const std::string& route) {
    if (route == "yolov8") return std::make_unique<Detector>();
    if (route == "split") return std::make_unique<SplitHeadDetector>();
    return nullptr;
}

float boxIou(const Detection& a, const Detection& b) {
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float iw = ix2 - ix1;
    const float ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;

    const float inter = iw * ih;
    const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    const float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

// 行内 '#' 之后视为注释
std::string stripComment(const std::string& line) {
    const size_t pos = line.find('#');
    return pos == std::string::npos ? line : line.substr(0, pos);
}

}  // namespace

bool loadModelSpecs(const std::string& path, std::vector<ModelSpec>& out) {
    out.clear();

    std::ifstream in(path);
    if (!in) {
        std::cerr << "无法打开模型配置: " << path << std::endl;
        return false;
    }

    std::string line;
    int lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = stripComment(line);

        std::istringstream iss(line);
        std::vector<std::string> tok;
        std::string t;
        while (iss >> t) tok.push_back(t);
        if (tok.empty()) continue;

        if (tok.size() < 7) {
            std::cerr << path << ":" << lineno
                      << " 字段不足 7 个，已跳过: " << line << std::endl;
            continue;
        }

        ModelSpec s;
        s.name = tok[0];
        s.route = tok[1];
        s.model_path = tok[2];
        s.classes_path = tok[3];

        bool bad = false;
        try {
            s.core = std::stoi(tok[4]);
            s.every = std::stoi(tok[5]);
            s.conf = std::stof(tok[6]);
        } catch (...) {
            bad = true;
        }
        if (bad) {
            std::cerr << path << ":" << lineno
                      << " core/every/conf 不是合法数字，已跳过: " << line << std::endl;
            continue;
        }

        // 父类名可含空格，位于第 8 个字段之后；也接受 "parent=xxx" 写法，
        // 前置 drop 表示子类同框时丢弃父类粗框；
        // "require=cls1,cls2" 表示本模型的框必须与这些类别同框才保留
        std::vector<std::string> ptok(tok.begin() + 7, tok.end());
        if (!ptok.empty() && ptok[0] == "drop") {
            s.suppress_parent = true;
            ptok.erase(ptok.begin());
        }
        std::string parent;
        for (size_t i = 0; i < ptok.size(); ++i) {
            if (ptok[i].rfind("require=", 0) == 0) {
                std::string list = ptok[i].substr(8);
                size_t start = 0;
                while (start <= list.size()) {
                    const size_t comma = list.find(',', start);
                    const std::string item = list.substr(
                        start, comma == std::string::npos ? std::string::npos
                                                          : comma - start);
                    if (!item.empty()) s.require_names.push_back(item);
                    if (comma == std::string::npos) break;
                    start = comma + 1;
                }
                continue;
            }
            if (!parent.empty()) parent += ' ';
            parent += ptok[i];
        }
        if (parent.rfind("parent=", 0) == 0) parent = parent.substr(7);
        if (parent == "-") parent.clear();
        s.parent_name = parent;
        if (s.suppress_parent && s.parent_name.empty()) {
            std::cerr << path << ":" << lineno
                      << " drop 需要同时给出父类名，已跳过: " << line << std::endl;
            continue;
        }

        if (s.route != "yolov8" && s.route != "split") {
            std::cerr << path << ":" << lineno
                      << " route 仅支持 yolov8 / split，已跳过: " << s.route << std::endl;
            continue;
        }
        if (s.core < 0 || s.core > 2) {
            std::cerr << path << ":" << lineno
                      << " core 必须是 0/1/2，已跳过: " << s.core << std::endl;
            continue;
        }
        if (s.every < 1) s.every = 1;
        if (!std::isfinite(s.conf) || s.conf <= 0.0f || s.conf > 1.0f) {
            std::cerr << path << ":" << lineno
                      << " conf 必须在 (0, 1]，已跳过: " << s.conf << std::endl;
            continue;
        }

        s.classes = readClassNames(s.classes_path);
        if (s.classes.empty()) {
            std::cerr << path << ":" << lineno
                      << " 类别表为空或不存在: " << s.classes_path << std::endl;
            continue;
        }

        out.push_back(std::move(s));
    }

    if (out.empty()) {
        std::cerr << "模型配置里没有可用模型: " << path << std::endl;
        return false;
    }

    std::cout << "模型配置 " << path << " 载入 " << out.size() << " 个模型:"
              << std::endl;
    for (const auto& s : out) {
        std::cout << "  " << s.name << " route=" << s.route
                  << " core=" << s.core << " every=" << s.every
                  << " conf=" << s.conf << " classes=" << s.classes.size()
                  << " parent=" << (s.parent_name.empty() ? "-" : s.parent_name)
                  << (s.suppress_parent ? "(drop)" : "")
                  << std::endl;
    }
    return true;
}

int UnifiedClassRegistry::add(const std::string& name) {
    for (size_t i = 0; i < names_.size(); ++i) {
        if (names_[i] == name) return (int)i;
    }
    if (names_.size() >= 255) {
        std::cerr << "统一类别表超过 255 类，TargetMsg.cls 放不下: " << name
                  << std::endl;
        return -1;
    }
    names_.push_back(name);
    return (int)names_.size() - 1;
}

int UnifiedClassRegistry::find(const std::string& name) const {
    for (size_t i = 0; i < names_.size(); ++i) {
        if (names_[i] == name) return (int)i;
    }
    return -1;
}

MultiModelRunner::~MultiModelRunner() {
    stop();
}

bool MultiModelRunner::init(const std::vector<ModelSpec>& specs, float iou_th) {
    stop();
    models_.clear();
    registry_ = UnifiedClassRegistry();

    // base 必须最先注册，保证 COCO 类别仍是 0~79（语义层按 COCO 下标估高）
    std::vector<ModelSpec> ordered = specs;
    for (size_t i = 1; i < ordered.size(); ++i) {
        if (ordered[i].name == "base") {
            std::rotate(ordered.begin(), ordered.begin() + (long)i,
                        ordered.begin() + (long)i + 1);
            break;
        }
    }

    for (const auto& s : ordered) {
        const DetectorPool::Factory factory =
            [route = s.route]() -> std::unique_ptr<IDetector> {
            return makeDetector(route);
        };

        auto entry = std::make_unique<Entry>();
        entry->spec = s;
        entry->index = (int)models_.size();
        entry->pool = std::make_unique<DetectorPool>();
        if (!entry->pool->start(1, factory, s.model_path, s.conf, iou_th,
                                (int)s.classes.size(), std::string(), s.core)) {
            std::cerr << "模型 " << s.name << " 初始化失败，已跳过" << std::endl;
            continue;
        }

        for (const auto& name : s.classes) {
            const int gid = registry_.add(name);
            if (gid < 0) {
                std::cerr << "模型 " << s.name << " 的类别超出统一表上限，放弃"
                          << std::endl;
                entry->pool->stop();
                return false;
            }
            entry->local_to_global.push_back(gid);
        }
        if (!s.parent_name.empty()) {
            entry->parent_id = registry_.add(s.parent_name);
            if (entry->parent_id < 0) {
                std::cerr << "模型 " << s.name << " 的父类注册失败，放弃" << std::endl;
                entry->pool->stop();
                return false;
            }
        }
        for (const auto& name : s.require_names) {
            const int id = registry_.find(name);
            if (id < 0) {
                std::cerr << "模型 " << s.name << " 的锚类 " << name
                          << " 不在统一类别表中，已忽略该约束" << std::endl;
                continue;
            }
            entry->require_ids.push_back(id);
        }

        std::string require_desc;
        for (const auto& name : s.require_names) {
            if (!require_desc.empty()) require_desc += ',';
            require_desc += name;
        }
        std::cout << "模型 " << s.name << " 就绪: route=" << s.route
                  << " core=" << s.core << " every=" << s.every
                  << " conf=" << s.conf << " classes=" << s.classes.size()
                  << " parent=" << (s.parent_name.empty() ? "-" : s.parent_name)
                  << (s.suppress_parent ? "(drop)" : "")
                  << " require=" << (require_desc.empty() ? "-" : require_desc)
                  << std::endl;
        models_.push_back(std::move(entry));
    }

    if (models_.empty()) {
        std::cerr << "所有模型都初始化失败" << std::endl;
        return false;
    }

    std::cout << "统一类别表共 " << registry_.size() << " 类" << std::endl;
    return true;
}

void MultiModelRunner::stop() {
    for (auto& m : models_) {
        if (m->pool) m->pool->stop();
    }
    models_.clear();
}

void MultiModelRunner::submit(std::shared_ptr<const cv::Mat> image, uint64_t seq,
                              int64_t timestamp_ms) {
    for (auto& m : models_) {
        if (!m->pool) continue;
        const int every = std::max(1, m->spec.every);
        // 帧计数降频：高开销模型每隔 N 帧才投递一次，主循环节奏不变
        if (++m->frames_since < every) continue;
        m->frames_since = 0;
        m->pool->submit(image, seq, timestamp_ms);
    }
}

bool MultiModelRunner::takeLatest(Result& out) {
    bool any_new = false;
    for (auto& m : models_) {
        if (!m->pool) continue;
        DetectorPool::Result r;
        if (!m->pool->takeLatest(r)) continue;
        any_new = true;

        std::vector<Detection> mapped;
        mapped.reserve(r.dets.size());
        for (const auto& d : r.dets) {
            if (d.cls < 0 || d.cls >= (int)m->local_to_global.size()) continue;
            Detection nd = d;
            nd.cls = m->local_to_global[(size_t)d.cls];
            nd.source = (uint8_t)m->index;
            // 父类 ID 是该类别的语义属性，不论本帧父类框是否出现都带上
            nd.parent = (int16_t)m->parent_id;
            mapped.push_back(nd);
        }

        m->last_dets = std::move(mapped);
        m->last_image = r.image;
        m->last_seq = r.seq;
        m->last_ts = r.timestamp_ms;
        m->has_result = true;
    }
    if (!any_new) return false;

    std::vector<Detection> all;
    const Entry* newest = nullptr;
    for (const auto& m : models_) {
        if (!m->has_result) continue;
        all.insert(all.end(), m->last_dets.begin(), m->last_dets.end());
        if (!newest || m->last_seq > newest->last_seq) newest = m.get();
    }

    // 锚定类约束（require=...）：例如车位框必须与自行车/摩托车同框，
    // 否则柜子、架子等外形相近的区域会被误判成占用车位。
    std::vector<Detection> anchored;
    anchored.reserve(all.size());
    for (const auto& d : all) {
        const Entry* owner =
            (d.source < models_.size()) ? models_[d.source].get() : nullptr;
        if (!owner || owner->require_ids.empty()) {
            anchored.push_back(d);
            continue;
        }
        bool matched = false;
        for (const auto& o : all) {
            if (o.source == d.source) continue;
            if (std::find(owner->require_ids.begin(), owner->require_ids.end(),
                          o.cls) == owner->require_ids.end()) {
                continue;
            }
            const float cx = (o.x1 + o.x2) * 0.5f;
            const float cy = (o.y1 + o.y2) * 0.5f;
            const bool center_inside =
                cx >= d.x1 && cx <= d.x2 && cy >= d.y1 && cy <= d.y2;
            if (center_inside || boxIou(o, d) >= 0.2f) {
                matched = true;
                break;
            }
        }
        if (matched) anchored.push_back(d);
    }
    all.swap(anchored);

    // 人物属性模型（community / non-community）只输出二分类框，本身不认人：
    // 只保留与真实 person 框同框的属性框，且每个 person 最多保留一个，
    // 否则背景/其他物体上会冒出大量属性框，画面会很杂乱。
    const int person_id = registry_.find("person");
    if (person_id >= 0) {
        std::vector<Detection> attrs;
        std::vector<Detection> others;
        attrs.reserve(all.size());
        others.reserve(all.size());
        for (const auto& d : all) {
            if (d.cls != person_id && d.parent == person_id) attrs.push_back(d);
            else others.push_back(d);
        }

        std::vector<size_t> persons;
        for (size_t i = 0; i < others.size(); ++i) {
            if (others[i].cls == person_id) persons.push_back(i);
        }

        std::sort(attrs.begin(), attrs.end(),
                  [](const Detection& a, const Detection& b) {
                      return a.conf > b.conf;
                  });
        std::vector<bool> used(persons.size(), false);
        std::vector<Detection> kept_attrs;
        for (const auto& a : attrs) {
            size_t best = persons.size();
            float best_iou = 0.0f;
            for (size_t k = 0; k < persons.size(); ++k) {
                if (used[k]) continue;
                const float iou = boxIou(others[persons[k]], a);
                if (iou > best_iou) {
                    best_iou = iou;
                    best = k;
                }
            }
            bool inside = false;
            if (best < persons.size()) {
                const Detection& p = others[persons[best]];
                const float cx = (a.x1 + a.x2) * 0.5f;
                const float cy = (a.y1 + a.y2) * 0.5f;
                inside = cx >= p.x1 && cx <= p.x2 && cy >= p.y1 && cy <= p.y2;
            }
            if (best < persons.size() && (best_iou >= 0.3f || inside)) {
                used[best] = true;
                kept_attrs.push_back(a);
            }
        }
        all.swap(others);
        all.insert(all.end(), kept_attrs.begin(), kept_attrs.end());
    }

    // 状态类模型（drop）：split 已给出更细的状态，同一物体不再保留父类粗框
    std::vector<bool> suppress(models_.size(), false);
    for (const auto& m : models_) suppress[(size_t)m->index] = m->spec.suppress_parent;

    std::vector<Detection> kept;
    kept.reserve(all.size());
    for (size_t i = 0; i < all.size(); ++i) {
        const Detection& child = all[i];
        bool covered = false;
        if (child.parent >= 0 && child.source < suppress.size() &&
            suppress[child.source]) {
            for (size_t j = 0; j < all.size() && !covered; ++j) {
                if (j == i || all[j].cls != child.parent) continue;
                covered = boxIou(all[j], child) >= 0.6f;
            }
        }
        if (!covered) kept.push_back(child);
    }

    out.dets = mergeDetections(std::move(kept));
    out.image = newest ? newest->last_image : nullptr;
    out.seq = newest ? newest->last_seq : 0;
    out.timestamp_ms = newest ? newest->last_ts : 0;
    return true;
}

std::vector<Detection> mergeDetections(std::vector<Detection> dets) {
    // 先按置信度降序，重复框保留靠前（也就置信度更高）的那条
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) {
                  return a.conf > b.conf;
              });

    std::vector<Detection> kept;
    kept.reserve(dets.size());
    for (const auto& d : dets) {
        bool dup = false;
        for (const auto& k : kept) {
            if (k.cls == d.cls && boxIou(k, d) >= 0.6f) {
                dup = true;
                break;
            }
        }
        if (!dup) kept.push_back(d);
    }
    return kept;
}
