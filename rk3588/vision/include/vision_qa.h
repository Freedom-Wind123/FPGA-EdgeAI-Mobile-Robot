#pragma once

#include <string>
#include <vector>

#include "types.h"

// 视觉问答：「这是什么」类问句 → 当前画面目标 → DeepSeek 组织口语化回答
//
// 调用时序（全部在主循环内发起，网络请求放后台线程）：
//   1. isSceneQuestion 判定问句
//   2. buildSceneDescription / localSceneAnswer 快照当前检测结果
//   3. 后台线程 askDeepSeek，失败回退本地模板，结果送语音模块播报

// 判定是否为「这是什么/看到了什么」类视觉问句
bool isSceneQuestion(const std::string& text);

// 把检测结果整理成结构化场景描述（DeepSeek 提示词用）
std::string buildSceneDescription(const std::vector<Detection>& dets,
                                  const std::vector<std::string>& names,
                                  int img_w, size_t max_items = 6);

// 调 DeepSeek 生成口语化回答；无网络/无 key/超时返回空串
std::string askDeepSeek(const std::string& api_key,
                        const std::string& question,
                        const std::string& scene_desc);

// 本地模板兜底回答（DeepSeek 不可用时用）
std::string localSceneAnswer(const std::vector<Detection>& dets,
                             const std::vector<std::string>& names,
                             int img_w, size_t max_items = 3);

// 本地规则问答：按问句类型（人数 / 身份 / 单个物体 / 场景）拼一句话回答
std::string localAnswerForQuestion(const std::string& question,
                                   const std::vector<Detection>& dets,
                                   const std::vector<std::string>& names,
                                   int img_w, size_t max_items = 3);
