#pragma once

#include <string>
#include <vector>

#include "semantic_types.h"

namespace sem {

// 把一句话指令解析成 Intent，支持中文关键字和英文类别名
class CommandParser {
public:
    void setClasses(const std::vector<std::string>& names);

    // 例如："找红色瓶子" / "approach the nearest person" / "stop"
    bool parse(const std::string& text, Intent& out) const;

private:
    int findClassIndex(const std::string& canonical) const;

    std::vector<std::string> names_;   // 已转小写，与 classes.txt 下标一致
};

// 文本里是否含“找/寻找/靠近”等需要目标的动作词，用于残句提示
bool hasTargetActionVerb(const std::string& text);

}  // namespace sem
