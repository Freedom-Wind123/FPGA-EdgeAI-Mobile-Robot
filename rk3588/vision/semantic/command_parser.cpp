#include "command_parser.h"

#include <algorithm>
#include <cctype>
#include <initializer_list>

namespace sem {
namespace {

std::string lowerAscii(const std::string& s) {
    std::string out = s;
    for (char& c : out) {
        unsigned char u = static_cast<unsigned char>(c);
        if (u < 128) c = static_cast<char>(std::tolower(u));
    }
    return out;
}

bool containsAny(const std::string& hay,
                 std::initializer_list<const char*> words) {
    for (const char* w : words) {
        if (hay.find(w) != std::string::npos) return true;
    }
    return false;
}

// ASCII 单词边界匹配，避免 "cup" 命中 "cupboard"
bool containsWord(const std::string& hay, const std::string& word) {
    if (word.empty()) return false;
    size_t pos = hay.find(word);
    while (pos != std::string::npos) {
        bool left_ok = (pos == 0) ||
                       !std::isalnum(static_cast<unsigned char>(hay[pos - 1]));
        size_t end = pos + word.size();
        bool right_ok = (end >= hay.size()) ||
                        !std::isalnum(static_cast<unsigned char>(hay[end]));
        if (left_ok && right_ok) return true;
        pos = hay.find(word, pos + 1);
    }
    return false;
}

bool hasStopVerb(const std::string& s) {
    return containsAny(s, {"停止", "停车", "刹车", "stop", "halt"});
}

bool hasReportVerb(const std::string& s) {
    return containsAny(s, {"报告", "汇报", "状态", "report", "status"});
}

bool hasApproachVerb(const std::string& s) {
    return containsAny(s, {"靠近", "走近", "前往", "过去", "跟随", "跟住",
                           "跟着", "追", "approach", "follow", "track",
                           "go to", "goto"});
}

bool hasFindVerb(const std::string& s) {
    return containsAny(s, {"找", "寻找", "搜索", "查找", "find", "search",
                           "look for"});
}

// ASR 对短指令的常见近音误识别，解析前做保守替换
std::string normalizeCommandText(const std::string& s) {
    struct TextFix {
        const char* from;
        const char* to;
    };
    const TextFix kFixes[] = {
        {"水平", "水瓶"},
        {"水并", "水瓶"},
    };
    std::string out = s;
    for (const auto& fix : kFixes) {
        const std::string from = fix.from;
        const std::string to = fix.to;
        size_t pos = 0;
        while ((pos = out.find(from, pos)) != std::string::npos) {
            out.replace(pos, from.size(), to);
            pos += to.size();
        }
    }
    return out;
}

struct ClassAlias {
    const char* word;
    const char* canonical;
};

// 中文口语别名 -> classes.txt 里的英文类别名
const ClassAlias kAliases[] = {
    {"人", "person"},         {"行人", "person"},
    {"自行车", "bicycle"},     {"汽车", "car"},
    {"摩托车", "motorcycle"},  {"飞机", "airplane"},
    {"椅子", "chair"},         {"沙发", "couch"},
    {"瓶子", "bottle"},        {"水瓶", "bottle"},
    {"杯子", "cup"},           {"水杯", "cup"},
    {"碗", "bowl"},            {"香蕉", "banana"},
    {"苹果", "apple"},         {"橙子", "orange"},
    {"桌子", "dining table"},  {"餐桌", "dining table"},
    {"笔记本", "laptop"},      {"电脑", "laptop"},
    {"鼠标", "mouse"},         {"键盘", "keyboard"},
    {"手机", "cell phone"},    {"书", "book"},
    {"球", "sports ball"},     {"背包", "backpack"},
    {"狗", "dog"},             {"猫", "cat"},
    {"花瓶", "vase"},          {"玩具熊", "teddy bear"},
};

}  // namespace

void CommandParser::setClasses(const std::vector<std::string>& names) {
    names_.clear();
    names_.reserve(names.size());
    for (const auto& n : names) {
        std::string s = lowerAscii(n);
        while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
        names_.push_back(s);
    }
}

int CommandParser::findClassIndex(const std::string& canonical) const {
    for (size_t i = 0; i < names_.size(); ++i) {
        if (names_[i] == canonical) return static_cast<int>(i);
    }
    return -1;
}

bool CommandParser::parse(const std::string& text, Intent& out) const {
    out = Intent{};
    std::string s = lowerAscii(text);
    s = normalizeCommandText(s);
    if (s.find_first_not_of(" \t\r\n") == std::string::npos) return false;

    if (hasStopVerb(s)) {
        out.action = Action::STOP;
    } else if (hasReportVerb(s)) {
        out.action = Action::REPORT;
    } else if (hasApproachVerb(s)) {
        out.action = Action::APPROACH;
    } else if (hasFindVerb(s)) {
        out.action = Action::FIND;
    } else {
        out.action = Action::FIND;
    }

    if (containsAny(s, {"红色", "红", "red"})) {
        out.target.color = COLOR_RED;
    } else if (containsAny(s, {"绿色", "绿", "green"})) {
        out.target.color = COLOR_GREEN;
    } else if (containsAny(s, {"蓝色", "蓝", "blue"})) {
        out.target.color = COLOR_BLUE;
    }

    if (containsAny(s, {"最近", "nearest", "closest"})) {
        out.target.relation = TargetDesc::NEAREST;
    } else if (containsAny(s, {"左边", "左侧", "左", "left"})) {
        out.target.relation = TargetDesc::LEFT;
    } else if (containsAny(s, {"右边", "右侧", "右", "right"})) {
        out.target.relation = TargetDesc::RIGHT;
    } else if (containsAny(s, {"中间", "中央", "center", "centre"})) {
        out.target.relation = TargetDesc::CENTER;
    }

    // 先按 classes.txt 里的英文名直接匹配，再查中文别名
    int best_cls = -1;
    size_t best_len = 0;
    for (size_t i = 0; i < names_.size(); ++i) {
        const std::string& n = names_[i];
        if (n.empty()) continue;
        if (containsWord(s, n) && n.size() > best_len) {
            best_cls = static_cast<int>(i);
            best_len = n.size();
        }
    }
    for (const auto& a : kAliases) {
        std::string word = a.word;
        if (s.find(word) == std::string::npos) continue;
        int idx = findClassIndex(a.canonical);
        if (idx >= 0 && word.size() > best_len) {
            best_cls = idx;
            best_len = word.size();
        }
    }
    out.target.cls = best_cls;

    const bool has_target = (best_cls >= 0) || (out.target.color != COLOR_ANY);
    const bool needs_target =
        (out.action == Action::FIND || out.action == Action::APPROACH);
    if (needs_target && !has_target) return false;

    out.valid = true;
    return true;
}

bool hasTargetActionVerb(const std::string& text) {
    const std::string s = normalizeCommandText(lowerAscii(text));
    return hasApproachVerb(s) || hasFindVerb(s);
}

}  // namespace sem
