#pragma once

#include <cstdint>

#include "semantic_types.h"

class UartLink;

namespace sem {

// 补帧头/序号/时间戳并发送语义命令
class ActionInterface {
public:
    explicit ActionInterface(UartLink* uart = nullptr) : uart_(uart) {}

    void setUart(UartLink* uart) { uart_ = uart; }

    bool publish(SemanticCommand& cmd, int64_t now_ms);

    uint32_t nextSeq() const { return seq_; }

private:
    UartLink* uart_ = nullptr;
    uint32_t seq_ = 0;
};

}  // namespace sem
