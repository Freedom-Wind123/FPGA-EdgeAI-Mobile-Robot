#include "action_interface.h"

#include "uart_link.h"

namespace sem {

bool ActionInterface::publish(SemanticCommand& cmd, int64_t now_ms) {
    cmd.magic = 0xAA56;
    cmd.ver = 1;
    cmd.seq = seq_++;
    cmd.tick_ms = static_cast<uint32_t>(now_ms & 0xFFFFFFFF);

    if (!uart_ || !uart_->isOpen()) return false;
    return uart_->send(cmd);
}

}  // namespace sem
