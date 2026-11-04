#pragma once

#include <cstdint>
#include <string>

#include "types.h"
#include "../semantic/semantic_types.h"

// 与 ACG720 的串口链路，3.3V TTL，921600 8N1
class UartLink {
public:
    enum class CrcMode {
        X25,
        MCRF4XX,
    };

    UartLink();
    ~UartLink();

    void setCrcMode(CrcMode mode) { crc_mode_ = mode; }
    bool open(const std::string& device, int baud = 921600);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 自动填充 crc 后发送
    bool send(VisionFrame& frame);
    bool send(sem::SemanticCommand& cmd);

    // 原样发送文本（FPGA 底盘的 "$vx,vy,wz\n" 协议），不填 CRC
    bool sendText(const std::string& text);

private:
    static uint16_t crc16(const uint8_t* data, size_t len, CrcMode mode);

    int fd_ = -1;
    CrcMode crc_mode_ = CrcMode::X25;
};
