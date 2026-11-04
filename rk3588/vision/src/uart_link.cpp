#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "uart_link.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace {

speed_t toSpeed(int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 460800: return B460800;
#ifdef B921600
        case 921600: return B921600;
#endif
        default:
            std::cerr << "unsupported baud " << baud
                      << ", fallback to 115200" << std::endl;
            return B115200;
    }
}

}  // namespace

UartLink::UartLink() = default;

UartLink::~UartLink() {
    close();
}

bool UartLink::open(const std::string& device, int baud) {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "open uart failed: " << device
                  << " (" << std::strerror(errno) << ")" << std::endl;
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "tcgetattr failed" << std::endl;
        close();
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t sp = toSpeed(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "tcsetattr failed" << std::endl;
        close();
        return false;
    }
    return true;
}

void UartLink::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UartLink::send(VisionFrame& frame) {
    if (fd_ < 0) return false;

    frame.crc = crc16(reinterpret_cast<const uint8_t*>(&frame),
                      sizeof(frame) - sizeof(frame.crc), crc_mode_);

    const uint8_t* p = reinterpret_cast<const uint8_t*>(&frame);
    size_t left = sizeof(frame);
    while (left > 0) {
        ssize_t n = ::write(fd_, p, left);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        p += n;
        left -= (size_t)n;
    }
    return true;
}

bool UartLink::send(sem::SemanticCommand& cmd) {
    if (fd_ < 0) return false;

    cmd.crc = crc16(reinterpret_cast<const uint8_t*>(&cmd),
                    sizeof(cmd) - sizeof(cmd.crc), crc_mode_);

    const uint8_t* p = reinterpret_cast<const uint8_t*>(&cmd);
    size_t left = sizeof(cmd);
    while (left > 0) {
        ssize_t n = ::write(fd_, p, left);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        p += n;
        left -= (size_t)n;
    }
    return true;
}

bool UartLink::sendText(const std::string& text) {
    if (fd_ < 0) return false;

    const char* p = text.data();
    size_t left = text.size();
    while (left > 0) {
        ssize_t n = ::write(fd_, p, left);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        p += n;
        left -= (size_t)n;
    }
    return true;
}

uint16_t UartLink::crc16(const uint8_t* data, size_t len, CrcMode mode) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
        }
    }
    return mode == CrcMode::X25 ? static_cast<uint16_t>(crc ^ 0xFFFF) : crc;
}
