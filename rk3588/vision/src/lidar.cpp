#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "lidar.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace {

constexpr int kBinCount = 720;                       // 0.5 度一格
constexpr float kBinDeg = 360.0f / kBinCount;
constexpr float kMinDistM = 0.05f;                   // C1 量程 0.05~12m
constexpr float kMaxDistM = 12.0f;
constexpr int64_t kScanHoldMs = 1000;                // 超过 1s 没新扫描视为失效

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

speed_t toSpeed(int baud) {
    switch (baud) {
        case 115200: return B115200;
#ifdef B256000
        case 256000: return B256000;
#endif
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            std::cerr << "unsupported lidar baud " << baud
                      << ", fallback to 460800" << std::endl;
            return B460800;
    }
}

}  // namespace

Lidar::~Lidar() {
    stop();
}

bool Lidar::open(const std::string& device, int baud) {
    stop();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "open lidar failed: " << device
                  << " (" << std::strerror(errno) << ")" << std::endl;
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "lidar tcgetattr failed" << std::endl;
        ::close(fd_);
        fd_ = -1;
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
    tio.c_cc[VTIME] = 1;   // 100ms 读超时，便于线程退出

    speed_t sp = toSpeed(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "lidar tcsetattr failed" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // CP210x 打开串口会触发设备复位，等它启动完再发命令，否则第一条会丢
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tcflush(fd_, TCIFLUSH);

    if (!sendScan()) {
        std::cerr << "lidar scan command failed" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cout << "lidar opened: " << device << " @" << baud << std::endl;
    return true;
}

bool Lidar::sendScan() {
    if (fd_ < 0) return false;
    // 标准 SCAN 请求：A5 20；电机随雷达上电自动转动
    uint8_t cmd[2] = {0xA5, 0x20};
    return ::write(fd_, cmd, sizeof(cmd)) == (ssize_t)sizeof(cmd);
}

void Lidar::start() {
    if (fd_ < 0 || running_) return;
    running_ = true;
    thread_ = std::thread(&Lidar::readLoop, this);
}

void Lidar::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void Lidar::publishScan(const std::vector<float>& bins) {
    std::lock_guard<std::mutex> lk(mtx_);
    bins_ = bins;
    last_scan_ms_ = nowMs();
    ++scan_count_;
}

void Lidar::readLoop() {
    // 协议状态机：0 找 A5 / 1 等 5A / 2 读响应描述符 / 3 读节点
    int state = 0;
    size_t payload = 0;
    float prev_angle = -1.0f;

    std::vector<uint8_t> buf;
    buf.reserve(8192);
    std::vector<float> cur(kBinCount, 0.0f);
    uint8_t chunk[512];
    int64_t last_rx_ms = nowMs();

    while (running_) {
        ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "lidar read failed: " << std::strerror(errno) << std::endl;
            break;
        }
        if (n == 0) {
            // 1s 没有数据就重发 SCAN，兼容命令在设备启动期丢失的情况
            if (nowMs() - last_rx_ms > 1000) {
                sendScan();
                last_rx_ms = nowMs();
            }
            continue;
        }
        last_rx_ms = nowMs();
        buf.insert(buf.end(), chunk, chunk + (size_t)n);

        size_t i = 0;
        while (i < buf.size()) {
            if (state == 0) {
                while (i < buf.size() && buf[i] != 0xA5) ++i;
                if (i >= buf.size()) break;
                state = 1;
                ++i;
                continue;
            }

            if (state == 1) {
                if (buf[i] == 0x5A) {
                    state = 2;
                    ++i;
                } else if (buf[i] == 0xA5) {
                    ++i;   // 连续两个 A5，当前这个仍可能是帧头
                } else {
                    state = 0;
                    ++i;
                }
                continue;
            }

            if (state == 2) {
                // A5 5A 之后是 4 字节长度/模式 + 1 字节数据类型
                if (i + 5 > buf.size()) break;
                uint32_t size = (uint32_t)buf[i] |
                                ((uint32_t)buf[i + 1] << 8) |
                                ((uint32_t)buf[i + 2] << 16) |
                                ((uint32_t)buf[i + 3] << 24);
                payload = size & 0x3FFFFFFFu;
                i += 5;
                state = payload > 0 ? 3 : 0;
                continue;
            }

            // state == 3：连续 5 字节扫描节点
            if (payload != 5) {   // 只支持标准 SCAN 模式
                state = 0;
                continue;
            }
            if (i + 5 > buf.size()) break;

            const uint8_t* p = &buf[i];
            const int quality = p[0] >> 2;
            const float angle = (((p[1] >> 1) | (p[2] << 7))) / 64.0f;
            const float dist_m = (float)(p[3] | (p[4] << 8)) / 4.0f / 1000.0f;

            if (quality > 0 && dist_m >= kMinDistM && dist_m <= kMaxDistM) {
                int idx = (int)(angle / kBinDeg);
                if (idx >= 0 && idx < kBinCount) cur[idx] = dist_m;
            }

            // 角度回绕代表新的一圈开始，发布上一圈
            if (prev_angle >= 0.0f && angle < prev_angle - 180.0f) {
                publishScan(cur);
                std::fill(cur.begin(), cur.end(), 0.0f);
            }
            prev_angle = angle;

            i += 5;
        }

        if (i > 0) buf.erase(buf.begin(), buf.begin() + (long)i);
    }
}

bool Lidar::distanceInWindow(float center_deg, float half_span_deg,
                             float& dist_m) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (bins_.empty()) return false;
    if (nowMs() - last_scan_ms_ > kScanHoldMs) return false;

    float c = std::fmod(center_deg, 360.0f);
    if (c < 0.0f) c += 360.0f;

    const float span = std::clamp(half_span_deg, 0.5f, 90.0f);
    const int n = (int)std::ceil(span / kBinDeg);
    const int center_bin = (int)std::lround(c / kBinDeg);

    float best = 0.0f;
    for (int k = -n; k <= n; ++k) {
        int idx = (center_bin + k) % kBinCount;
        if (idx < 0) idx += kBinCount;
        const float v = bins_[idx];
        if (v > 0.0f && (best <= 0.0f || v < best)) best = v;
    }

    if (best <= 0.0f) return false;
    dist_m = best;
    return true;
}
