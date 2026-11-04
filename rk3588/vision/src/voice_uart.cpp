#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "voice_uart.h"

#include <fcntl.h>
#include <iconv.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace {

speed_t toSpeed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 460800: return B460800;
#ifdef B921600
        case 921600: return B921600;
#endif
        default:
            std::cerr << "[voice] unsupported baud " << baud
                      << ", fallback to 115200" << std::endl;
            return B115200;
    }
}

}  // namespace

VoiceUart::VoiceUart() = default;

VoiceUart::~VoiceUart() { close(); }

bool VoiceUart::open(const std::string& device, int baud, Encoding enc) {
    close();
    enc_ = enc;

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[voice] open failed: " << device
                  << " (" << std::strerror(errno) << ")" << std::endl;
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "[voice] tcgetattr failed" << std::endl;
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
        std::cerr << "[voice] tcsetattr failed" << std::endl;
        close();
        return false;
    }

    tcflush(fd_, TCIFLUSH);
    return true;
}

void VoiceUart::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool VoiceUart::readLine(std::string& line) {
    if (fd_ < 0) return false;

    char buf[256];
    ssize_t n;
    while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
        buffer_.append(buf, static_cast<size_t>(n));
    }

    auto pos = buffer_.find('\n');
    if (pos == std::string::npos) return false;

    std::string raw = buffer_.substr(0, pos);
    buffer_.erase(0, pos + 1);

    if (!raw.empty() && raw.back() == '\r') raw.pop_back();

    size_t s = raw.find_first_not_of(" \t\r\n");
    if (s == std::string::npos) return false;
    size_t e = raw.find_last_not_of(" \t\r\n");
    raw = raw.substr(s, e - s + 1);

    if (raw.empty()) return false;

    line = (enc_ == Encoding::GBK) ? gbkToUtf8(raw) : raw;
    return true;
}

bool VoiceUart::sendText(const std::string& utf8) {
    if (fd_ < 0) return false;

    std::string out = (enc_ == Encoding::GBK) ? utf8ToGbk(utf8) : utf8;
    out += "\r\n";

    size_t off = 0;
    while (off < out.size()) {
        ssize_t n = ::write(fd_, out.data() + off, out.size() - off);
        if (n > 0) {
            off += (size_t)n;
        } else if (n < 0 && errno == EAGAIN) {
            usleep(2000);  // 发送缓冲满，稍等重试
        } else {
            std::cerr << "[voice] write failed: " << std::strerror(errno)
                      << std::endl;
            return false;
        }
    }
    return true;
}

// UTF-8→GBK，与 gbkToUtf8 互为逆过程
std::string VoiceUart::utf8ToGbk(const std::string& utf8) {
    iconv_t cd = iconv_open("GBK", "UTF-8");
    if (cd == reinterpret_cast<iconv_t>(-1)) {
        std::cerr << "[voice] iconv_open UTF8->GBK failed: "
                  << std::strerror(errno) << std::endl;
        return utf8;
    }

    std::string out;
    out.resize(utf8.size() + 8);  // GBK 不超过 2 字节/汉字

    char* inbuf = const_cast<char*>(utf8.data());
    size_t inleft = utf8.size();
    char* outbuf = out.data();
    size_t outleft = out.size();

    size_t r = iconv(cd, &inbuf, &inleft, &outbuf, &outleft);
    iconv_close(cd);

    if (r == static_cast<size_t>(-1)) {
        std::cerr << "[voice] UTF8->GBK convert failed at byte "
                  << (utf8.size() - inleft) << ": "
                  << std::strerror(errno) << std::endl;
    }

    out.resize(out.size() - outleft);
    return out;
}

// 使用系统 iconv 做 GBK→UTF-8 转换，覆盖全部 GBK 字符
std::string VoiceUart::gbkToUtf8(const std::string& gbk) {
    iconv_t cd = iconv_open("UTF-8", "GBK");
    if (cd == reinterpret_cast<iconv_t>(-1)) {
        std::cerr << "[voice] iconv_open GBK->UTF8 failed: "
                  << std::strerror(errno) << std::endl;
        return gbk;  // 转换失败时原样返回，避免崩溃
    }

    std::string out;
    out.resize(gbk.size() * 3);  // UTF-8 最多 3 字节/汉字

    char* inbuf = const_cast<char*>(gbk.data());
    size_t inleft = gbk.size();
    char* outbuf = out.data();
    size_t outleft = out.size();

    size_t r = iconv(cd, &inbuf, &inleft, &outbuf, &outleft);
    iconv_close(cd);

    if (r == static_cast<size_t>(-1)) {
        std::cerr << "[voice] iconv convert failed at byte "
                  << (gbk.size() - inleft) << ": "
                  << std::strerror(errno) << std::endl;
    }

    out.resize(out.size() - outleft);
    return out;
}
