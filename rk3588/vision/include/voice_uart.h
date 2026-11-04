#pragma once

#include <cstdint>
#include <string>

// 离线语音模块串口文本读取器
// 模块识别到语音后通过 UART 发送一行中文文本（通常以 \r\n 结尾），
// 本类负责非阻塞读取整行，并做 GBK→UTF8 转换后供语义层使用。
class VoiceUart {
public:
    enum class Encoding {
        UTF8,
        GBK,
    };

    VoiceUart();
    ~VoiceUart();

    bool open(const std::string& device, int baud = 115200,
              Encoding enc = Encoding::GBK);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 非阻塞读取一行完整文本（遇到 \n 或 \r\n 为止）。
    // 读到一行返回 true 并写入 line（已去除换行符、已转 UTF-8）；
    // 无完整行返回 false。
    bool readLine(std::string& line);

    // 发送一行 UTF-8 文本（按模块编码转 GBK，结尾补 \r\n），
    // 供文本播报使用；需模块固件支持串口文本播报
    bool sendText(const std::string& utf8);

private:
    static std::string gbkToUtf8(const std::string& gbk);
    static std::string utf8ToGbk(const std::string& utf8);

    int fd_ = -1;
    Encoding enc_ = Encoding::GBK;
    std::string buffer_;
};
