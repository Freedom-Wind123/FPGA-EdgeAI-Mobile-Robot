#pragma once
#include <string>
// POSIX shell single-argument quoting; recognition/API output is untrusted text.
inline std::string shellQuote(const std::string& value) {
    std::string out = "'";
    for (char c : value) {
        if (c == '\'') out += "'\\''";
        else if (c != '\0') out += c;
    }
    return out + "'";
}
