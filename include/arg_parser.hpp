#pragma once
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <stdexcept>

class ArgParser {
public:
    ArgParser(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            args_.emplace_back(argv[i]);
        }
    }

    template <typename T>
    bool get(const std::string& name, T& value, const T& default_value, std::function<T(const std::string&)> convert) const {
        std::string prefix = "--" + name + "=";
        for (const auto& arg : args_) {
            if (arg.rfind(prefix, 0) == 0) {
                try {
                    value = convert(arg.substr(prefix.size()));
                    return true;
                } catch (...) {
                    // Conversion failed, fall through to set default
                    break;
                }
            }
        }
        value = default_value;
        return false;
    }

    // 常用类型的默认转换函数
    static int to_int(const std::string& s) { return std::stoi(s); }
    static double to_double(const std::string& s) { return std::stod(s); }
    static bool to_bool(const std::string& s) {
        if (s == "1" || s == "true" || s == "on") return true;
        if (s == "0" || s == "false" || s == "off") return false;
        throw std::invalid_argument("Invalid bool: " + s);
    }
    static std::string to_string(const std::string& s) { return s; }

private:
    std::vector<std::string> args_;
};
