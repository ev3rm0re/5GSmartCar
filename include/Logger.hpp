#pragma once

#include <iostream>
#include <chrono>
#include <ctime>
#include <opencv2/opencv.hpp>


// Logger: 负责日志输出
class Logger {
public:
    // 日志级别枚举
    enum LogLevel {
        DEBUG = 1,
        INFO = 2,
        WARNING = 3,
        ERROR = 4
    };

    // 获取Logger单例
    static Logger* getLogger() {
        static Logger instance;
        return &instance;
    }

    // 设置日志级别
    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

    // 信息日志（绿色字体）
    void info(const std::string& message) {
        if (currentLogLevel <= INFO) {
            log("\033[32m", "[INFO]", message);  // 绿色
        }
    }

    // 调试日志（蓝色字体）
    void debug(const std::string& message) {
        if (currentLogLevel <= DEBUG) {
            log("\033[34m", "[DEBUG]", message);  // 蓝色
        }
    }

    // 错误日志（红色字体）
    void error(const std::string& message) {
        if (currentLogLevel <= ERROR) {
            log("\033[31m", "[ERROR]", message);  // 红色
        }
    }

    // 警告日志（黄色字体）
    void warning(const std::string& message) {
        if (currentLogLevel <= WARNING) {
            log("\033[33m", "[WARNING]", message);  // 黄色
        }
    }

    // 显示Mat图像
    void showMat(const std::string& windowName, cv::Mat& frame) {
        if (currentLogLevel >= DEBUG) {
            cv::imshow(windowName, frame);
            cv::waitKey(1);
        }
    }

private:
    LogLevel currentLogLevel = DEBUG;  // 默认日志级别是DEBUG

    // 打印日志内容
    void log(const std::string& colorCode, const std::string& level, const std::string& message) {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
        std::cout << colorCode << oss.str() << " " << level << " " << message << "\033[0m" << std::endl;
    }
};
