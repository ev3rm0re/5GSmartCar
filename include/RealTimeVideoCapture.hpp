#pragma once

#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <mutex>

#include "Logger.hpp"

class RealTimeVideoCapture {
public:
    RealTimeVideoCapture() : stop(false) {}

    void open(const std::string& source) {
        cap.open(source, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            Logger::getLogger()->error("视频流打开失败");
            throw std::runtime_error("视频流打开失败");
        }

        readerThread = std::thread(&RealTimeVideoCapture::reader, this);
    }

    ~RealTimeVideoCapture() {
        stop = true;
        if (readerThread.joinable()) {
            readerThread.join();
        }
    }

    bool read(cv::Mat& frame) {
        std::unique_lock<std::mutex> lock(mtx);
        if (frames.empty()) return false;

        frame = frames.front();
        frames.pop();
        return true;
    }

private:
    cv::VideoCapture cap;
    std::queue<cv::Mat> frames;
    std::mutex mtx;
    std::thread readerThread;
    bool stop;

    void reader() {
        while (!stop) {
            cv::Mat frame;
            if (!cap.read(frame)) {
                stop = true;
                break;
            }

            std::unique_lock<std::mutex> lock(mtx);
            if (!frames.empty()) {
                frames.pop();                   // 丢弃上一帧
            }
            frames.push(frame);
            lock.unlock();
        }
        Logger::getLogger()->info("释放摄像头资源...");
        cap.release();
    }
};