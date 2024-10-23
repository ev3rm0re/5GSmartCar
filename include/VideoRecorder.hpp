#pragma once

#include <csignal>
#include <atomic>
#include <opencv2/opencv.hpp>

#include "Logger.hpp"


extern std::atomic<bool> isRunning;

// VideoRecorder: 负责视频录制
class VideoRecorder {
public:
    VideoRecorder(const std::string outputpath) : outputpath(outputpath) {};

    void record() {
        cv::VideoCapture videoCapture("/dev/cam0", cv::CAP_V4L2);
        if (!videoCapture.isOpened()) {
            Logger::getLogger()->error("VideoRecorder打开摄像头失败");
            return;
        }
        int width = videoCapture.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
        cv::VideoWriter videoWriter(outputpath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));
        cv::Mat frame;
        int frameCount = 0;
        try {
            Logger::getLogger()->info("开始录制视频");
            while (videoCapture.isOpened() && isRunning.load()) {
                videoCapture >> frame;
                videoWriter.write(frame);
                frameCount++;
                if (frameCount % 100 == 0) Logger::getLogger()->info("Frame count: " + std::to_string(frameCount));
            }                           
        } catch (cv::Exception& e) {
            Logger::getLogger()->error("VideoRecorder: " + std::string(e.what()));
        }
        videoWriter.release();
        Logger::getLogger()->info("录制视频结束");
    }

private:
    std::string outputpath;
};
