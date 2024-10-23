#pragma once

#include <opencv2/opencv.hpp>

#include "Logger.hpp"


// VideoRecorder: 负责视频录制
class VideoRecorder {
public:
    VideoRecorder(const std::string outputpath) : outputpath(outputpath) {};

    void record() {
        cv::VideoCapture videoCapture("/dev/cam0", cv::CAP_V4L2);
        int width = videoCapture.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
        cv::VideoWriter videoWriter(outputpath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));
        cv::Mat frame;
        int frameCount = 0;
        try {
            while (videoCapture.isOpened()) {
                videoCapture >> frame;
                videoWriter.write(frame);
                frameCount++;
                if (frameCount % 100 == 0) Logger::getLogger()->info("Frame count: " + std::to_string(frameCount));
            }                           
        } catch (cv::Exception& e) {
            Logger::getLogger()->error("VideoRecorder: " + std::string(e.what()));
        }
        videoWriter.release();
    }

private:
    std::string outputpath;
};