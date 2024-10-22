#pragma once

#include <opencv2/opencv.hpp>

#include "Logger.hpp"

class VideoRecorder {
public:
    VideoRecorder(const std::string outputpath, int width, int height) : width(width), height(height) {
        // Initialize video writer
        cv::VideoWriter videoWriter(outputpath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));
    };

    void record() {
        cv::VideoCapture videoCapture(0);
        cv::Mat frame;
        int frameCount = 0;
        while (videoCapture.isOpened()) {
            videoCapture >> frame;
            videoWriter.write(frame);
            frameCount++;
            if (frameCount % 100 == 0) Logger::getLogger().info("Frame count: " + std::to_string(frameCount));
        }
    }

    ~VideoRecorder() {
        videoWriter.release();
    }

private:
    cv::VideoWriter videoWriter;
    int width;
    int height;
};