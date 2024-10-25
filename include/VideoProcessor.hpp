#pragma once

#include <opencv2/opencv.hpp>
#include <csignal>
#include <unistd.h>
#include <time.h>

#include "Struct.hpp"
#include "Detector.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


extern std::atomic<bool> isRunning, cameraOpened, detectedCone;
extern std::atomic<int> direction, mode;
extern std::atomic<double> lane_center;
extern GPIOHandler gpio;

// VideoProcessor: 负责视频处理
class VideoProcessor {
public:
    VideoProcessor(int initialThreshold, bool isVideo, std::string videopath, bool playaudio, std::string audiopath, int width, int height, 
                    std::string onnxmodelpath, State& state, int servo_pin) : 
                    initialThreshold(initialThreshold), isVideo(isVideo), videopath(videopath), playaudio(playaudio), audiopath(audiopath), 
                    width(width), height(height), onnxmodelpath(onnxmodelpath), state(state), servo_pin(servo_pin) {};

    void videoProcessing() {
        Logger::getLogger()->info("开始视频处理...");
        cv::VideoCapture cap;
        if (isVideo) {
            std::string video_path = videopath;
            cap.open(video_path);
        }
        else {
            cap.open("/dev/cam0", cv::CAP_V4L2);
        }

        if (!cap.isOpened()) {
            Logger::getLogger()->error("视频流打开失败");
            return;
        }

        // 初始化边线检测器
        LineDetector lineDetector(width, height);
        // 初始化赛道检测器
        LaneDetector laneDetector(width, height);
        // 初始化二值图像处理器
        BinaryImageProcessor binaryProcessor(width, height);
        // 初始化箭头检测器
        ArrowProcessor arrowProcessor(onnxmodelpath);
        // 初始化人行横道检测器
        CrosswalkDetector crosswalkDetector(width, height);
        // 初始化蓝色挡板检测器
        BlueBoardDetector blueboardDetector(width, height);
        // 初始化锥桶检测器
        ConeDetector coneDetector(width, height);
        // 初始化阈值
        int threshold = initialThreshold;

        while (cap.isOpened() && isRunning.load()) {
            cv::Mat frame;
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            cap >> frame;
            if (frame.empty()) break;
            cv::resize(frame, frame, cv::Size(width, height));

            // 检测蓝色挡板
            bool has_blueboard = blueboardDetector.hasBlueBoard(&frame);
            if (has_blueboard) {
                state.has_blueboard.store(true);
                continue;
            }
            else {
                state.has_blueboard.store(false);
            }
            // 取ROI并二值化
            double roi_start = height / 3.0;
            cv::Rect ROI = cv::Rect(0, roi_start, width, height - roi_start);
            cv::Mat binary = binaryProcessor.getBinaryFrame(&frame, ROI, threshold);
            Logger::getLogger()->showMat("binary", binary);

            // 检测斑马线, 只检测一次
            if (!state.has_crosswalk.load()) {
                if (crosswalkDetector.hasCrosswalk(&binary)) {
                    state.has_crosswalk.store(true);
                    // 检测箭头
                    int direct = arrowProcessor.detectArrowDirection(&frame);
                    Logger::getLogger()->info("检测到箭头，方向为: " + directions.at(direct));
                    if (playaudio == true) {
                        system(("aplay " + audiopath).data());
                    }
                    direction.store(direct);
                    mode.store(1);
                }
            }

            // TODO: 检测锥桶
            if (!detectedCone.load()) {
                if (coneDetector.hasCone(&frame)) {
                    Logger::getLogger()->info("检测到锥桶");
                    detectedCone.store(true);
                    mode.store(2);
                }
            }

            // 检测边线
            std::vector<Line> lines = lineDetector.detectLines(&binary);
            lineDetector.filterLines(&lines);

            // 自适应调整阈值
            int white_count = cv::countNonZero(binary);
            binaryProcessor.adjustThreshold(white_count, &threshold, lines.size());

            // 绘制边线
            for (const auto& line : lines) {
                cv::line(frame, line.top + cv::Point2f(0, roi_start), line.bottom + cv::Point2f(0, roi_start), cv::Scalar(255, 0, 0), 2);
            }

            // 匹配赛道
            Lane lane;
            laneDetector.getLane(lines, &lane);

            // 绘制赛道
            if (lane.width > 0) {
                cv::line(frame, lane.left_line.center + cv::Point2f(0, roi_start), lane.left_line.center + cv::Point2f(0, roi_start), cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, lane.center + cv::Point2f(0, roi_start), 5, cv::Scalar(0, 255, 0), -1);
                // 舵机控制
                lane_center.store(lane.center.x);
            }
            else {
                lane_center.store(width / 2.0);
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            double fps = 1.0 / time_span.count();
            cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
            // 显示图像
            Logger::getLogger()->showMat("frame", frame);
            if (!cameraOpened.load()) {
                cameraOpened.store(true); // 摄像头已经打开, 用于通知电机启动
                Logger::getLogger()->info("摄像头已打开, 第一帧获取成功, 通知电机启动");
            }
        }
        Logger::getLogger()->info("释放摄像头资源...");
        cap.release();
        Logger::getLogger()->info("关闭所有窗口...");
        cv::destroyAllWindows();
    };

private:
    int initialThreshold;
    State& state;
    bool isVideo;
    std::string videopath;
    bool playaudio;
    std::string audiopath;
    std::string onnxmodelpath;
    int servo_pin;
    int width;
    int height;
    Logger logger;
};
