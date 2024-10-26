#pragma once

#include <opencv2/opencv.hpp>
#include <csignal>
#include <unistd.h>
#include <time.h>

#include "Struct.hpp"
#include "Detector.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


extern std::atomic<bool> isRunning, cameraOpened;
extern std::atomic<int> direction, mode, detectedCone;
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
        /******************************打开摄像头或视频******************************/
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
        Logger::getLogger()->info("视频流打开成功");

        /******************************初始化检测器******************************/
        LineDetector lineDetector(width, height);               // 初始化边线检测器
        LaneDetector laneDetector(width, height);               // 初始化赛道检测器
        BinaryImageProcessor binaryProcessor(width, height);    // 初始化二值化处理器
        ArrowProcessor arrowProcessor(onnxmodelpath);           // 初始化箭头检测器
        CrosswalkDetector crosswalkDetector(width, height);     // 初始化人行横道检测器
        BlueBoardDetector blueboardDetector(width, height);     // 初始化蓝色挡板检测器
        ConeDetector coneDetector(width, height);               // 初始化锥桶检测器
        
        int threshold = initialThreshold;                       // 初始化阈值

        /******************************视频处理循环******************************/
        while (cap.isOpened() && isRunning.load()) {
            cv::Mat frame;
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();  // 计时开始
            cap >> frame;
            if (frame.empty()) break;
            cv::resize(frame, frame, cv::Size(width, height));

            /******************************检测蓝色挡板******************************/
            bool has_blueboard = blueboardDetector.hasBlueBoard(&frame);
            if (has_blueboard) {
                state.has_blueboard.store(true);
                continue;
            }
            else {
                state.has_blueboard.store(false);
            }
            
            /******************************取ROI和二值化处理******************************/
            double roi_start = height / 3.0;
            cv::Rect ROI = cv::Rect(0, roi_start, width, height - roi_start);
            cv::Mat binary = binaryProcessor.getBinaryFrame(&frame, ROI, threshold);
            Logger::getLogger()->showMat("binary", binary);

            /******************************检测斑马线, 一次******************************/
            if (!state.has_crosswalk.load()) {
                if (crosswalkDetector.hasCrosswalk(&binary)) { // 检测到斑马线
                    state.has_crosswalk.store(true);
                    Logger::getLogger()->info("检测到斑马线");
                    // 检测箭头
                    int direct = arrowProcessor.detectArrowDirection(&frame);
                    Logger::getLogger()->info("检测到箭头，方向为: " + directions.at(direct));
                    if (playaudio == true) {
                        system(("aplay " + audiopath).data());
                    }
                    direction.store(direct);                // 设置方向
                    mode.store(1);                          // 设置变道模式
                }
            }

            /******************************检测锥桶******************************/
            if (detectedCone.load() < 4) {                  // 绕行锥桶三次后就不再检测, 提高运行速度
                if (coneDetector.hasCone(&frame)) {
                    mode.store(2);                          // 设置绕行模式
                }
            }

            /******************************检测边线******************************/
            std::vector<Line> lines = lineDetector.detectLines(&binary);
            lineDetector.filterLines(&lines);
            // 绘制边线
            for (const auto& line : lines) {
                cv::line(frame, line.top + cv::Point2f(0, roi_start), line.bottom + cv::Point2f(0, roi_start), cv::Scalar(255, 0, 0), 2);
            }

            /******************************调整阈值******************************/
            int white_count = cv::countNonZero(binary);
            binaryProcessor.adjustThreshold(white_count, &threshold, lines.size());

            /******************************检测赛道******************************/
            Lane lane;
            laneDetector.getLane(lines, &lane);
            // 绘制赛道
            if (lane.width > 0) {
                cv::line(frame, lane.left_line.center + cv::Point2f(0, roi_start), lane.left_line.center + cv::Point2f(0, roi_start), cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, lane.center + cv::Point2f(0, roi_start), 5, cv::Scalar(0, 255, 0), -1);
                lane_center.store(lane.center.x);           // 储存赛道中心
            }
            else {
                lane_center.store(width / 2.0);
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();  // 计时结束
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            double fps = 1.0 / time_span.count();
            cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

            /******************************视频显示******************************/
            Logger::getLogger()->showMat("frame", frame);
            if (!cameraOpened.load()) {
                cameraOpened.store(true);                                                                   // 摄像头已经打开, 用于通知电机启动
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
