#include "VideoProcessor.hpp"

std::chrono::high_resolution_clock::time_point timeCount(std::chrono::high_resolution_clock::time_point t1, std::string message) {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - t1);
    Logger::getLogger()->debug(message + "用时: " + std::to_string(time_span.count() * 1000) + "ms");
    return now;
}

void VideoProcessor::videoProcessing() {
    Logger::getLogger()->info("开始视频处理...");
    /******************************打开摄像头或视频******************************/
    RealTimeVideoCapture recap;
    if (isvideo) {
        recap.open(videopath);
    }
    else {
        recap.open("/dev/cam0");
    }

    /******************************初始化检测器******************************/
    LineDetector lineDetector(width, height);               // 初始化边线检测器
    LaneDetector laneDetector(width, height);               // 初始化赛道检测器
    BinaryImageProcessor binaryProcessor(width, height);    // 初始化二值化处理器
    ArrowProcessor arrowProcessor(onnxmodelpath);           // 初始化箭头检测器
    CrosswalkDetector crosswalkDetector(width, height);     // 初始化人行横道检测器
    BlueBoardDetector blueboardDetector(width, height);     // 初始化蓝色挡板检测器
    ConeDetector coneDetector(width, height);               // 初始化锥桶检测器
    LetterOCR letterOCR;                                    // 初始化OCR
    
    int threshold = initialThreshold;                       // 初始化阈值
    int detectedCone = 0;                                   // 已检测到的锥桶数量
    bool detectedBlueBoard = false;                         // 是否检测过蓝色挡板

    /******************************视频处理循环******************************/
    while (isRunning.load()) {
        cv::Mat frame;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();  // 计时开始
        if (!recap.read(frame)) {                             // 读取视频帧，失败则跳过这一帧
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        cv::resize(frame, frame, cv::Size(width, height));
        
        /******************************检测蓝色挡板******************************/
        bool has_blueboard = blueboardDetector.hasBlueBoard(&frame);
        if (has_blueboard && !detectedBlueBoard) {
            state.has_blueboard.store(true);
            continue;
        }
        else {
            state.has_blueboard.store(false);
        }
        detectedBlueBoard = true;
        // std::chrono::high_resolution_clock::time_point t2 = timeCount(t1, "检测蓝色挡板");
        /******************************取ROI和二值化处理******************************/
        double roi_start = height / 2.0;
        cv::Rect ROI = cv::Rect(0, roi_start, width, height - roi_start);
        cv::Mat binary = binaryProcessor.getBinaryFrame(&frame, ROI, threshold);
        // Logger::getLogger()->showMat("binary", binary);

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
                servoController.changeLane(direct);
            }
        }
        // std::chrono::high_resolution_clock::time_point t3 = timeCount(t2, "检测斑马线");
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
        double laneCenter;
        // 绘制赛道
        if (lane.width > 0) {
            // cv::line(frame, lane.left_line.center + cv::Point2f(0, roi_start), lane.right_line.center + cv::Point2f(0, roi_start), cv::Scalar(0, 255, 0), 2);
            // cv::circle(frame, lane.center + cv::Point2f(0, roi_start), 5, cv::Scalar(0, 255, 0), -1);
            laneCenter = lane.center.x;             // 储存赛道中心
        }
        else {
            laneCenter = width / 2.0;
        }
        // std::chrono::high_resolution_clock::time_point t4 = timeCount(t3, "检测赛道");
        /******************************检测锥桶******************************/
        if (detectedCone < 3) {                     // 绕行锥桶三次后就不再检测, 提高运行速度
            double coneCenter;
            if (coneDetector.hasCone(&frame, &coneCenter)) {
                servoController.coneDetour(&detectedCone, coneCenter, lane);
            }
        }
        else {                                      // 锥桶之后进行蓝色区域检测，大于一定阈值进行OCR识别
            cv::Mat roi;
            double blueArea = letterOCR.blueAreaCount(frame, roi);
            if (blueArea > width * height / 60) {
                std::string text = letterOCR.recognize(roi);
                Logger::getLogger()->info("OCR识别结果: " + text);
                if (text.find('B') != std::string::npos) {
                    servoController.stopToArea('B', state);
                }
                else if (text.find('A') != std::string::npos) {
                    servoController.stopToArea('A', state);
                }
            }
        }
        servoController.setServoAngle(laneCenter);  // 舵机转向赛道中心
        // std::chrono::high_resolution_clock::time_point t5 = timeCount(t4, "检测锥桶");
        /******************************计算FPS******************************/
        std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();  // 计时结束
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t1);

        cv::putText(frame, "delay: " + std::to_string(time_span.count() * 1000).substr(0, 5) + "ms", cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

        /******************************视频显示******************************/
        Logger::getLogger()->showMat("frame", frame);
        if (!cameraOpened.load()) {
            cameraOpened.store(true);                                                                   // 摄像头已经打开, 用于通知电机启动
            Logger::getLogger()->info("摄像头已打开, 第一帧获取成功, 通知电机启动");
        }
    }
    Logger::getLogger()->info("关闭所有窗口...");
    cv::destroyAllWindows();
};