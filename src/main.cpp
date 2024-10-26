#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <time.h>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "VideoProcessor.hpp"
#include "VideoRecorder.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


std::atomic<bool> isRunning(true), cameraOpened(false);
std::atomic<int> direction, mode(0), detectedCone(0);
std::atomic<double> lane_center;

void signalHandler(int signum) { // 信号处理函数
    isRunning = false;
    Logger::getLogger()->info("接收到信号: " + std::to_string(signum) + "，程序即将退出...");
}

GPIOHandler gpio; // GPIOHandler 全局实例(必须放到主函数外面，不然不知道为什么会影响ctrl+c退出信号的获取)

int main() {
    /******************************系统设置******************************/
    system("sudo cp /home/pi/.Xauthority /root"); // 用于解决无法显示图像的问题

    /******************************信号处理函数******************************/
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signalHandler;
    sigaction(SIGINT, &sigIntHandler, nullptr);
    sigaction(SIGABRT, &sigIntHandler, nullptr);
    sigaction(SIGTERM, &sigIntHandler, nullptr);
    sigaction(SIGCONT, &sigIntHandler, nullptr);

    std::cout << "****************程序开始运行****************" << std::endl;

    /******************************加载配置文件******************************/
    if (access("/home/pi/Code/5GSmartCar/config/configs.yaml", F_OK) == -1) {
        Logger::getLogger()->error("配置文件不存在");
        return -1;
    }
    YAML::Node config = YAML::LoadFile("/home/pi/Code/5GSmartCar/config/configs.yaml");

    /******************************设置LOGLEVEL******************************/
    std::string loglevel = config["loglevel"].as<std::string>();
    if (loglevel == "INFO") {
        Logger::getLogger()->setLogLevel(Logger::INFO);
    } else if (loglevel == "WARNING") {
        Logger::getLogger()->setLogLevel(Logger::WARNING);
    } else if (loglevel == "ERROR") {
        Logger::getLogger()->setLogLevel(Logger::ERROR);
    } else {
        Logger::getLogger()->setLogLevel(Logger::DEBUG);
    }

    /******************************参数获取******************************/
    // gpio
    int servo_pin = config["gpio"]["servo_pin"].as<int>();
    int motor_pin = config["gpio"]["motor_pin"].as<int>();
    int init_pwm = config["gpio"]["init_pwm"].as<int>();
    int target_pwm = config["gpio"]["target_pwm"].as<int>();
    // 摄像头
    int initialThreshold = config["initialThreshold"].as<int>();
    int width = config["frame"]["width"].as<int>();
    int height = config["frame"]["height"].as<int>();
    // 音频
    bool playaudio = config["audio"]["playaudio"].as<bool>();
    std::string audiopath = config["audio"]["audiopath"].as<std::string>();
    // 检测
    std::string onnxmodelpath = config["linedetect"]["onnxmodelpath"].as<std::string>();
    // 视频
    bool isvideo = config["video"]["isvideo"].as<bool>();
    std::string videopath = config["video"]["videopath"].as<std::string>();
    // 移动控制
    bool movecontrol = config["movecontrol"].as<bool>();
    // 是否调用录像
    bool recordvideo = config["recordvideo"].as<bool>();

    /******************************录像******************************/
    if (recordvideo) {
        VideoRecorder videoRecorder("/home/pi/Code/5GSmartCar/medias/output.avi");
        videoRecorder.record();
        return 0;
    }

    /******************************控制******************************/
    // 初始化状态
    State state;
    state.has_crosswalk.store(false);
    state.has_blueboard.store(false);
    lane_center.store(width / 2.0);

    // 初始化 videoProcessor
    VideoProcessor videoProcessor(initialThreshold, isvideo, videopath, playaudio, audiopath, width, height, onnxmodelpath, state, servo_pin);
    // 初始化 motorController
    MotorController motorController(gpio, motor_pin, init_pwm, target_pwm);
    // 初始化 servoController
    ServoController servoController(gpio, servo_pin, width);

    // 初始化检测线程和移动线程
    std::thread videoThread(&VideoProcessor::videoProcessing, &videoProcessor);                                     // 检测线程
    std::thread motorThread;
    if (movecontrol) motorThread = std::thread(&MotorController::moveForward, &motorController, std::ref(state));   // 电机控制线程
    std::thread servoThread(&ServoController::setAngle, &servoController, width);                                   // 舵机控制线程

    // 等待线程结束
    videoThread.join();
    if (movecontrol) motorThread.join();
    servoThread.join();

    /******************************结束******************************/
    std::cout << "****************主线程结束****************" << std::endl;
    return 0;
}