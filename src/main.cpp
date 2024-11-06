#include <opencv2/opencv.hpp>
#include <memory>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <time.h>
#include <chrono>
#include <cassert>
#include <yaml-cpp/yaml.h>

#include "VideoProcessor.hpp"
#include "VideoRecorder.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


std::atomic<bool> isRunning(true), cameraOpened(false);

static std::unique_ptr<GPIOHandler> gpioHandlerPtr; // 全局静态智能指针，用于管理 GPIOHandler 实例

void signalHandler(int signum) { // 信号处理函数
    isRunning.store(false);
    Logger::getLogger()->info("接收到信号: " + std::to_string(signum) + "，程序即将退出...");
}

YAML::Node config = YAML::LoadFile("/home/pi/Code/5GSmartCar/config/configs.yaml");

int main() {
    assert(config.IsDefined());
    std::cout << "****************程序开始运行****************" << std::endl;

    /******************************系统设置******************************/
    system("sudo cp /home/pi/.Xauthority /root"); // 用于解决无法显示图像的问题

    /******************************GPIO初始化******************************/
    gpioHandlerPtr = std::make_unique<GPIOHandler>();

    /******************************信号处理函数******************************/
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

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

    // 初始化 motorController
    MotorController motorController(gpioHandlerPtr.get());
    // 初始化舵机控制器
    ServoController servoController(gpioHandlerPtr.get());
    // 初始化 videoProcessor
    VideoProcessor videoProcessor(std::ref(servoController), std::ref(state));

    std::thread videoThread(&VideoProcessor::videoProcessing, &videoProcessor);                                     // 检测线程
    std::thread motorThread;
    if (movecontrol) motorThread = std::thread(&MotorController::moveForward, &motorController, std::ref(state));   // 电机控制线程                                  // 舵机控制线程

    while (isRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待线程结束
    videoThread.join();
    if (movecontrol) motorThread.join();

    /******************************释放资源******************************/
    motorController.stop();
    servoController.reset();

    gpioHandlerPtr->cleanup();
    gpioHandlerPtr.reset();

    /******************************结束******************************/
    std::cout << "****************程序结束运行****************" << std::endl;
    return 0;
}