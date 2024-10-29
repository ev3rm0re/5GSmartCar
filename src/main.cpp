#include <opencv2/opencv.hpp>
#include <memory>
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

static std::unique_ptr<GPIOHandler> gpioHandlerPtr; // 全局静态智能指针，用于管理 GPIOHandler 实例

void signalHandler(int signum) { // 信号处理函数
    isRunning = false;
    Logger::getLogger()->info("接收到信号: " + std::to_string(signum) + "，程序即将退出...");
}

// GPIOHandler gpio; // GPIOHandler 全局实例(必须放到主函数外面，不然不知道为什么会影响ctrl+c退出信号的获取)

int main() {
    std::cout << "****************程序开始运行****************" << std::endl;
    /******************************系统设置******************************/
    system("sudo cp /home/pi/.Xauthority /root"); // 用于解决无法显示图像的问题

    /******************************GPIO初始化******************************/
    gpioHandlerPtr = std::make_unique<GPIOHandler>();

    /******************************信号处理函数******************************/
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

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

    // 初始化 motorController
    MotorController motorController(gpioHandlerPtr.get(), motor_pin, init_pwm, target_pwm);
    // 初始化舵机控制器
    ServoController servoController(gpioHandlerPtr.get(), servo_pin, width);
    // 初始化 videoProcessor
    VideoProcessor videoProcessor(std::ref(servoController), initialThreshold, isvideo, videopath, playaudio, audiopath, width, height, onnxmodelpath, state, servo_pin);

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