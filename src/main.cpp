#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <time.h>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "VideoProcessor.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


int main() {
    system("sudo killall pigpiod");
  	system("sudo cp /home/pi/.Xauthority /root/");
  	sleep(2);

    std::cout << "****************程序开始运行****************" << std::endl;
    // 判断是否有配置文件
    if (access("/home/pi/Code/5GSmartCar/config/configs.yaml", F_OK) == -1) {
        Logger::getLogger()->error("配置文件不存在");
        return -1;
    }
    YAML::Node config = YAML::LoadFile("/home/pi/Code/5GSmartCar/config/configs.yaml");
    // LOG_LEVEL
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
    // gpio参数
    int servo_pin = config["gpio"]["servo_pin"].as<int>();
    int pwm_pin = config["gpio"]["pwm_pin"].as<int>();
    int init_pwm = config["gpio"]["init_pwm"].as<int>();
    int target_pwm = config["gpio"]["target_pwm"].as<int>();
    // 视频参数
    int width = config["frame"]["width"].as<int>();
    int height = config["frame"]["height"].as<int>();
    // 音频路径
    bool isaudio = config["audio"]["isaudio"].as<bool>();
    std::string audiopath = config["audio"]["audiopath"].as<std::string>();
    if (isaudio) {
        system(("aplay " + audiopath).data());
    }
    // 检测参数
    std::string onnxmodelpath = config["linedetect"]["onnxmodelpath"].as<std::string>();
    // 获取 video 部分的参数
    bool isvideo = config["video"]["isvideo"].as<bool>();
    std::string videopath = config["video"]["videopath"].as<std::string>();
    // 移动控制参数
    bool movecontrol = config["movecontrol"].as<bool>();

    // 初始化 gpio
    GPIOHandler gpio(servo_pin, pwm_pin);

    // 初始化状态
    State state;
    state.has_crosswalk.store(false);
    state.has_blueboard.store(false);

    // 初始化 videoProcessor
    VideoProcessor videoProcessor(isvideo, videopath, audiopath, width, height, onnxmodelpath, init_pwm, target_pwm, state, std::ref(gpio));
    // 初始化电机控制器
    MotorController motor(std::ref(gpio), pwm_pin, init_pwm, target_pwm);

    std::thread videoThread(&VideoProcessor::videoProcessing, &videoProcessor);
    std::thread moveThread;
    if (movecontrol) {
            moveThread = std::thread(&MotorController::moveForward, &motor, std::ref(state));
    }
    try {
        videoThread.join();
        if (movecontrol) moveThread.join();
    } catch (const std::exception& e) {
        Logger::getLogger()->error(e.what());
    }
    return 0;
}