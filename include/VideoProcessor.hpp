#pragma once

#include <opencv2/opencv.hpp>
#include <csignal>
#include <unistd.h>
#include <time.h>
#include <yaml-cpp/yaml.h>

#include "RealTimeVideoCapture.hpp"
#include "Struct.hpp"
#include "Detector.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


extern std::atomic<bool> isRunning, cameraOpened;
extern GPIOHandler gpio;
extern YAML::Node config;

// VideoProcessor: 负责视频处理
class VideoProcessor {
public:
    VideoProcessor(ServoController& servoController, State& state) : servoController(servoController), state(state) {
        this->initialThreshold = config["initialThreshold"].as<int>();
        this->isvideo = config["video"]["isvideo"].as<bool>();
        this->videopath = config["video"]["videopath"].as<std::string>();
        this->playaudio = config["audio"]["playaudio"].as<bool>();
        this->audiopath = config["audio"]["audiopath"].as<std::string>();
        this->width = config["frame"]["width"].as<int>();
        this->height = config["frame"]["height"].as<int>();
        this->onnxmodelpath = config["linedetect"]["onnxmodelpath"].as<std::string>();
    };

    void videoProcessing();

private:
    ServoController& servoController;
    int initialThreshold;
    State& state;
    bool isvideo;
    std::string videopath;
    bool playaudio;
    std::string audiopath;
    std::string onnxmodelpath;
    int width;
    int height;
    Logger logger;
};
