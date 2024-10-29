#pragma once

#include <opencv2/opencv.hpp>
#include <csignal>
#include <unistd.h>
#include <time.h>

#include "RealTimeVideoCapture.hpp"
#include "Struct.hpp"
#include "Detector.hpp"
#include "Controller.hpp"
#include "Logger.hpp"


extern std::atomic<bool> isRunning, cameraOpened;
extern GPIOHandler gpio;

// VideoProcessor: 负责视频处理
class VideoProcessor {
public:
    VideoProcessor(ServoController& servoController, int initialThreshold, bool isVideo, std::string videopath, bool playaudio, std::string audiopath, int width, int height, 
                    std::string onnxmodelpath, State& state, int servo_pin) : 
                    servoController(servoController), initialThreshold(initialThreshold), isVideo(isVideo), videopath(videopath), playaudio(playaudio), 
                    audiopath(audiopath), width(width), height(height), onnxmodelpath(onnxmodelpath), state(state), servo_pin(servo_pin) {};

    void videoProcessing();

private:
    ServoController& servoController;
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
