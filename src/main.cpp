#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <time.h>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "LineDetector.hpp"
#include "Controller.hpp"

void videoProcessing(Controller& controller, LineDetector& detector, std::atomic<bool>& flag, bool& isVideo, 
                        std::string& videopath, int& width, int& height) {
    cv::VideoCapture cap;
    if (isVideo) {
        std::string video_path = videopath;
        cap.open(video_path);
    }
    else {
        cap.open(0, cv::CAP_V4L2);
    }
    if (!cap.isOpened()) {
        std::cerr << "打开失败" << std::endl;
        return;
    }
    while (cap.isOpened()) {
        cv::Mat frame;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        cap >> frame;
        if (frame.empty()) break;

        DetectResult result;
        cv::resize(frame, frame, cv::Size(width, height));
        detector.detect(&frame, &result);
        flag.store(result.has_crosswalk, std::memory_order_release);
        controller.pidControl(result.center.x, width);
        // std::cout << "*****偏移量: " << result.center.x - width / 2.0 << "*****" << std::endl;
        std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t1);
        double fps = 1.0 / time_span.count();
        cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        cv::imshow("frame", frame);
        int key = cv::waitKey(1);
        if (key == 27) break;
    }
    cap.release();
    cv::destroyAllWindows();
}

void videoRecord() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "打开失败" << std::endl;
        return;
    }

    // 保存录像
    // clock_t start, end;
    double fps = 30.0;
    cv::Size size = cv::Size(600, 400);
    std::string save_path = "/home/pi/Code/5G_ws/medias/playground.avi";
    const char *p = save_path.data();
    cv::VideoWriter writer(save_path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, size);
    // start = clock();
    int i = 0;
    while (cap.isOpened()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) break;
        cv::resize(frame, frame, size);
        std::cout << "录像帧数: " << i++ << std::endl;
        writer.write(frame);
    }
    writer.release();
}

void mover(Controller* controller, std::atomic<bool>& has_crosswalk) {
    controller->moveforward(has_crosswalk);
}

int main() {
    system("sudo killall pigpiod");
  	system("sudo cp /home/pi/.Xauthority /root/");
  	sleep(2);
    // system("aplay /home/pi/Code/5G_ws/medias/dz-banmaxian.wav");
    
    // 判断是否有配置文件
    if (access("/home/pi/Code/5G_ws/config/configs.yaml", F_OK) == -1) {
        std::cerr << "配置文件不存在" << std::endl;
        return -1;
    }

    YAML::Node config = YAML::LoadFile("/home/pi/Code/5G_ws/config/configs.yaml");

    // gpio参数
    int servo_pin = config["gpio"]["servo_pin"].as<int>();
    int pwm_pin = config["gpio"]["pwm_pin"].as<int>();

    // 视频参数
    int width = config["frame"]["width"].as<int>();
    int height = config["frame"]["height"].as<int>();

    // 检测参数
    std::string onnxmodelpath = config["linedetect"]["onnxmodelpath"].as<std::string>();

    // 获取 video 部分的参数
    bool isvideo = config["video"]["isvideo"].as<bool>();
    std::string videopath = config["video"]["videopath"].as<std::string>();

    // 移动控制参数
    bool movecontrol = config["movecontrol"].as<bool>();

    Controller controller(servo_pin, pwm_pin);
    LineDetector detector(width, height, onnxmodelpath);
    std::atomic<bool> flag(false);

    // std::thread video_record_thread(videoRecord);
    std::thread video_thread(videoProcessing, std::ref(controller), std::ref(detector), std::ref(flag), 
                            std::ref(isvideo), std::ref(videopath), std::ref(width), std::ref(height));
    std::thread move_thread(mover, &controller, std::ref(flag));
    try {
        // video_record_thread.join();
        video_thread.join();
        if (movecontrol) {
            move_thread.join();
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    gpioTerminate();
    return 0;
}