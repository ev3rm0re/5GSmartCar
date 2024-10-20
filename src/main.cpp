#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <time.h>
#include <chrono>

#include "LineDetector.hpp"
#include "Controller.hpp"

int width = 300;
int height = 200;

void videoProcessing(Controller& controller, LineDetector& detector, std::atomic<bool>& flag) {
    std::string video_path = "/home/pi/Code/5G_ws/medias/arrow.avi";
    cv::VideoCapture cap(video_path);
    // cv::VideoCapture cap(0, cv::CAP_V4L2);
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

void imageProcessing(Controller& controller, LineDetector& detector, std::atomic<bool>& flag) {
    std::string image_path = "/home/pi/Code/5G_ws/medias/playground_arrow.jpg";
    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cerr << "读取失败" << std::endl;
        return;
    }
    DetectResult result;
    cv::resize(frame, frame, cv::Size(width, height));
    detector.detect(&frame, &result);
    flag.store(result.has_crosswalk, std::memory_order_release);
    controller.pidControl(result.center.x, width);
    cv::imshow("frame", frame);
    cv::waitKey(0);
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
  	int servo_pin = 12;
  	int pwm_pin = 13;
    Controller controller(servo_pin, pwm_pin);
    LineDetector detector(width, height);
    std::atomic<bool> flag(false);

    // std::thread video_record_thread(videoRecord);
    std::thread video_thread(videoProcessing, std::ref(controller), std::ref(detector), std::ref(flag));
    // std::thread move_thread(mover, &controller, std::ref(flag));
    try {
        // video_record_thread.join();
        video_thread.join();
        // move_thread.join();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    gpioTerminate();
    return 0;
}