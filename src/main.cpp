#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <atomic>

#include "LineDetector.hpp"
#include "Controller.hpp"

int width = 300;
int height = 200;

void videoProcessing(Controller& controller, LineDetector& detector, std::atomic<bool>& flag) {
    std::string video_path = "/home/pi/5G_ws/medias/playground.mp4";
    cv::VideoCapture cap(video_path);
    // cv::VideoCapture cap(0, cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        std::cerr << "打开失败" << std::endl;
        return;
    }

    // double fps = cap.get(cv::CAP_PROP_FPS);
    // cv::Size size = cv::Size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    // std::string save_path = "/home/pi/5G_ws/medias/playground.avi";
    // cv::VideoWriter writer(save_path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, size);

    while (cap.isOpened()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        cv::resize(frame, frame, cv::Size(width, height));
        
        DetectResult result = detector.detect(&frame);
        flag.store(result.has_crosswalk, std::memory_order_release);
        controller.pidControl(result.center.x, width);
        cv::imshow("frame", frame);
        // writer.write(frame);
        int key = cv::waitKey(1);
        if (key == 27) break;
    }
    // writer.release();
    cap.release();
    cv::destroyAllWindows();
}

void imageProcessing(Controller& controller, LineDetector& detector, std::atomic<bool>& flag) {
    std::string image_path = "/home/pi/5G_ws/medias/playground_debug.jpg";
    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cerr << "读取失败" << std::endl;
        return;
    }
    cv::resize(frame, frame, cv::Size(width, height));
    DetectResult result = detector.detect(&frame);
    flag.store(result.has_crosswalk, std::memory_order_release);
    std::cout << "center: " << result.center << std::endl;
    controller.pidControl(result.center.x, width);
    cv::imshow("frame", frame);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void mover(Controller* controller, std::atomic<bool>& has_crosswalk) {
    controller->moveforward(has_crosswalk);
}

int main() {
    system("sudo killall pigpiod");
  	system("sudo cp /home/pi/.Xauthority /root/");
  	sleep(2);
  	int servo_pin = 12;
  	int pwm_pin = 13;
    Controller controller(servo_pin, pwm_pin);
    LineDetector detector(width, height);
    std::atomic<bool> flag(false);
    std::thread video_thread(videoProcessing, std::ref(controller), std::ref(detector), std::ref(flag));
    std::thread move_thread(mover, &controller, std::ref(flag));
    try {
        video_thread.join();
        move_thread.join();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    gpioTerminate();
    return 0;
}