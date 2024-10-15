#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>

#include "LineDetector.hpp"
#include "Controller.hpp"

void videoProcessing(Controller controller, LineDetector detector) {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "打开失败" << std::endl;
        return;
    }
    int width = 300;
    int height = 200;

    while (cap.isOpened()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        cv::resize(frame, frame, cv::Size(width, height));
        cv::Point2f center = detector.detect(&frame);
        controller.pidControl(center.x, width);
        cv::imshow("frame", frame);
        int key = cv::waitKey(1);
        if (key == 27) break;
    }
}

void move(Controller* controller) {
    controller->moveforward();
}

int main() {
    system("sudo killall pigpiod");
  	system("sudo cp /home/pi/.Xauthority /root/");
  	sleep(2);
  	int servo_pin = 12;
  	int pwm_pin = 13;
    Controller controller(servo_pin, pwm_pin);
    LineDetector detector(300, 200);

    std::thread video_thread(videoProcessing, controller, detector);
    std::thread move_thread(move, &controller);
    try {
        video_thread.join();
        move_thread.join();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    gpioTerminate();
    return 0;
}