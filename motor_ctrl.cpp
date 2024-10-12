#include <iostream>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <thread>
#include <time.h>
#include <chrono>


double angleToDutycycle(int angle) {
    return 2.5 + (angle / 180.0) * 10.0;
}

void forward(int pwmPin) {
    time_sleep(5); // 等待摄像头初始化
    int i = 11000;
    while (true) {
        if (i < 12000) i+=100;
        gpioPWM(pwmPin, i);
        time_sleep(3);
    }
}

int servoPin = 12;
int pwmPin = 13;
double kp = 0.33;
double kd = 0.11;
double last_error = 0;
double error = 0;
int angle_max = 45;
int angle_min = -45;

void process() {
    cv::VideoCapture cap;
    const std::string filename = "/home/pi/5G_ws/output8.avi";
    cap.open(filename);
    if (!cap.isOpened()) {
        std::cout << "Camera not found" << std::endl;
        return;
    }
    while (cap.isOpened()) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cout << "Frame not found" << std::endl;
            break;
        }
        cv::Mat resized, roi, gray, equal, binary;
        cv::resize(frame, resized, cv::Size(600, 400));
        roi = resized(cv::Rect(0, 200, 600, 200));
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, equal);
        cv::imshow("equal", equal);
        cv::threshold(equal, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
        cv::imshow("binary", binary);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (int i = 0; i < contours.size(); i++) {
            cv::drawContours(roi, contours, i, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("roi", roi);
        if (cv::waitKey(50) == 27) {
            break;
        }

        error = 300 - 300;
        double error_angle = kp * error + kd * (error - last_error);
        if (error_angle > angle_max) {
            error_angle = angle_max;
        }
        if (error_angle < angle_min) {
            error_angle = angle_min;
        }
        double angle = 85 - error_angle;
        last_error = error;

        gpioPWM(servoPin, angleToDutycycle(angle));
        time_sleep(0.01);
        gpioPWM(servoPin, angleToDutycycle(angle));
    }
    cap.release();
    cv::destroyAllWindows();
}

int main(int argc, char *argv[]) {
    system("sudo killall pigpiod");
    system("sudo cp /home/pi/.Xauthority /root/");
    // system("sudo pigpiod");
    // system("sudo systemctl stop network-rc.service");
    // time_sleep(1);

    if (gpioInitialise() < 0) {
        std::cout << "GPIO Initialization failed" << std::endl;
        return 1;
    }
    gpioSetMode(servoPin, PI_OUTPUT);
    gpioSetPWMfrequency(servoPin, 50);
    gpioSetPWMrange(servoPin, 100);
    gpioPWM(servoPin, angleToDutycycle(45));
    gpioDelay(500000);
    gpioPWM(servoPin, angleToDutycycle(135));
    gpioDelay(500000);
    gpioPWM(servoPin, angleToDutycycle(85));
    std::cout << "Servo test complete" << std::endl;

    gpioSetMode(pwmPin, PI_OUTPUT);
    gpioSetPWMfrequency(pwmPin, 200);
    gpioSetPWMrange(pwmPin, 40000);
    std::cout << "Motor test complete" << std::endl;

    std::thread t1(process), t2(forward, pwmPin);
    try {
        t1.join();
    }
    catch (std::exception &e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    gpioTerminate();
    return 0;
}