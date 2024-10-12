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
        cv::Mat resized, roi, gray, equal, binary, blackhat, canny, kernel;
        cv::resize(frame, resized, cv::Size(600, 400));
        roi = resized(cv::Rect(0, 200, 600, 200));
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, equal);
        cv::imshow("equal", equal);
        cv::threshold(equal, binary, 235, 255, cv::THRESH_BINARY);
        cv::imshow("binary", binary);
        // kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // cv::morphologyEx(binary, blackhat, cv::MORPH_BLACKHAT, kernel);
        cv::Canny(binary, canny, 150, 255);

        int t = 0;
        int left[canny.rows], right[canny.rows], mid[canny.rows];
        for (int i = 0; i < canny.rows; i++) {
            left[i] = -1;
            right[i] = -1;
            mid[i] = -1;
        }

        double mid_sum = 0;
        left[0] = 0, right[0] = 0;

        for (int i = canny.rows - 1; i >= 0; i--) {
            int flag_left = 0, flag_right = 0;
            for (int j = 0; j < canny.cols; j++) {
                if (canny.at<uchar>(i, j) == 255) {
                    left[t] = j;
                    flag_left = 1;
                    // cv::circle(roi, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 2);
                    for (int k = j; k < canny.cols; k++) {
                        if (canny.at<uchar>(i, k) == 255 && k - j > 100) {
                            right[t] = k;
                            flag_right = 1;
                            // cv::circle(roi, cv::Point(k, i), 1, cv::Scalar(0, 0, 255), 2);
                            break;
                        }
                    }
                    break;
                }
                if (flag_left == 0) {
                    left[t] = 0;
                    // cv::circle(roi, cv::Point(0, i), 1, cv::Scalar(0, 0, 255), 2);
                    right[t] = 599;
                    // cv::circle(roi, cv::Point(599, i), 1, cv::Scalar(0, 0, 255), 2);
                }
                if (flag_left == 1 && flag_right == 0) {
                    right[t] = 599;
                }
            }
            if (t > 0) {
                if (flag_right == 0) {
                    if (left[t - 1] - left[t] > 0) {
                        int n = left[t - 1];
                        left[t - 1] = 599 - right[t - 1];
                        right[t - 1] = n;
                        // cv::circle(roi, cv::Point(left[t - 1], i), 1, cv::Scalar(0, 0, 255), 2);
                    }
                    else {
                        // cv::circle(roi, cv::Point(right[t - 1], i), 1, cv::Scalar(0, 0, 255), 2);
                    }
                }
                mid[t - 1] = (left[t - 1] + right[t - 1]) / 2;
                mid_sum += mid[t - 1];
            }
            t++;
        }

        double mid_final = mid_sum / canny.rows;
        cv::circle(canny, cv::Point(mid_final, canny.rows / 2), 3, cv::Scalar(255, 255, 255), -1);
        cv::imshow("canny", canny);
        if (cv::waitKey(50) == 27) {
            break;
        }
        std::cout << mid_final << std::endl;

        error = mid_final - 300;
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

    std::thread t1(process);
    try {
        t1.join();
    }
    catch (std::exception &e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    gpioTerminate();
    return 0;
}