#include <iostream>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <thread>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace cv;

int size_x = 90;
int size_y = 60;

void moveforward(int pwm_pin) {
    sleep(5);
    int i = 10750;
    while (true) {
        gpioPWM(pwm_pin, i);
        // i += 50;
        // if (i >= 11000) {
        //     i = 10000;
        // }
        sleep(3);
    }
}

double angleToDutyCycle(double angle) {
    return 2.5 + (angle / 180.0) * 10.0;
}

void initServo(int servo_pin) {
    gpioSetPWMfrequency(servo_pin, 50);
    gpioSetPWMrange(servo_pin, 100);
    // gpioPWM(servo_pin, angleToDutyCycle(45));
    // sleep(2);
    // gpioPWM(servo_pin, angleToDutyCycle(135));
    // sleep(2);
    gpioPWM(servo_pin, angleToDutyCycle(90));
    cout << "舵机初始化完成" << endl;
}

void initMotor(int pwm_pin) {
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    gpioPWM(pwm_pin, 10000);
    sleep(2);
    cout << "电机初始化完成" << endl;
}

void pidControl(double center_cal, int servo_pin) {
    static double kp = 0.33;
    static double kd = 0.2;
    static double last_error = 0;
    static double error = 0;

    double angle_outmax = 45;
    double angle_outmin = -45;

    error = center_cal - size_x / 2;
    double error_angle = kp * error + kd * (error - last_error);

    if (error_angle > angle_outmax) {
        error_angle = angle_outmax;
    } else if (error_angle < angle_outmin) {
        error_angle = angle_outmin;
    }

    double angle = 90 - error_angle;
    angle = (angle - 90) * 4 + 90;
    // cout << "servo angle: " << angle << endl;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    sleep(0.005);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

Mat color_thresh(const Mat& image, int s_thresh[2], int l_thresh[2], int b_thresh[2], int v_thresh[2]) {
    Mat luv, hls, hsv, lab;
    cvtColor(image, luv, COLOR_RGB2Luv);
    cvtColor(image, hls, COLOR_RGB2HLS);
    cvtColor(image, hsv, COLOR_RGB2HSV);
    cvtColor(image, lab, COLOR_RGB2Lab);

    vector<Mat> luv_channels, hls_channels, hsv_channels, lab_channels;
    split(luv, luv_channels);
    split(hls, hls_channels);
    split(hsv, hsv_channels);
    split(lab, lab_channels);

    Mat s_channel = hsv_channels[1];
    Mat l_channel = hls_channels[1];
    Mat b_channel = lab_channels[2];
    Mat v_channel = hsv_channels[2];

    Mat s_binary = Mat::zeros(s_channel.size(), CV_8UC1);
    inRange(s_channel, Scalar(s_thresh[0]), Scalar(s_thresh[1]), s_binary);
    // imshow("s_binary", s_binary);

    Mat b_binary = Mat::zeros(b_channel.size(), CV_8UC1);
    inRange(b_channel, Scalar(b_thresh[0]), Scalar(b_thresh[1]), b_binary);
    // imshow("b_binary", b_binary);

    Mat l_binary = Mat::zeros(l_channel.size(), CV_8UC1);
    inRange(l_channel, Scalar(l_thresh[0]), Scalar(l_thresh[1]), l_binary);
    // imshow("l_binary", l_binary);

    Mat v_binary = Mat::zeros(v_channel.size(), CV_8UC1);
    inRange(v_channel, Scalar(v_thresh[0]), Scalar(v_thresh[1]), v_binary);
    // imshow("v_binary", v_binary);

    Mat combined = Mat::zeros(s_channel.size(), CV_8UC1);
    bitwise_and(s_binary, b_binary, combined);
    bitwise_and(combined, l_binary, combined);
    bitwise_and(combined, v_binary, combined);

    return combined;
}

void videoProcessing(int servo_pin) {
    // namedWindow("trackbar", WINDOW_NORMAL);
    int s_thresh[2] = {114, 255};
    int l_thresh[2] = {109, 174};
    int b_thresh[2] = {162, 232};
    int v_thresh[2] = {176, 224};
    // createTrackbar("s_min", "trackbar", &(s_thresh[0]), 255, nullptr);
    // createTrackbar("s_max", "trackbar", &(s_thresh[1]), 255, nullptr);
    // createTrackbar("l_min", "trackbar", &(l_thresh[0]), 255, nullptr);
    // createTrackbar("l_max", "trackbar", &(l_thresh[1]), 255, nullptr);
    // createTrackbar("b_min", "trackbar", &(b_thresh[0]), 255, nullptr);
    // createTrackbar("b_max", "trackbar", &(b_thresh[1]), 255, nullptr);
    // createTrackbar("v_min", "trackbar", &(v_thresh[0]), 255, nullptr);
    // createTrackbar("v_max", "trackbar", &(v_thresh[1]), 255, nullptr);
    // const string video_path = "/home/pi/5G_ws/output8.avi";
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cout << "视频打开失败" << endl;
        return;
    }

    Mat mask = Mat::zeros(Size(size_x, size_y / 2), CV_8UC1);
    vector<Point> points = {Point(size_x / 4, 0), Point(size_x * 3 / 4, 0), Point(size_x, size_y), Point(0, size_y)};
    fillPoly(mask, vector<vector<Point>>{points}, Scalar(255));

    while (cap.isOpened()) {
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // cout << "获取图像延迟: " << time_span1.count() * 1000 << "ms" << endl;
        resize(frame, frame, Size(size_x, size_y));
        Mat roi = frame(Rect(0, size_y / 4, size_x, size_y / 2));
        // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
        // cout << "ROI延迟: " << time_span2.count() * 1000 << "ms" << endl;
        
        // int s_thresh[2] = {getTrackbarPos("s_min", "trackbar"), getTrackbarPos("s_max", "trackbar")};
        // int l_thresh[2] = {getTrackbarPos("l_min", "trackbar"), getTrackbarPos("l_max", "trackbar")};
        // int b_thresh[2] = {getTrackbarPos("b_min", "trackbar"), getTrackbarPos("b_max", "trackbar")};
        // int v_thresh[2] = {getTrackbarPos("v_min", "trackbar"), getTrackbarPos("v_max", "trackbar")};

        Mat combined = color_thresh(roi, s_thresh, l_thresh, b_thresh, v_thresh);
        // std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span3 = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
        // cout << "颜色阈值延迟: " << time_span3.count() * 1000 << "ms" << endl;
        Mat binary;
        bitwise_and(combined, mask, binary);
        // std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span4 = std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4);
        // cout << "二值化延迟: " << time_span4.count() * 1000 << "ms" << endl;
        // cv::imshow("binary", binary);
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        // std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span5 = std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t5);
        // cout << "轮廓查找延迟: " << time_span5.count() * 1000 << "ms" << endl;

        vector<double> center_x;
        for (const auto& contour : contours) {
            // if (contourArea(contour) < 5) continue;

            RotatedRect rect = minAreaRect(contour);
            double angle = rect.angle;
            double width = rect.size.width;
            double height = rect.size.height;

            if (width < height) {
                angle += 90;
            }
            // cout << "angle: " << angle << endl;
            if (angle > 150 || angle < 30) continue;

            Point2f box[4];
            rect.points(box);

            center_x.push_back(rect.center.x);
        }
        
        // cout << "center_x: " << center_x.size() << endl;
        double center = size_x / 2;
        if (!center_x.empty()) {
            sort(center_x.begin(), center_x.end());
            if (center_x.size() >= 2) {
                center = (center_x[0] + center_x.back()) / 2;
            } else {
                if (center_x[0] < center) {
                    center = center_x[0] / 2 + center;
                } else {
                    center = center_x[0] / 2;
                }
            }
        }
        // std::chrono::high_resolution_clock::time_point t7 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span6 = std::chrono::duration_cast<std::chrono::duration<double>>(t7 - t6);
        // cout << "中心计算延迟: " << time_span6.count() * 1000 << "ms" << endl;
        // cv::circle(frame, Point(center, size_y / 2), 5, Scalar(0, 0, 255), -1);
        // cv::imshow("frame", frame);
        // if (waitKey(1) == 27) break;
        pidControl(center, servo_pin);
        // std::chrono::high_resolution_clock::time_point t8 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span7 = std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t7);
        // cout << "舵机控制延迟: " << time_span7.count() * 1000 << "ms" << endl;
    }

    cap.release();
    // destroyAllWindows();
}

int main() {
    system("sudo killall pigpiod");
    system("sudo cp /home/pi/.Xauthority /root/");
    sleep(2);

    int servo_pin = 12;
    int pwm_pin = 13;
    int pi = gpioInitialise();
    if (pi < 0) {
        cerr << "pigpio 初始化失败" << endl;
        return -1;
    }

    initServo(servo_pin);
    initMotor(pwm_pin);
    sleep(1);

    std::thread t1(videoProcessing, servo_pin), t2(moveforward, pwm_pin);
    t1.join();
    t2.join();

    gpioTerminate();
    return 0;
}
