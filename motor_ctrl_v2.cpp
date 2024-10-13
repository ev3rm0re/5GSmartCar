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

int size_x = 300;
int size_y = 200;

void moveforward(int pwm_pin) {
    sleep(2);
    int i = 12300;
    while (true) {
        gpioPWM(pwm_pin, i);
        if (i != 12900) i += 100;
        else cout << "启动！" << endl;
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
    // gpioPWM(pwm_pin, 10000);
    // sleep(2);
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
    angle = (angle - 90) * 2 + 90;
    // cout << "servo angle: " << angle << endl;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    sleep(0.005);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

Mat pipeline(const Mat& img, int s_thresh_min = 170, int s_thresh_max = 255, int sx_thresh_min = 40, int sx_thresh_max = 200) {
    Mat img_copy;
    img.copyTo(img_copy);

    // 1. 将图像转换为HLS色彩空间，并分离各个通道
    Mat hls;
    cvtColor(img_copy, hls, COLOR_BGR2HLS);
    vector<Mat> channels;
    split(hls, channels);
    Mat h_channel = channels[0];
    Mat l_channel = channels[1];
    Mat s_channel = channels[2];

    // 2. 利用Sobel计算x方向的梯度
    Mat sobelx;
    Sobel(l_channel, sobelx, CV_64F, 1, 0);
    Mat abs_sobelx;
    convertScaleAbs(sobelx, abs_sobelx);

    // 将导数转换为8bit整数
    Mat scaled_sobel;
    double minVal, maxVal;
    minMaxLoc(abs_sobelx, &minVal, &maxVal);
    abs_sobelx.convertTo(scaled_sobel, CV_8U, 255.0 / maxVal);

    Mat sxbinary = Mat::zeros(scaled_sobel.size(), CV_8U);
    inRange(scaled_sobel, sx_thresh_min, sx_thresh_max, sxbinary);

    // 3. 对s通道进行阈值处理
    Mat s_binary = Mat::zeros(s_channel.size(), CV_8U);
    inRange(s_channel, s_thresh_min, s_thresh_max, s_binary);

    // 4. 将边缘检测的结果和颜色空间阈值的结果合并，并结合l通道的取值，确定车道提取的二值图结果
    Mat color_binary = Mat::zeros(sxbinary.size(), CV_8U);
    for (int i = 0; i < sxbinary.rows; i++) {
        for (int j = 0; j < sxbinary.cols; j++) {
            if ((sxbinary.at<uchar>(i, j) == 255 || s_binary.at<uchar>(i, j) == 255) && l_channel.at<uchar>(i, j) > 100) {
                color_binary.at<uchar>(i, j) = 255;
            }
        }
    }

    return color_binary;
}

Mat detectLanes(const Mat& img) {
    Mat gray, blurred, edges;

    // 1. 将图像转换为灰度图
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // 2. 使用高斯模糊平滑图像
    GaussianBlur(gray, blurred, Size(5, 5), 0);

    // 3. 使用Canny边缘检测
    Canny(blurred, edges, 50, 150);

    // 4. 使用霍夫变换检测直线
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // 5. 绘制检测到的直线
    Mat result;
    cvtColor(edges, result, COLOR_GRAY2BGR);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }

    return result;
}

void videoProcessing(int servo_pin) {
    const string video_path = "/home/pi/5G_ws/output8.avi";
    VideoCapture cap(video_path);
    // VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cout << "视频打开失败" << endl;
        return;
    }

    while (cap.isOpened()) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        resize(frame, frame, Size(size_x, size_y));
        Mat roi = frame(Rect(0, size_y / 2, size_x, size_y / 2));

        Mat binary;
        binary = pipeline(roi);
        imshow("binary", binary);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(35, 1));
        Mat blackhatImg;
        morphologyEx(binary, blackhatImg, MORPH_BLACKHAT, kernel);
        imshow("blackhat", blackhatImg);

        Mat canny;
        Canny(blackhatImg, canny, 100, 255);
        imshow("canny", canny);

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
                    // cv::circle(frame, cv::Point(j, i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                    for (int k = j; k < canny.cols; k++) {
                        if (canny.at<uchar>(i, k) == 255 && k - j > size_x / 4) {
                            right[t] = k;
                            flag_right = 1;
                            // cv::circle(frame, cv::Point(k, i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                            break;
                        }
                    }
                    break;
                }
                if (flag_left == 0) {
                    left[t] = 0;
                    // cv::circle(frame, cv::Point(0, i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                    right[t] = size_x - 1;
                    // cv::circle(frame, cv::Point(599, i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                }
                if (flag_left == 1 && flag_right == 0) {
                    right[t] = size_x - 1;
                }
            }
            if (t > 0) {
                if (flag_right == 0) {
                    if (left[t - 1] - left[t] > 0) {
                        int n = left[t - 1];
                        left[t - 1] = size_x - 1 - right[t - 1];
                        right[t - 1] = n;
                        // cv::circle(frame, cv::Point(left[t - 1], i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                    }
                    // else {
                    //     cv::circle(frame, cv::Point(right[t - 1], i + size_y / 2), 1, cv::Scalar(0, 0, 255), 2);
                    // }
                }
                mid[t - 1] = (left[t - 1] + right[t - 1]) / 2;
                mid_sum += mid[t - 1];
            }
            t++;
        }
        double mid_final = mid_sum / canny.rows;

        cv::circle(frame, Point(mid_final, size_y / 2), 5, Scalar(0, 0, 255), -1);
        cv::imshow("frame", frame);
        int key = waitKey(50);
        if (key == 27) break;
        else continue;

        pidControl(mid_final, servo_pin);
    }

    cap.release();
    destroyAllWindows();
}

// int main() {
//     system("sudo killall pigpiod");
//     system("sudo cp /home/pi/.Xauthority /root/");
//     sleep(2);

//     int servo_pin = 12;
//     int pwm_pin = 13;
//     int pi = gpioInitialise();
//     if (pi < 0) {
//         cerr << "pigpio 初始化失败" << endl;
//         return -1;
//     }

//     initServo(servo_pin);
//     initMotor(pwm_pin);
//     sleep(1);

//     thread t1(videoProcessing, servo_pin);
//     // thread t2(moveforward, pwm_pin);
//     try {
//         t1.join();
//         // t2.join();
//     } catch (const exception& e) {
//         cerr << "Exception: " << e.what() << endl;
//     }

//     gpioTerminate();
//     return 0;
// }

int main() {
    const string video_path = "/home/pi/5G_ws/output238.avi";
    VideoCapture cap(video_path);
    // VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cout << "视频打开失败" << endl;
        return 1;
    }

    while (cap.isOpened()) {
        Mat frame;
        cap >> frame;
        Mat lane_detected = detectLanes(frame);

        imshow("Detected Lanes", lane_detected);
        if (waitKey(1) == 27) break;
    }
    return 0;
}