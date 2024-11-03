#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap("/dev/cam0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error opening camera." << std::endl;
        return -1;
    }
    cv::namedWindow("HSV Tuning", cv::WINDOW_AUTOSIZE);
    int hmin = 0, hmax = 179, smin = 0, smax = 255, vmin = 0, vmax = 255;
    cv::createTrackbar("Hue Min", "HSV Tuning", &hmin, 179);
    cv::createTrackbar("Hue Max", "HSV Tuning", &hmax, 179);
    cv::createTrackbar("Sat Min", "HSV Tuning", &smin, 255);
    cv::createTrackbar("Sat Max", "HSV Tuning", &smax, 255);
    cv::createTrackbar("Val Min", "HSV Tuning", &vmin, 255);
    cv::createTrackbar("Val Max", "HSV Tuning", &vmax, 255);

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
    cap.set(cv::CAP_PROP_EXPOSURE, 0.0);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error capturing frame." << std::endl;
            break;
        }
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), mask);
        cv::imshow("frame", frame);
        cv::imshow("mask", mask);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
}