#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "打开失败" << std::endl;
        return -1;
    }
    while (cap.isOpened()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("frame", frame);
        int key = cv::waitKey(1);
        if (key == 27) break;
    }
}