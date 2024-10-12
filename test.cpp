#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace cv;

int main(int argc, char *argv[])
{
    VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }
    while (cap.isOpened())
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Error frame empty" << std::endl;
            break;
        }
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "获取图像延迟: " << time_span.count() * 1000 << "ms" << std::endl;
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
        std::cout << "灰度转换延迟: " << time_span2.count() * 1000 << "ms" << std::endl;
        // imshow("Frame", frame);
        // if (waitKey(1) == 27)
        //     break;
    }
}