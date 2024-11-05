#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <iostream>

int main() {
	cv::Mat img = cv::imread("/home/pi/Code/5GSmartCar/medias/A.jpg");
	cv::resize(img, img, cv::Size(640, 480));
	// cv::imshow("Image", img);

	cv::Mat hsv;
	cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
	// cv::imshow("HSV", hsv);

	cv::Scalar lowerblue(100, 150, 50);
	cv::Scalar upperblue(140, 255, 255);

	cv::Mat mask;
	cv::inRange(hsv, lowerblue, upperblue, mask);
	// cv::imshow("Mask", mask);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	int largestContourIndex = -1;
	double maxArea = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = cv::contourArea(contours[i]);
		if (area > maxArea) {
			maxArea = area;
			largestContourIndex = i;
		}
	}

	if (largestContourIndex != -1) {
		cv::Rect blueROI = cv::boundingRect(contours[largestContourIndex]);
		cv::Mat roi = img(blueROI);
		// cv::imshow("ROI", roi);

		// 将ROI转换为灰度图并二值化
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // 初始化Tesseract API
        tesseract::TessBaseAPI ocr;
        if (ocr.Init(nullptr, "eng")) {  // 使用英语识别
            std::cerr << "无法初始化Tesseract OCR" << std::endl;
            return -1;
        }

        // 将二值化后的图像传递给Tesseract进行OCR识别
        ocr.SetImage(binary.data, binary.cols, binary.rows, 1, binary.step);
        std::string text = ocr.GetUTF8Text();
        std::cout << "识别结果: \n" << text << std::endl;

        // 释放Tesseract资源
        ocr.End();
	}

	cv::waitKey(0);
	cv::destroyAllWindows();
	return 0;
}