#include <opencv2/opencv.hpp>
#include "Logger.hpp"

class LetterClassificator {
public:

    double blueAreaCount(const cv::Mat& img, cv::Mat& roi) {
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(70, 100, 35), cv::Scalar(145, 255, 255), mask);

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
            // 裁剪ROI，取中心部分
            int x = blueROI.x + blueROI.width / 4;
            int y = blueROI.y;
            int w = blueROI.width / 2;
            int h = blueROI.height;
            roi = img(cv::Rect(x, y, w, h));
        }
        return cv::countNonZero(mask);
    }

    int recognize(const cv::Mat& roi) {
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // 投影变换
        cv::Point2f src[4] = {cv::Point2f(10, 0), cv::Point2f(roi.cols - 10, 0), cv::Point2f(roi.cols, roi.rows), cv::Point2f(0, roi.rows)}; 
        cv::Point2f dst[4] = {cv::Point2f(20, 0), cv::Point2f(roi.cols - 20, 0), cv::Point2f(roi.cols - 20, roi.rows), cv::Point2f(20, roi.rows)};
        cv::Mat t = cv::getPerspectiveTransform(src, dst);
        cv::warpPerspective(binary, binary, t, binary.size());
        Logger::getLogger()->showMat("binary", binary);

        cv::imwrite("/home/pi/Code/5GSmartCar/medias/letter.jpg", binary);

        // 使用onnx模型识别字母
        cv::dnn::Net net = cv::dnn::readNetFromONNX("/home/pi/Code/5GSmartCar/models/letter_model.onnx");
        if (net.empty()) {
        	Logger::getLogger()->error("无法加载ONNX模型");
        	return -1;
        }
        cv::Mat blob;
        blob = cv::dnn::blobFromImage(binary, 1 / 255.0, cv::Size(50, 50), cv::Scalar(0, 0, 0));
        net.setInput(blob);
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        cv::Mat outputs = net.forward();
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        int max_index = 0;
        float max_value = 0.0;
        for (int i = 0; i < outputs.cols; i++) {
        	if (outputs.at<float>(0, i) > max_value) {
        		max_value = outputs.at<float>(0, i);
        		max_index = i;
        	}
        }
	    std::string onnx_outputs = "";
        for (int i = 0; i < outputs.cols; i++) {
        	onnx_outputs += std::to_string(outputs.at<float>(0, i)) + " ";
        }
	    Logger::getLogger()->info("********字母识别ONNX输出********\n" + onnx_outputs + 
	    							"\n推理时间: " + std::to_string(time_span.count() * 1000) + "ms");
        return max_index;
    }

private:
};