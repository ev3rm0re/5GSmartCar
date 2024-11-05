#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

class LetterOCR {
public:
    LetterOCR(const std::string& lang = "eng") {
        if (ocr_.Init(nullptr, lang.c_str())) {
            std::cerr << "无法初始化Tesseract OCR" << std::endl;
            exit(-1);
        }
    }

    ~LetterOCR() {
        ocr_.End();
    }

    double blueAreaCount(const cv::Mat& img, cv::Mat& roi) {
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), mask);

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

    std::string recognize(const cv::Mat& roi) {
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // 投影变换
        cv::Point2f src[4] = {cv::Point2f(10, 0), cv::Point2f(roi.cols - 10, 0), cv::Point2f(roi.cols, roi.rows), cv::Point2f(0, roi.rows)}; 
        cv::Point2f dst[4] = {cv::Point2f(20, 0), cv::Point2f(roi.cols - 20, 0), cv::Point2f(roi.cols - 20, roi.rows), cv::Point2f(20, roi.rows)};
        cv::Mat t = cv::getPerspectiveTransform(src, dst);
        cv::warpPerspective(binary, binary, t, binary.size());

        Logger::getLogger()->showMat("binary", binary);

        ocr_.SetImage(binary.data, binary.cols, binary.rows, 1, binary.step);
        return ocr_.GetUTF8Text();
    }

private:
    tesseract::TessBaseAPI ocr_;
};