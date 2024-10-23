#pragma once

// std
#include <vector>

// 边线结构体
#include "Struct.hpp"
#include "Logger.hpp"


// LineDetector: 负责边线检测
class LineDetector {
public:
    LineDetector(const int width, const int height) : width(width), height(height) {};
    bool isLine(Line& line) const {
        return std::abs(line.slope) > 0.2 && line.length > height / 4.0 && line.length > 30.0;
    }
    
	std::vector<Line> detectLines(cv::Mat* binary) const {
        // 查找边线
	    cv::Mat canny;
	    cv::Canny(*binary, canny, 50, 150);

	    std::vector<cv::Vec4i> linesP;
	    cv::HoughLinesP(canny, linesP, 1, CV_PI / 180, 50, 50, 20);

	    std::vector<Line> lines;
	    for (const auto& lineP : linesP) {
	    	Line line({ cv::Point(lineP[0], lineP[1]), cv::Point(lineP[2], lineP[3]) });
	    	if (!isLine(line)) continue;
	    	lines.push_back(line);
	    }
	    return lines;
    };

    void filterLines(std::vector<Line>* lines) const {
        if (lines->size() >= 2) {
            std::vector<bool> toKeep(lines->size(), true);  // 用于标记保留哪些线

            for (size_t i = 0; i < lines->size(); ++i) {
                if (!toKeep[i]) continue;  // 跳过已经删除的线

                for (size_t j = i + 1; j < lines->size(); ++j) {
                    if (!toKeep[j]) continue;  // 跳过已经删除的线

                    if ((*lines)[i].slope < 0 && (*lines)[j].slope < 0) {
                        // 两条线斜率为负，删除靠近中心的
                        if ((*lines)[i].center.x < (*lines)[j].center.x) {
                            toKeep[j] = false;  // 标记删除第 j 条
                        } else {
                            toKeep[i] = false;  // 标记删除第 i 条
                            break;  // 当前 i 被删除，跳出内循环
                        }
                    } else if ((*lines)[i].slope > 0 && (*lines)[j].slope > 0) {
                        // 两条线斜率为正，删除靠近中心的
                        if ((*lines)[i].center.x < (*lines)[j].center.x) {
                            toKeep[i] = false;  // 标记删除第 i 条
                            break;  // 当前 i 被删除，跳出内循环
                        } else {
                            toKeep[j] = false;  // 标记删除第 j 条
                        }
                    }
                }
            }

            // 实际删除操作
            auto it = lines->begin();
            for (size_t i = 0; i < toKeep.size(); ++i) {
                if (!toKeep[i]) {
                    it = lines->erase(it);  // 删除被标记为 false 的线
                } else {
                    ++it;
                }
            }
        }

	    if (lines->size() == 1) {
            if ((*lines)[0].center.x < width / 2.0) {
	    		cv::Point newCenter = cv::Point(width, (*lines)[0].center.y);
	    		cv::Point newTop = cv::Point(width + (*lines)[0].center.y / (*lines)[0].slope, 0);
	    		Line newLine(std::vector<cv::Point>{newTop, newCenter});
	    		newLine.center = newCenter;
	    		lines->push_back(newLine);
	    	} else {
	    		cv::Point newCenter = cv::Point(0, (*lines)[0].center.y);
	    		cv::Point newTop = cv::Point(0 + (*lines)[0].center.y / (*lines)[0].slope, 0);
	    		Line newLine(std::vector<cv::Point>{newTop, newCenter});
	    		newLine.center = newCenter;
	    		lines->push_back(newLine);
	    	}
        }
    };

private:
	int width;
	int height;
};


// LaneDetector: 负责车道检测
class LaneDetector {
public:
   	LaneDetector(const int width, const int height) : width(width), height(height) {};

    bool isLane(const Lane& lane) const {
        return lane.width > width / 4.0 && lane.center.x > width / 4.0 && lane.center.x < width * 3 / 4.0 &&
		std::abs(lane.left_line.center.y - lane.right_line.center.y) < height / 3.0;
    };

    void getLane(std::vector<Line>& lines, Lane* lane) const {
        for (size_t i = 0; i < lines.size(); i++) {
		    for (size_t j = i + 1; j < lines.size(); j++) {
		    	Lane lane_(lines[i], lines[j]);
		    	if (isLane(lane_)) {
		    		*lane = lane_;
		    		return;
		    	}
		    }
        }
    };

private:
    int width;
    int height;
};


// CrosswalkDetector: 负责人行横道检测
class CrosswalkDetector {
public:
    CrosswalkDetector(int width, int height) : width(width), height(height) {};
    bool isCrosswalk(CrossWalk& crosswalk) const {
        return std::abs(crosswalk.slope) > 5.0 && 
		        crosswalk.area > width * height / 120 && crosswalk.area < width * height / 50.0 && 
		        crosswalk.height / crosswalk.width < 2.0 && crosswalk.height / crosswalk.width > 0.1;
    };
    bool hasCrosswalk(cv::Mat* binary) const {
        bool has_crosswalk = false;
	    // 查找人行横道
	    std::vector<CrossWalk> crosswalks;
	    std::vector<std::vector<cv::Point>> contours;
	    cv::findContours(*binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	    for (const auto& contour : contours) {
	    	cv::RotatedRect rect = cv::minAreaRect(contour);
	    	CrossWalk crosswalk(rect);
	    	if (isCrosswalk(crosswalk)) {
	    		crosswalks.push_back(crosswalk);
	    	}
	    }

	    if (crosswalks.size() > 2) has_crosswalk = true;

	    return has_crosswalk;
    };

private:
    int width;
    int height;
};


// BinaryImageProcessor: 负责二值化图像处理
class BinaryImageProcessor {
public:
    BinaryImageProcessor(int width, int height) : width(width), height(height) {};

    cv::Mat getBinaryFrame(cv::Mat* frame, cv::Rect ROI, int threshold) const {
        cv::Mat frame_copy = frame->clone();
	    cv::Mat roi = frame_copy(ROI);
	    cv::Mat gray;
	    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
	    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
	    cv::Mat binary;
	    cv::threshold(gray, binary, threshold, 255, cv::THRESH_BINARY);
	    return binary;
    };

    void adjustThreshold(int white_count, int* threshold, int lines_size) const {
        int current_threshold = *threshold;
	    if (white_count < width * height / 80.0 && lines_size == 0) {
	    	(*threshold) -= 2;
	    }
	    else if (white_count < width * height / 100 && lines_size == 1) {
	    	(*threshold)--;
	    }
	    else if (white_count > width * height / 15.0) {
	    	(*threshold)++;
	    }
	    if (current_threshold != *threshold) {
	    	Logger::getLogger()->debug("二值化阈值更新为: " + std::to_string(*threshold));
	    }
    };

private:
    int width;
    int height;
};


// ArrowProcessor: 负责箭头检测
class ArrowProcessor {
public:
    ArrowProcessor(const std::string& modelPath) : onnxModelPath(modelPath) {};

    int detectArrowDirection(cv::Mat* frame) const {
        // 使用onnx模型识别箭头
	    cv::Mat frame_copy = frame->clone();
	    cv::dnn::Net net = cv::dnn::readNetFromONNX(onnxModelPath);
	    if (net.empty()) {
	    	Logger::getLogger()->error("无法加载ONNX模型");
	    	return -1;
	    }

	    cv::Mat blob;
	    blob = cv::dnn::blobFromImage(frame_copy, 1 / 255.0, cv::Size(128, 128), cv::Scalar(0, 0, 0), true, false);
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

		Logger::getLogger()->info("********ONNX输出********\n" + onnx_outputs + 
									"\n推理时间: " + std::to_string(time_span.count() * 1000) + "ms");

	    return max_index;
    };

private:
    std::string onnxModelPath;
};


// BlueBoardDetector: 负责蓝色板检测
class BlueBoardDetector {
public:
	BlueBoardDetector(int width, int height) : width(width), height(height) {};
	bool hasBlueBoard(cv::Mat* frame) const {
		cv::Scalar upperblue = cv::Scalar(125, 255, 255);
		cv::Scalar lowerblue = cv::Scalar(90, 130, 150);

		cv::Mat hsv_frame;
		cv::cvtColor(*frame, hsv_frame, cv::COLOR_BGR2HSV);

		cv::Mat blue_mask;
		cv::inRange(hsv_frame, lowerblue, upperblue, blue_mask);

		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    	cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

		double blue_area = cv::countNonZero(blue_mask);
		return blue_area > width * height / 1.5;
	};

private:
	int width;
	int height;
};