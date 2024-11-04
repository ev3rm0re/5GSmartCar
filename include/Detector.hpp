#pragma once

// std
#include <vector>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

// 边线结构体
#include "Struct.hpp"
#include "Logger.hpp"

extern YAML::Node config;


// LineDetector: 负责边线检测
class LineDetector {
public:
    LineDetector(const int width, const int height) : width(width), height(height) {};
    bool isLine(Line& line) const;
	std::vector<Line> detectLines(cv::Mat* binary) const;
    void filterLines(std::vector<Line>* lines) const;

private:
	int width;
	int height;
};


// LaneDetector: 负责车道检测
class LaneDetector {
public:
   	LaneDetector(const int width, const int height) : width(width), height(height) {};
    bool isLane(const Lane& lane) const;
	void getLane(std::vector<Line>& lines, Lane* lane) const;
	// TODO: Lane筛选

private:
    int width;
    int height;
};


// CrosswalkDetector: 负责人行横道检测
class CrosswalkDetector {
public:
    CrosswalkDetector(int width, int height) : width(width), height(height) {};
    bool isCrosswalk(CrossWalk& crosswalk) const;
    bool hasCrosswalk(cv::Mat* binary) const;

private:
    int width;
    int height;
};


// BinaryImageProcessor: 负责二值化图像处理
class BinaryImageProcessor {
public:
    BinaryImageProcessor(int width, int height) : width(width), height(height) {};
    cv::Mat getBinaryFrame(cv::Mat* frame, cv::Rect ROI, int threshold) const;
    void adjustThreshold(int white_count, int* threshold, int lines_size) const;

private:
    int width;
    int height;
};


// ArrowProcessor: 负责箭头检测
class ArrowProcessor {
public:
    ArrowProcessor(const std::string& modelPath) : onnxModelPath(modelPath) {};
    int detectArrowDirection(cv::Mat* frame) const;

private:
    std::string onnxModelPath;
};


// BlueBoardDetector: 负责蓝色板检测
class BlueBoardDetector {
public:
	BlueBoardDetector(int width, int height) : width(width), height(height) {};

	bool hasBlueBoard(cv::Mat* frame) const;

private:
	int width;
	int height;
};


// ConeDetector: 负责锥桶检测
class ConeDetector {
public:
	ConeDetector(int width, int height) : width(width), height(height) {
        this->upperblue = cv::Scalar(config["conedetector"]["upperblue"]["H"].as<int>(),
                                      config["conedetector"]["upperblue"]["S"].as<int>(),
                                      config["conedetector"]["upperblue"]["V"].as<int>());
        this->lowerblue = cv::Scalar(config["conedetector"]["lowerblue"]["H"].as<int>(),
                                      config["conedetector"]["lowerblue"]["S"].as<int>(),
                                      config["conedetector"]["lowerblue"]["V"].as<int>());
    };
	bool hasCone(cv::Mat* frame, double* center) const;

private:
	int width;
	int height;
    cv::Scalar upperblue;
    cv::Scalar lowerblue;
};