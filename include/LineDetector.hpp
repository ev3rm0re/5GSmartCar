#pragma once

// std
#include <vector>

// 边线结构体
#include "Line.hpp"

class LineDetector {
public:
	LineDetector(const int width, const int height);
	// 判断是否为边线
	bool isLine(Line& line) const;
	// 判断是否为赛道
	bool isTrack(const Track& track) const;
	// 判断是否为人行横道
	bool isCrosswalk(CrossWalk& line) const;
	// 获取赛道
	Track getTrack(std::vector<Line>& lines) const;
	// 获取二值化图像
	cv::Mat getBinaryFrame(cv::Mat* frame, cv::Rect ROI, int threshold) const;
	// 检测
	DetectResult detect(cv::Mat* frame) const;
	// 检测边线
	std::vector<Line> getLines(cv::Mat* frame) const;
	// 检测人行横道
	bool hasCrosswalk(cv::Mat* binary) const;
private:
	int width;
	int height;
};