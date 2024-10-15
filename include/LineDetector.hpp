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
	// 判断是否为人行横道
	bool isCrosswalk(Line& line) const;
	// 判断是否为赛道
	bool isTrack(const Track& track) const;
	// 获取赛道
	std::vector<Track> getTrack(const std::vector<Line>& lines) const;
	// 检测赛道
	cv::Point2f detect(cv::Mat* frame) const;
	bool crosswalkDetect(cv::Mat* frame) const;
private:
	int width;
	int height;
};