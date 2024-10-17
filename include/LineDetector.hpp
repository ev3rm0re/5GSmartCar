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
	// 检测赛道
	DetectResult detect(cv::Mat* frame) const;
	//// 检测人行横道
	//bool crosswalkDetect(cv::Mat* frame) const;
private:
	int width;
	int height;
};