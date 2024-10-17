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
	// 过滤边线
	void filterLines(std::vector<Line>* lines) const;
	// 获取赛道
	void getTrack(std::vector<Line>& lines, Track* track) const;
	// 获取二值化图像
	cv::Mat getBinaryFrame(cv::Mat* frame, cv::Rect ROI, int threshold) const;
	// 阈值调整
	void threshChanger(int white_count, int* threshold, int lines_size) const;
	// 检测
	void detect(cv::Mat* frame, DetectResult* result) const;
	// 检测边线
	std::vector<Line> getLines(cv::Mat* binary) const;
	// 检测人行横道
	bool hasCrosswalk(cv::Mat* binary) const;
	// 检测是否有箭头，及箭头方向
	int getArrow(cv::Mat* frame) const;
private:
	int width;
	int height;
};