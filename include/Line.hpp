#pragma once

// std
#include <algorithm>
#include <numeric>

// opencv
#include <opencv2/opencv.hpp>

const std::map<int, std::string> directions = {
	{"直行", 0},
	{"左变道", 1},
	{"右变道", 2}
};

// 边线结构体
struct Line : public std::vector<cv::Point> {
	Line() = default;
	explicit Line(const std::vector<cv::Point> fitline) : std::vector<cv::Point>(fitline) {
		std::sort(begin(), end(), [](const cv::Point& a, const cv::Point& b) {
			return a.y < b.y;
			});
		top = front();
		bottom = back();
		center = (top + bottom) / 2.0;
		length = cv::norm(top - bottom);
		slope = (top.y - bottom.y) / (top.x - bottom.x);
	};
	cv::Point2f top, bottom, center;
	double length = 0.0;
	double area = 0.0;
	double slope = 0.0;
};

// 人行横道结构体
struct CrossWalk :public cv::RotatedRect {
	CrossWalk() = default;
	explicit CrossWalk(const cv::RotatedRect& rect) : cv::RotatedRect(rect) {
		cv::Point2f vertices[4];
		rect.points(vertices);
		std::sort(vertices, vertices + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
			return a.y < b.y;
			});

		top = (vertices[0] + vertices[1]) / 2.0;
		bottom = (vertices[2] + vertices[3]) / 2.0;
		center = (top + bottom) / 2.0;
		width = cv::norm(vertices[0] - vertices[1]);
		height = cv::norm(top - bottom);
		area = width * height;
		slope = (top.y - bottom.y) / (top.x - bottom.x);
	};
	cv::Point2f center, top, bottom;
	double width = 0.0;
	double height = 0.0;
	double slope = 0.0;
	double area = 0.0;
};

// 赛道结构体
struct Track {
	Track() = default;
	Track(const Line& l1, const Line& l2) {
		if (l1.center.x < l2.center.x) {
			left_line = l1, right_line = l2;
		}
		else {
			left_line = l2, right_line = l1;
		}
		center = (left_line.center + right_line.center) / 2.0;
		width = cv::norm(left_line.center - right_line.center);
	};
	Line left_line, right_line;
	double width = 0.0;
	cv::Point2f center;
};

// 检测结果结构体
struct DetectResult
{
	cv::Point2f center;
	bool has_crosswalk;
};