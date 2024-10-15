#pragma once

// std
#include <algorithm>

// opencv
#include <opencv2/opencv.hpp>

// 边线结构体
struct Line : public cv::RotatedRect {
	Line() = default;
	explicit Line(const cv::RotatedRect& rect) : cv::RotatedRect(rect) {
		cv::Point2f p[4];
		rect.points(p);
		std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
			return a.y < b.y;
			});
		top = (p[0] + p[1]) / 2;
		bottom = (p[2] + p[3]) / 2;
		length = cv::norm(top - bottom);
		width = cv::norm(p[0] - p[1]);
		area = length * width;
		angle = 180 - std::atan2(bottom.y - top.y, bottom.x - top.x) * 180 / CV_PI;
	};
	cv::Point2f top, bottom;
	double length = 0.0;
	double width = 0.0;
	double area = 0.0;
	double angle = 0.0;
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
	double width;
	cv::Point2f center;
};