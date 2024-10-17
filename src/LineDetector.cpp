#include <opencv2/imgproc.hpp>

#include "Line.hpp"
#include "LineDetector.hpp"

LineDetector::LineDetector(const int width, const int height) {
	this->width = width;
	this->height = height;
}

// 判断是否为边线
bool LineDetector::isLine(Line& line) const {
	return line.length / line.width >= 2 && line.area > width * height / 200.0 && 
		std::abs(line.angle - 90.0) < 80.0 && std::abs(line.center.x - width / 2.0) >= width / 4.0;
}

// 判断是否为赛道
bool LineDetector::isTrack(const Track& track) const {
	return track.right_line.center.x - track.left_line.center.x > width / 4.0 && 
		std::abs(track.left_line.center.y - track.right_line.center.y) < height / 2.0;
}

// 判断是否为人行横道
bool LineDetector::isCrosswalk(Line& line) const {
	return line.length / line.width >= 0.2 && line.length / line.width <= 2.0 && 
	line.area > width * height / 400.0 && std::abs(line.angle  - 90.0) < 75.0 && 
	std::abs(line.center.x - width / 3.0) < width * 2.0 / 9.0;
}

// 获取赛道
std::vector<Track> LineDetector::getTrack(const std::vector<Line>& lines) const {
	std::vector<Track> tracks;
	for (int i = 0; i < lines.size() - 1; i++) {
		for (int j = i + 1; j < lines.size(); j++) {
			Track track(lines[i], lines[j]);
			if (isTrack(track)) {
				tracks.push_back(track);
			}
		}
	}
	return tracks;
}

// 边线检测
cv::Point2f LineDetector::detect(cv::Mat* frame) const {
	static int threshold = 160;
	
	// 提取ROI
	cv::Mat roi_frame;
	roi_frame = (*frame)(cv::Rect(0, height * 3.0 / 5.0, width, height * 2.0 / 5.0));
	
	// 灰度化
	cv::Mat gray_frame;
	cv::cvtColor(roi_frame, gray_frame, cv::COLOR_BGR2GRAY);

	// 高斯滤波
	cv::Mat gauss_frame;
	cv::GaussianBlur(gray_frame, gauss_frame, cv::Size(3, 3), 0);
	cv::imshow("line_gauss", gauss_frame);

	// 直方图均衡化
	// cv::Mat equalized_frame;
	// cv::equalizeHist(perspective_frame, equalized_frame);
	// cv::imshow("equalized_frame", equalized_frame);

	// 二值化
	cv::Mat binary_frame;
	cv::threshold(gauss_frame, binary_frame, threshold, 255, cv::THRESH_BINARY_INV);
	cv::imshow("binary_frame", binary_frame);

	// 查找轮廓
	std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// 筛选边线
	std::vector<Line> lines;
	for (const auto& contour : contours) {
		if (cv::contourArea(contour) < width * height / 400.0) continue;
		cv::RotatedRect rect = cv::minAreaRect(contour);
		Line line(rect);
		if (isLine(line)) {
			lines.push_back(line);
			cv::line(roi_frame, line.top, line.bottom, cv::Scalar(255, 0, 0), 2);
		}
	}
	
	int current_threshold = threshold;

	// 计算二值化图像白色区域的个数
	int white_count = cv::countNonZero(binary_frame);
	if (white_count < width * height / 80.0 && lines.size() == 0) {
		threshold += 2;
	}
	if (white_count < width * height / 150.0 && lines.size() == 1) {
		threshold++;
	}
	if (white_count > width * height / 20.0) {
		threshold--;
	}

	if (current_threshold != threshold) {
		std::cout << "边线白色区域: " << white_count << std::endl;
		std::cout << "边线检测阈值更新为: " << threshold << std::endl;
	}

	if (lines.size() == 0) {
		return cv::Point2f(width / 2, height / 2);
	}

	// 筛选轨道
	if (lines.size() == 1) {
		if (lines[0].center.x < width / 2.0) {
			lines.push_back(Line(cv::RotatedRect(cv::Point2f(width, lines[0].center.y), lines[0].size, 180 - lines[0].angle))); //这里角度为什么是这样的？
		}
		else {
			lines.push_back(Line(cv::RotatedRect(cv::Point2f(0, lines[0].center.y), lines[0].size, lines[0].angle))); // 还有这里
		}
	}

	std::vector<Track> tracks;
 	if (lines.size() >= 2) {
		tracks = getTrack(lines);
	}

	if (tracks.empty()) {
		return cv::Point2f(width / 2, height / 2);
	}

	// 有多个赛道时筛选赛道
	int index = 0;
	if (tracks.size() > 1) {
		int width = tracks[0].width;
		for (size_t i = 1; i < tracks.size(); i++) {
			if (tracks[i].width > width) {
				index = i;
			}
		}
	}

	// 绘制赛道
	cv::line(roi_frame, tracks[index].left_line.top, tracks[index].left_line.bottom, cv::Scalar(0, 255, 0), 2);
	cv::line(roi_frame, tracks[index].right_line.top, tracks[index].right_line.bottom, cv::Scalar(0, 255, 0), 2);
	cv::circle(roi_frame, tracks[index].center, 5, cv::Scalar(0, 255, 0), -1);

	return tracks[index].center;
}

bool LineDetector::crosswalkDetect(cv::Mat* frame) const {
	static int threshold = 160;
	int cross_count = 0;
	
	cv::Mat frame_copy = frame->clone();

	// 提取ROI
	cv::Mat roi_frame;
	roi_frame = frame_copy(cv::Rect(width / 6.0, height * 3.0 / 5.0, width * 4.0 / 6.0, height * 2.0 / 5.0));

	// 灰度化
	cv::Mat gray_frame;
	cv::cvtColor(roi_frame, gray_frame, cv::COLOR_BGR2GRAY);

	// 高斯滤波
	cv::Mat gauss_frame;
	cv::GaussianBlur(gray_frame, gauss_frame, cv::Size(3, 3), 0);
	cv::imshow("cross_gauss_frame", gauss_frame);

	// 二值化
	cv::Mat binary_frame;
	cv::threshold(gauss_frame, binary_frame, threshold, 255, cv::THRESH_BINARY_INV);
	cv::imshow("cross_binary_frame", binary_frame);

	// 查找轮廓
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binary_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// 筛选人行横道线
	std::vector<Line> crosswalks;
	for (const auto& contour : contours) {
		if (cv::contourArea(contour) < width * height / 400.0) continue;
		cv::RotatedRect rect = cv::minAreaRect(contour);
		Line crosswalk(rect);
		if (isCrosswalk(crosswalk)) {
			cv::line(roi_frame, crosswalk.top, crosswalk.bottom, cv::Scalar(0, 0, 255), 2);
			crosswalks.push_back(crosswalk);
			cross_count++;
		}
	}

	int current_threshold = threshold;

	// 计算二值化图像白色区域的个数
	int white_count = cv::countNonZero(binary_frame);
	if (white_count < width * height / 100.0) {
		threshold += 2;
	}
	if (white_count < width * height / 100 && crosswalks.size() == 1) {
		threshold++;
	}
	if (white_count > width * height / 20.0) {
		threshold--;
	}

	if (current_threshold != threshold) {
		std::cout << "人行横道白色区域: " << white_count << std::endl;
		std::cout << "人行横道检测阈值更新为: " << threshold << std::endl;
	}

	// cv::imshow("cross_frame", roi_frame);

	return cross_count > 1 ? true : false;
}