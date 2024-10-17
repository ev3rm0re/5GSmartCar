#include "Line.hpp"
#include "LineDetector.hpp"

LineDetector::LineDetector(const int width, const int height) {
	this->width = width;
	this->height = height;
}

// 判断是否为边线
bool LineDetector::isLine(Line& line) const {
	line.top.y = line.top.y + height / 3.0;
	line.bottom.y = line.bottom.y + height / 3.0;
	line.center.y = line.center.y + height / 3.0;
	return std::abs(line.slope) > std::sqrt(3) / 3.0 && line.length > height / 3.0;
}

// 判断是否为赛道
bool LineDetector::isTrack(const Track& track) const {
	return track.right_line.center.x - track.left_line.center.x > width / 4.0 &&
		std::abs(track.left_line.center.y - track.right_line.center.y) < height / 3.0;
}

// 判断是否为人行横道
bool LineDetector::isCrosswalk(CrossWalk& crosswalk) const {
	crosswalk.top.x = crosswalk.top.x + width / 3.0;
	crosswalk.bottom.x = crosswalk.bottom.x + width / 3.0;
	crosswalk.center.x = crosswalk.center.x + width / 3.0;
	crosswalk.top.y = crosswalk.top.y + height / 4.0;
	crosswalk.bottom.y = crosswalk.bottom.y + height / 4.0;
	crosswalk.center.y = crosswalk.center.y + height / 4.0;
	return std::abs(crosswalk.slope) > 0.8 && crosswalk.area > width * height / 300.0 && 
		crosswalk.height / crosswalk.width > 1 && crosswalk.height / crosswalk.width < 4;
}

// 获取赛道
Track LineDetector::getTrack(std::vector<Line>& lines) const {
	std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
		return a.center.x < b.center.x;
		});
	lines.front().top.x = (0 - lines.front().top.y) / lines.front().slope + lines.front().top.x;
	lines.front().top.y = 0;
	lines.front().bottom.x = (height - lines.front().bottom.y) / lines.front().slope + lines.front().bottom.x;
	lines.front().bottom.y = height;
	lines.back().top.x = (0 - lines.back().top.y) / lines.back().slope + lines.back().top.x;
	lines.back().top.y = 0;
	lines.back().bottom.x = (height - lines.back().bottom.y) / lines.back().slope + lines.back().bottom.x;
	lines.back().bottom.y = height;

	Track track(lines.front(), lines.back());
	return track;
}

cv::Mat LineDetector::getBinaryFrame(cv::Mat* frame, cv::Rect ROI, int threshold) const {
	cv::Mat frame_copy = frame->clone();
	cv::Mat roi = frame_copy(ROI);
	cv::Mat gray;
	cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
	cv::Mat binary;
	cv::threshold(gray, binary, threshold, 255, cv::THRESH_BINARY);
	return binary;
}

bool LineDetector::hasCrosswalk(cv::Mat* binary) const {
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

	if (crosswalks.size() > 1) {
		has_crosswalk = true;
	}
	return has_crosswalk;
}

std::vector<Line> LineDetector::getLines(cv::Mat* binary) const {
	// 查找边线
	cv::Mat canny;
	cv::Canny(*binary, canny, 50, 150);

	std::vector<cv::Vec4i> linesP;
	cv::HoughLinesP(canny, linesP, 1, CV_PI / 180, 50, 50, 10);

	std::vector<Line> lines;
	for (const auto& lineP : linesP) {
		Line line({ cv::Point(lineP[0], lineP[1]), cv::Point(lineP[2], lineP[3]) });
		if (!isLine(line)) continue;
		lines.push_back(line);
	}
	return lines;
}

int LineDetector::getArrow(cv::Mat* frame) const {

}

// 边线检测
DetectResult LineDetector::detect(cv::Mat* frame) const {
	DetectResult result;

	static int threshold = 160;
	

	cv::Rect line_roi = cv::Rect(0, height / 3.0, width, height * 2.0 / 3.0);
	cv::Rect crosswalk_roi = cv::Rect(width / 4.0, height / 4.0, width / 2.0, height * 3.0 / 4.0);

	cv::Mat binary = getBinaryFrame(frame, line_roi, threshold);
	cv::Mat binary_c = getBinaryFrame(frame, crosswalk_roi, threshold);

	bool has_crosswalk = hasCrosswalk(&binary_c);

	std::vector<Line> lines = getLines(&binary);

	// 改变阈值
	int current_threshold = threshold;
	int white_count = cv::countNonZero(binary);
	if (white_count < width * height / 80.0 && lines.size() == 0) {
		threshold -= 2;
	}
	else if (white_count < width * height / 100 && lines.size() == 1) {
		threshold--;
	}
	else if (white_count > width * height / 15.0) {
		threshold++;
	}

	if (current_threshold != threshold) {
		std::cout << "车道线阈值更新为: " << threshold << std::endl;
	}

	if (lines.size() == 0) {
		result = { cv::Point2f(width / 2, height / 2), has_crosswalk };
		return result;
	}

	// 筛选轨道
	if (lines.size() == 1) {
		if (lines[0].center.x < width / 2.0) {
			lines.push_back(Line({ cv::Point(width, lines[0].center.y), cv::Point(width - lines[0].top.x, lines[0].top.y) }));
		}
		else {
			lines.push_back(Line({ cv::Point(0, lines[0].center.y), cv::Point(width - lines[0].top.x, lines[0].top.y) }));
		}
	}

	Track track;
	if (lines.size() >= 2) {
		track = getTrack(lines);
	}

	result = { track.center, has_crosswalk };

	// 绘制赛道
	cv::fillPoly((*frame), std::vector<std::vector<cv::Point>>{{
			track.left_line.top,
				track.right_line.top,
				track.right_line.bottom,
				track.left_line.bottom
		}}, cv::Scalar(255, 0, 0, 0.1));
	cv::line((*frame), track.left_line.top, track.left_line.bottom, cv::Scalar(0, 0, 255), 3);
	cv::line((*frame), track.right_line.top, track.right_line.bottom, cv::Scalar(0, 0, 255), 3);
	cv::circle((*frame), track.center, 5, cv::Scalar(0, 255, 0), -1);
	return result;
}