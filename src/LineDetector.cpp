#include "Line.hpp"
#include "LineDetector.hpp"

LineDetector::LineDetector(const int width, const int height) {
	this->width = width;
	this->height = height;
}

// 判断是否为边线
bool LineDetector::isLine(Line& line) const {
	return std::abs(line.slope) > 0.2 && line.length > height / 4.0;
}

// 判断是否为赛道
bool LineDetector::isTrack(const Track& track) const {
	return track.width > width / 4.0 && track.center.x > width / 4.0 && track.center.x < width * 3 / 4.0 &&
		std::abs(track.left_line.center.y - track.right_line.center.y) < height / 3.0;
}

// 判断是否为人行横道
bool LineDetector::isCrosswalk(CrossWalk& crosswalk) const {
	return std::abs(crosswalk.slope) > 0.8 && crosswalk.area > width * height / 300.0 && 
		crosswalk.height / crosswalk.width > 1 && crosswalk.height / crosswalk.width < 4;
}

// 过滤边线
void LineDetector::filterLines(std::vector<Line>* lines) const {
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
}

// 获取赛道
void LineDetector::getTrack(std::vector<Line>& lines, Track* track) const {
	for (size_t i = 0; i < lines.size(); i++) {
		for (size_t j = i + 1; j < lines.size(); j++) {
			Track track_(lines[i], lines[j]);
			if (isTrack(track_)) {
				*track = track_;
				return;
			}
		}
	}
}

// 获取二值化图像
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

// 检测人行横道
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

// 检测边线
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

// 阈值调整
void LineDetector::threshChanger(int white_count, int* threshold, int lines_size) const {
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
		std::cout << "阈值更新为: " << *threshold << std::endl;
	}
}

// 检测是否有箭头，及箭头方向
int LineDetector::getArrow(cv::Mat* frame) const {
	return 0;
}

// 边线检测
void LineDetector::detect(cv::Mat* frame, DetectResult* result) const {
	static int threshold = 160;
	
	cv::Rect line_roi = cv::Rect(0, height / 2.0, width, height / 2.0);
	cv::Rect crosswalk_roi = cv::Rect(width / 4.0, height / 2.0, width / 2.0, height / 2.0);

	cv::Mat binary = getBinaryFrame(frame, line_roi, threshold);
	cv::Mat binary_c = getBinaryFrame(frame, crosswalk_roi, threshold);
	cv::imshow("binary", binary);

	bool has_crosswalk = hasCrosswalk(&binary_c);

	result->center = cv::Point2f(width / 2.0, height / 2.0);
	result->has_crosswalk = has_crosswalk;

	std::vector<Line> lines = getLines(&binary);
	filterLines(&lines);
	threshChanger(cv::countNonZero(binary), &threshold, lines.size());
	for (const auto& line : lines) {
		cv::circle(*frame, line.center + cv::Point2f(0, height / 2.0), 5, cv::Scalar(255, 0, 0), -1);
		// cv::line(*frame, line.top + cv::Point2f(0, height / 2.0), line.bottom + cv::Point2f(0, height / 2.0), cv::Scalar(255, 0, 0), 2);
	}

	Track track;
	getTrack(lines, &track);
	if (track.width == 0) {
		std::cout << "未检测到赛道" << std::endl;
		return;
	}
	result->center = track.center;
	// std::cout << "赛道宽度: " << track.width << " 赛道中心: " << track.center << std::endl;
	cv::line(*frame, track.left_line.center + cv::Point2f(0, height / 2.0), track.right_line.center + cv::Point2f(0, height / 2.0), cv::Scalar(0, 255, 0), 2);
	cv::circle(*frame, track.center + cv::Point2f(0, height / 2.0), 5, cv::Scalar(0, 255, 0), -1);
}