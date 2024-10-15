#include <iostream>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <thread>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <chrono>

const int width = 300;
const int height = 200;

// 检测代码
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
	};
	Line left_line, right_line;
	cv::Point2f center;
};

// 判断是否为边线
static bool isLine(Line& line) {
	line.center.y = line.center.y + height * 2.0 / 3.0;
	line.top.y = line.top.y + height * 2.0 / 3.0;
	line.bottom.y = line.bottom.y + height * 2.0 / 3.0;
	return line.length / line.width >= 2 && line.area > width * height / 200.0;
			// (line.center.x < width * 4 / 9.0 || line.center.x > width * 5 / 9.0);
}

// 判断是否为赛道
static bool isTrack(const Track& track) {
	return track.right_line.center.x - track.left_line.center.x > width / 4.0 && 
		std::abs(track.left_line.center.y - track.right_line.center.y) < height / 2.0;
}

// 获取赛道
static std::vector<Track> getTrack(const std::vector<Line>& lines) {
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
static cv::Point2f lineDetect(cv::Mat* frame) {
	static int threshold = 140;
	
	// 提取ROI
	cv::Mat roi_frame;
	roi_frame = (*frame)(cv::Rect(0, height * 2.0 / 3.0, width, height / 3.0));

	// 灰度化
	cv::Mat gray_frame;
	cv::cvtColor(roi_frame, gray_frame, cv::COLOR_BGR2GRAY);
	// cv::imshow("gray_frame", gray_frame);

	// 直方图均衡化
	// cv::Mat equalized_frame;
	// cv::equalizeHist(perspective_frame, equalized_frame);
	// cv::imshow("equalized_frame", equalized_frame);

	// 二值化
	cv::Mat binary_frame;
	cv::threshold(gray_frame, binary_frame, threshold, 255, cv::THRESH_BINARY_INV);
	// cv::bitwise_and(binary_frame, mask, binary_frame);
	cv::imshow("binary_frame", binary_frame);

	// 开运算
	// cv::Mat morphology_frame;
	// cv::Mat	kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// cv::morphologyEx(binary_frame, morphology_frame, cv::MORPH_OPEN, kernel);
	// cv::imshow("blackhat_frame", morphology_frame);

	// 查找轮廓
	std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	// 筛选边线
	std::vector<Line> lines;
	for (const auto& contour : contours) {
		cv::RotatedRect rect = cv::minAreaRect(contour);
		Line line(rect);
		// cv::line((*frame), line.top, line.bottom, cv::Scalar(0, 0, 255), 2);
		// cv::putText((*frame), std::to_string(line.area), line.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
		// cv::putText((*frame), std::to_string(line.angle), line.center + cv::Point2f(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
		if (isLine(line)) {
			lines.push_back(line);
		}
	}

	// 计算二值化图像白色区域的个数
	int white_count = cv::countNonZero(binary_frame);
	if (white_count < width * height / 80.0 && lines.size() == 0) {
		threshold += 2;
	}
	else if (white_count < width * height / 100 && lines.size() == 1) {
		threshold++;
	}
	else if (white_count > width * height / 30.0 || lines.size() > 2) {
		threshold -= 2;
	}
	std::cout << "阈值: " << threshold << std::endl;

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

	std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
		return a.center.x > b.center.x; });

	std::vector<Track> tracks;
 	if (lines.size() >= 2) {
		tracks = getTrack(lines);
	}
	
	// 绘制赛道
	for (const auto& track : tracks) {
		cv::line((*frame), track.left_line.top, track.left_line.bottom, cv::Scalar(0, 255, 0), 2);
		cv::line((*frame), track.right_line.top, track.right_line.bottom, cv::Scalar(0, 255, 0), 2);
		cv::line((*frame), track.left_line.center, track.right_line.center, cv::Scalar(0, 255, 0), 2);
		cv::putText((*frame), std::to_string(track.left_line.area), track.left_line.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
		cv::putText((*frame), std::to_string(track.right_line.area), track.right_line.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
		cv::circle((*frame), track.center, 5, cv::Scalar(0, 255, 0), -1);
	}

	// TODO: 有多个赛道时筛选赛道

	return tracks.empty() ? cv::Point2f(width / 2, height / 2) : tracks[0].center;
}

// 控制代码
// 前进
void moveforward(int pwm_pin) {
	std::cout << "前进!!!" << std::endl;
	sleep(5);
    int i = 12000;
	int start = 0;
    while (true) {
		if (i != 12700 && start == 0) {
			i += 100;
		};
        gpioPWM(pwm_pin, i);
		std::cout << "PWM值:" << i << std::endl;
		if (i == 12700) {
			start = 1;
			i = 12600;
		}
        sleep(3);
    }
}

double angleToDutyCycle(double angle) {
    return 2.5 + (angle / 180.0) * 10.0;
}

void initServo(int servo_pin) {
    gpioSetPWMfrequency(servo_pin, 50);
    gpioSetPWMrange(servo_pin, 100);
    // gpioPWM(servo_pin, angleToDutyCycle(45));
    // sleep(2);
    // gpioPWM(servo_pin, angleToDutyCycle(135));
    // sleep(2);
    gpioPWM(servo_pin, angleToDutyCycle(90));
    std::cout << "舵机初始化完成" << std::endl;
}

void initMotor(int pwm_pin) {
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    // gpioPWM(pwm_pin, 10000);
    // sleep(2);
    std::cout << "电机初始化完成" << std::endl;
}

void pidControl(double center, int servo_pin) {
    static double kp = 0.5;
    static double kd = 0.11;
    static double last_error = 0;
    static double error = 0;

    double angle_outmax = 45;
    double angle_outmin = -45;

    error = center - width / 2;
    double error_angle = kp * error + kd * (error - last_error);

    if (error_angle > angle_outmax) {
        error_angle = angle_outmax;
    } else if (error_angle < angle_outmin) {
        error_angle = angle_outmin;
    }

    double angle = 90 - error_angle;
    // angle = (angle - 90) * 1.2 + 90;
    std::cout << "舵机角度: " << angle << std::endl;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    sleep(0.005);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

void videoProcessing(int servo_pin) {
	// const std::string video_path = "/home/pi/5G_ws/output238.avi";
	cv::VideoCapture cap(0, cv::CAP_V4L2);
	if (!cap.isOpened()) {
		std::cerr << "打开失败" << std::endl;
		return;
	}
	while (cap.isOpened()) {
		cv::Mat frame;
		cap >> frame;
		if (frame.empty()) break;
		cv::resize(frame, frame, cv::Size(width, height));
		cv::Point2f center = lineDetect(&frame);
		pidControl(center.x, servo_pin);
		cv::imshow("frame", frame);
		int key = cv::waitKey(1);
		if (key == 27) break;
	}
	cap.release();
	cv::destroyAllWindows();
}


int main() {
	system("sudo killall pigpiod");
  	system("sudo cp /home/pi/.Xauthority /root/");
  	sleep(2);
  	int servo_pin = 12;
  	int pwm_pin = 13;
  	int pi = gpioInitialise();
  	if (pi < 0) {
  	    std::cerr << "pigpio 初始化失败" << std::endl;
  	    return -1;
  	}

	initServo(servo_pin);
	initMotor(pwm_pin);
	sleep(1);

	std::thread t1(videoProcessing, servo_pin);
	std::thread t2(moveforward, pwm_pin);
	try {
		t1.join();
		t2.join();
	} catch (const std::exception& e) {
		std::cerr << "Exception: " << e.what() << std::endl;
	}

	gpioTerminate();
	return 0;
}