#include "Controller.hpp"
#include <unistd.h>
#include <iostream>
#include <atomic>

Controller::Controller(int servo_pin, int pwm_pin, int init_pwm, int target_pwm) {
    if (gpioInitialise() < 0) {
        std::cerr << "初始化失败" << std::endl;
        return;
    }
    this->servo_pin = servo_pin;
    this->pwm_pin = pwm_pin;
    this->init_pwm = init_pwm;
    this->target_pwm = target_pwm;

    // 初始化舵机
    gpioSetMode(servo_pin, PI_OUTPUT);
    gpioSetPWMfrequency(servo_pin, 50);
    gpioSetPWMrange(servo_pin, 100);
    gpioPWM(servo_pin, angleToDutyCycle(80));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(140));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(110));
    std::cout << "舵机初始化完成" << std::endl;

    // 初始化电机
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    gpioPWM(pwm_pin, init_pwm);
    sleep(2);
    std::cout << "电机初始化完成" << std::endl;
}

void Controller::moveforward(std::atomic<bool>& has_crosswalk, std::atomic<bool>& has_blueboard) const {
    std::cout << "前进!!!" << std::endl;
    int i = init_pwm;
    bool start = false;
    bool detected_crosswalk = false; // 是否已经检测过人行横道
    while (true) {
        if (has_blueboard.load(std::memory_order_acquire) == true) {
            i = init_pwm;
            gpioPWM(pwm_pin, i);
            usleep(200 * 1000);
            continue;
        }
        if (has_crosswalk.load(std::memory_order_acquire) == true && detected_crosswalk == false) {
            i = init_pwm;
            gpioPWM(pwm_pin, i);
            // system("aplay /home/pi/Code/5G_ws/medias/dz-banmaxian.wav");
            sleep(3);
            has_crosswalk.store(false, std::memory_order_release);
            detected_crosswalk = true;
        }
        if (i < target_pwm && (start == false || detected_crosswalk == true)) {
            i += 100;
        };
        std::cout << "pwm: " << i << std::endl;
        gpioPWM(pwm_pin, i);
        if (i == target_pwm) start = true;
        usleep(200 * 1000);
    }
}

double Controller::angleToDutyCycle(double angle) const {
    return 2.5 + (angle / 180.0) * 10.0;
}

void Controller::pidControl(double center, int width) const {
    error = center - width / 2;
    double error_angle = kp * error + kd * (error - last_error);

    if (error_angle > angle_outmax) {
        error_angle = angle_outmax;
    } else if (error_angle < angle_outmin) {
        error_angle = angle_outmin;
    }

    double angle = 115 - error_angle;
    // angle = (angle - 90) * 1.2 + 90;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    usleep(2 * 1000);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

void Controller::changeDirection(int direction, int width) const {
    if (direction == 0) {
        // 左变道
        std::cout << "左变道" << std::endl;
        gpioPWM(servo_pin, angleToDutyCycle(140));
        sleep(1);
        gpioPWM(servo_pin, angleToDutyCycle(110));
        sleep(1);
        gpioPWM(servo_pin, angleToDutyCycle(80));
    }
    else if (direction == 1) {
        // 右变道
        std::cout << "右变道" << std::endl;
        gpioPWM(servo_pin, angleToDutyCycle(80));
        sleep(1);
        gpioPWM(servo_pin, angleToDutyCycle(110));
        sleep(1);
        gpioPWM(servo_pin, angleToDutyCycle(140));
    }
}