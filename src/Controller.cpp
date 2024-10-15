#include "Controller.hpp"
#include <unistd.h>
#include <iostream>

Controller::Controller(int servo_pin, int pwm_pin) {
    if (gpioInitialise() < 0) {
        std::cerr << "初始化失败" << std::endl;
        return;
    }
    this->servo_pin = servo_pin;
    this->pwm_pin = pwm_pin;

    // 初始化舵机
    gpioSetMode(servo_pin, PI_OUTPUT);
    gpioSetPWMfrequency(servo_pin, 50);
    gpioSetPWMrange(servo_pin, 100);
    gpioPWM(servo_pin, angleToDutyCycle(45));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(135));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(90));
    std::cout << "舵机初始化完成" << std::endl;

    // 初始化电机
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    std::cout << "电机初始化完成" << std::endl;
}

void Controller::moveforward() const {
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

    double angle = 90 - error_angle;
    // angle = (angle - 90) * 1.2 + 90;
    std::cout << "舵机角度: " << angle << std::endl;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    sleep(0.005);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

