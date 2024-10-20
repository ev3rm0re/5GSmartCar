#include "Controller.hpp"
#include <unistd.h>
#include <iostream>
#include <atomic>

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
    gpioPWM(servo_pin, angleToDutyCycle(70));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(130));
    sleep(1);
    gpioPWM(servo_pin, angleToDutyCycle(100));
    std::cout << "舵机初始化完成" << std::endl;

    // 初始化电机
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    // gpioPWM(pwm_pin, 12800);
    // sleep(2);
    std::cout << "电机初始化完成" << std::endl;
}

void Controller::moveforward(std::atomic<bool>& flag) const {
    std::cout << "前进!!!" << std::endl;
    sleep(5);
    int i = 12800;
    int start = 0;
    int detected_crosswalk = 0;
    while (true) {
        if (flag.load(std::memory_order_acquire) == true && detected_crosswalk == 0) {
            i = 12800;
            gpioPWM(pwm_pin, i);
            system("aplay /home/pi/Code/5G_ws/medias/dz-banmaxian.wav");
            sleep(6);
            flag.store(false, std::memory_order_release);
            detected_crosswalk = 1;
        }
        if (i != 13000 && (start == 0 || detected_crosswalk == 1)) {
            i += 100;
        };
        gpioPWM(pwm_pin, i);
        if (i == 13000) start = 1;
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

    double angle = 100 - error_angle;
    // angle = (angle - 90) * 1.2 + 90;
    last_error = error;
    gpioPWM(servo_pin, angleToDutyCycle(angle));
    usleep(2 * 1000);
    gpioPWM(servo_pin, angleToDutyCycle(angle));
}

