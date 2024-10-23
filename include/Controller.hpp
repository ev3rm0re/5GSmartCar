#pragma once

#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <atomic>

#include "Logger.hpp"


// GPIOHandler: 负责 GPIO 操作
class GPIOHandler {
public:
    GPIOHandler(int servo_pin, int pwm_pin) {
        if (gpioInitialise() < 0) {
            Logger::getLogger()->error("GPIO 初始化失败");
            return;
        }
        this->servo_pin = servo_pin;
        this->pwm_pin = pwm_pin;
        gpioSetMode(servo_pin, PI_OUTPUT);
        gpioSetMode(pwm_pin, PI_OUTPUT);
        Logger::getLogger()->info("GPIO 初始化成功");
    }

    // 设置 PWM 值
    void setPWM(int pin, int value) {
        gpioPWM(pin, value);
    }

    // 设置舵机的角度
    double angleToDutyCycle(double angle) {
        return 2.5 + (angle / 180.0) * 10.0;
    }

    ~GPIOHandler() {
        gpioTerminate(); // 终止 GPIO
        Logger::getLogger()->info("GPIO 终止成功");
    }

private:
    int servo_pin;
    int pwm_pin;
};


// ServoController: 负责舵机操作
class ServoController {
public:
    ServoController(GPIOHandler& gpio, int pin) : gpio(gpio), pin(pin) {
        gpioSetPWMfrequency(pin, 50);
        gpioSetPWMrange(pin, 100);
        gpioPWM(pin, gpio.angleToDutyCycle(70));
        gpioDelay(1500 * 1000);
        gpioPWM(pin, gpio.angleToDutyCycle(130));
        gpioDelay(1500 * 1000);
        gpioPWM(pin, gpio.angleToDutyCycle(100));
        Logger::getLogger()->info("舵机初始化成功");
    }

    void setAngle(double center, int width) {
        error = center - width / 2.0;
        double error_angle = kp * error + kd * (error - last_error);

        if (error_angle > angle_outmax) {
            error_angle = angle_outmax;
        } else if (error_angle < angle_outmin) {
            error_angle = angle_outmin;
        }

        double angle = 100 - error_angle;

        last_error = error;
        gpio.setPWM(pin, gpio.angleToDutyCycle(angle));
        gpioDelay(2000);
        gpio.setPWM(pin, gpio.angleToDutyCycle(angle));
    }

    void changeLane(int direction, int width) {
        double first_angle, second_angle;
        if (direction == 0) {
            Logger::getLogger()->info("向左变道");
            first_angle = 130;
            second_angle = 70;
        } else if (direction == 1) {
            Logger::getLogger()->info("向右变道");
            first_angle = 70;
            second_angle = 130;
        }
        gpio.setPWM(pin, gpio.angleToDutyCycle(first_angle));
        gpioDelay(2000 * 1000);
        gpio.setPWM(pin, gpio.angleToDutyCycle(100));
        gpioDelay(500 * 1000);
        gpio.setPWM(pin, gpio.angleToDutyCycle(second_angle));
        gpioDelay(500 * 1000);
    }

private:
    GPIOHandler& gpio;
    int pin;
    double error;
    double last_error;
    double kp = 0.5;
    double kd = 0.11;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
};


// MotorController: 负责电机控制
class MotorController {
public:
    MotorController(GPIOHandler& gpio, int pin, int init_pwm, int target_pwm)
                    : gpio(gpio), pin(pin), init_pwm(init_pwm), target_pwm(target_pwm) {
        gpioSetPWMfrequency(pin, 200);
        gpioSetPWMrange(pin, 40000);
        gpioPWM(pin, init_pwm);
        gpioDelay(2000 * 1000);
        Logger::getLogger()->info("电机初始化成功");
    }

    // 向前移动逻辑
    void moveForward(State& state) {
        sleep(5);
        int i = init_pwm;
        bool detected_crosswalk = false;
        while (true) {
            if (state.has_blueboard.load()) {
                i = init_pwm;
                gpio.setPWM(pin, i);
                gpioDelay(200);
                continue;
            }
            if (state.has_crosswalk.load() && !detected_crosswalk) {
                i = init_pwm;
                gpio.setPWM(pin, i);
                gpioDelay(4000 * 1000);
                detected_crosswalk = true;
            }
            if (i < target_pwm) {
                i += 100;
            }
            gpio.setPWM(pin, i);
            gpioDelay(100);
        }
    }

private:
    GPIOHandler& gpio;
    int pin;
    int init_pwm;
    int target_pwm;
};