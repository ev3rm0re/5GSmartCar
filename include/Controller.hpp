#pragma once

#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <atomic>

#include "Logger.hpp"

extern std::atomic<bool> isRunning; // 引入全局变量

// GPIOHandler: 负责 GPIO 操作
class GPIOHandler {
public:
    GPIOHandler() {};

    void initGPIO(int servo_pin, int motor_pin, int init_pwm, int target_pwm) {
        if (gpioInitialise() < 0) {
            Logger::getLogger()->error("GPIO 初始化失败");
            return;
        }
        this->servo_pin = servo_pin;
        this->motor_pin = motor_pin;
        this->init_pwm = init_pwm;
        this->target_pwm = target_pwm;
        
        initializeServo();
        initializeMotor();

        Logger::getLogger()->info("GPIO 初始化成功");
    }

    // 设置舵机的角度
    double angleToDutyCycle(double angle) {
        return 2.5 + (angle / 180.0) * 10.0;
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
        gpioPWM(servo_pin, angleToDutyCycle(angle));
        gpioDelay(2000);
        gpioPWM(servo_pin, angleToDutyCycle(angle));
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
        gpioPWM(servo_pin, angleToDutyCycle(first_angle));
        gpioDelay(2000 * 1000);
        gpioPWM(servo_pin, angleToDutyCycle(100));
        gpioDelay(500 * 1000);
        gpioPWM(servo_pin, angleToDutyCycle(second_angle));
        gpioDelay(500 * 1000);
    }

    // 向前移动逻辑
    void moveForward(State& state) {
        sleep(5);
        int i = init_pwm;
        bool detected_crosswalk = false;
        while (true && isRunning.load()) {
            if (state.has_blueboard.load()) {
                i = init_pwm;
                gpioPWM(motor_pin, i);
                gpioDelay(200);
                continue;
            }
            if (state.has_crosswalk.load() && !detected_crosswalk) {
                i = init_pwm;
                gpioPWM(motor_pin, i);
                gpioDelay(4000 * 1000);
                detected_crosswalk = true;
            }
            if (i < target_pwm) {
                i += 100;
            }
            gpioPWM(motor_pin, i);
            gpioDelay(200);
        }
    }

    ~GPIOHandler() {
        gpioTerminate(); // 终止 GPIO
        Logger::getLogger()->info("GPIO 销毁成功");
    }

private:
    void initializeServo() {
        gpioSetMode(servo_pin, PI_OUTPUT);
        gpioSetPWMfrequency(servo_pin, 50);
        gpioSetPWMrange(servo_pin, 100);
        gpioPWM(servo_pin, angleToDutyCycle(70));
        gpioDelay(1500 * 1000);
        gpioPWM(servo_pin, angleToDutyCycle(130));
        gpioDelay(1500 * 1000);
        gpioPWM(servo_pin, angleToDutyCycle(100));
    }

    void initializeMotor() {
        gpioSetMode(motor_pin, PI_OUTPUT);
        gpioSetPWMfrequency(motor_pin, 200);
        gpioSetPWMrange(motor_pin, 40000);
        gpioPWM(motor_pin, init_pwm);
        gpioDelay(2000 * 1000);
    }
    int motor_pin;
    int init_pwm;
    int target_pwm;

    int servo_pin;
    double error = 0.0;
    double last_error = 0.0;
    double kp = 0.5;
    double kd = 0.11;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
};