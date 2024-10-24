#pragma once

#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <atomic>

#include "Logger.hpp"

extern std::atomic<bool> isRunning, cameraOpened; // 引入全局变量

// GPIOHandler: 负责 GPIO 操作
class GPIOHandler {
public:
    GPIOHandler() {
        if (gpioInitialise() < 0) {
            Logger::getLogger()->error("GPIO 初始化失败");
            return;
        }
        Logger::getLogger()->info("GPIO 初始化成功");
    }

    void setMode(int pin, int mode) {
        gpioSetMode(pin, mode);
    }

    void setPWMfrequency(int pin, int frequency) {
        gpioSetPWMfrequency(pin, frequency);
    }

    void setPWMrange(int pin, int range) {
        gpioSetPWMrange(pin, range);
    }

    void setPWM(int pin, int pwm) {
        gpioPWM(pin, pwm);
    }

    void setDelay(int delay) {
        gpioDelay(delay);
    }

    ~GPIOHandler() {
        gpioTerminate(); // 终止 GPIO
        Logger::getLogger()->info("GPIO 销毁成功");
    }
};


class ServoController {
public:
    ServoController(GPIOHandler& gpio, int servo_pin) : gpio(gpio), servo_pin(servo_pin) {
        initializeServo();
        Logger::getLogger()->info("ServoController 初始化成功");
    }

    ~ServoController() {
        gpio.setPWM(servo_pin, angleToDutyCycle(100));
        gpio.setDelay(500 * 1000);
        Logger::getLogger()->info("舵机归位, ServoController 销毁成功");
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
        gpio.setPWM(servo_pin, angleToDutyCycle(angle));
        gpio.setDelay(2000);
        gpio.setPWM(servo_pin, angleToDutyCycle(angle));
    }

    // 变道
    void changeLane(int direction, int width) {
        double first_angle, second_angle;
        if (direction == 0) {
            Logger::getLogger()->info("左变道...");
            first_angle = 130;
            second_angle = 70;
        } else if (direction == 1) {
            Logger::getLogger()->info("右变道...");
            first_angle = 70;
            second_angle = 130;
        }
        gpio.setPWM(servo_pin, angleToDutyCycle(first_angle));
        gpio.setDelay(2000 * 1000);
        gpio.setPWM(servo_pin, angleToDutyCycle(100));
        gpio.setDelay(500 * 1000);
        gpio.setPWM(servo_pin, angleToDutyCycle(second_angle));
        gpio.setDelay(500 * 1000);
    }



private:
    void initializeServo() {
        gpio.setMode(servo_pin, PI_OUTPUT);
        gpio.setPWMfrequency(servo_pin, 50);
        gpio.setPWMrange(servo_pin, 100);
        gpio.setPWM(servo_pin, angleToDutyCycle(70));
        gpio.setDelay(1500 * 1000);
        gpio.setPWM(servo_pin, angleToDutyCycle(130));
        gpio.setDelay(1500 * 1000);
        gpio.setPWM(servo_pin, angleToDutyCycle(100));
    }

    GPIOHandler& gpio;
    int servo_pin;
    double error = 0.0;
    double last_error = 0.0;
    double kp = 0.5;
    double kd = 0.11;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
};


class MotorController {
public:
    MotorController(GPIOHandler& gpio, int motor_pin, int init_pwm, int target_pwm) : gpio(gpio), motor_pin(motor_pin), init_pwm(init_pwm), target_pwm(target_pwm) {
        initializeMotor();
        Logger::getLogger()->info("MotorController 初始化成功");
    }

    ~MotorController() {
        gpio.setPWM(motor_pin, init_pwm);
        Logger::getLogger()->info("电机归零, MotorController 销毁成功");
    }

    void moveForward(State& state) {
        int i = init_pwm;
        bool detected_crosswalk = false;
        while (true && isRunning.load()) {
            if (!cameraOpened.load()) { // 摄像头未打开
                usleep(1000);
                continue;
            }
            if (state.has_blueboard.load()) {
                i = init_pwm;
                gpio.setPWM(motor_pin, i);
                gpio.setDelay(200);
                continue;
            }
            if (state.has_crosswalk.load() && !detected_crosswalk) {
                i = init_pwm;
                gpio.setPWM(motor_pin, i);
                gpio.setDelay(4000 * 1000);
                detected_crosswalk = true;
            }
            if (i < target_pwm) {
                i += 100;
            }
            gpio.setPWM(motor_pin, i);
            gpio.setDelay(200);
        }
    }

private:
    void initializeMotor() {
        gpio.setMode(motor_pin, PI_OUTPUT);
        gpio.setPWMfrequency(motor_pin, 200);
        gpio.setPWMrange(motor_pin, 40000);
        gpio.setPWM(motor_pin, init_pwm);
        gpio.setDelay(2000 * 1000);
    }

    GPIOHandler& gpio;
    int motor_pin;
    int init_pwm;
    int target_pwm;
};
