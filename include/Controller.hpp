#pragma once

#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <atomic>

#include "Logger.hpp"

extern std::atomic<bool> isRunning, cameraOpened; // 引入全局变量
extern std::atomic<int> mode, direction, detectedCone;
extern std::atomic<double> lane_center;

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


// ServoController: 舵机控制器
class ServoController {
public:
    ServoController(GPIOHandler& gpio, int servo_pin, int width) : gpio(gpio), servo_pin(servo_pin), width(width) {
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

    void setAngle(int width) {
        while (isRunning.load()) {
            if (mode.load() == 0) {
                error = lane_center.load() - width / 2.0;
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
            else if (mode.load() == 1) {
                double first_angle, second_angle;
                if (direction.load() == 0) {
                    Logger::getLogger()->info("左变道...");
                    first_angle = 130;
                    second_angle = 70;
                } else if (direction.load() == 1) {
                    Logger::getLogger()->info("右变道...");
                    first_angle = 70;
                    second_angle = 130;
                }
                gpio.setPWM(servo_pin, angleToDutyCycle(first_angle)); // 舵机转到第一个角度
                gpio.setDelay(2000 * 1000);
                gpio.setPWM(servo_pin, angleToDutyCycle(100)); // 舵机归位
                gpio.setDelay(500 * 1000);
                gpio.setPWM(servo_pin, angleToDutyCycle(second_angle)); // 舵机转到第二个角度
                gpio.setDelay(500 * 1000);
                mode.store(0);
            }
            else if (mode.load() == 2) {
                Logger::getLogger()->info("绕行锥桶...");
                if (detectedCone.load() % 2 == 0) {
                    gpio.setPWM(servo_pin, angleToDutyCycle(120));
                    gpio.setDelay(300 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(80));
                    gpio.setDelay(500 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(105));
                    gpio.setDelay(300 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(100));
                    gpio.setDelay(300 * 1000);
                } else {
                    gpio.setPWM(servo_pin, angleToDutyCycle(80));
                    gpio.setDelay(300 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(120));
                    gpio.setDelay(500 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(105));
                    gpio.setDelay(300 * 1000);
                    gpio.setPWM(servo_pin, angleToDutyCycle(100));
                    gpio.setDelay(300 * 1000);
                }
                detectedCone.store(detectedCone.load() + 1);
                mode.store(0);
            }
        }
    }

private:
    void initializeServo() {
        gpio.setMode(servo_pin, PI_OUTPUT);
        gpio.setPWMfrequency(servo_pin, 50);
        gpio.setPWMrange(servo_pin, 100);
        // gpio.setPWM(servo_pin, angleToDutyCycle(70));
        // gpio.setDelay(1500 * 1000);
        // gpio.setPWM(servo_pin, angleToDutyCycle(130));
        // gpio.setDelay(1500 * 1000);
        gpio.setPWM(servo_pin, angleToDutyCycle(100));
        gpio.setDelay(1500 * 1000);
    }

    GPIOHandler& gpio;
    int servo_pin;
    double error = 0.0;
    double last_error = 0.0;
    double kp = 0.5;
    double kd = 0.11;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
    int width;
};


// MotorController: 电机控制器
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
            if (state.has_blueboard.load()) { // 蓝板停车
                i = init_pwm;
                gpio.setPWM(motor_pin, i);
                gpio.setDelay(200);
                continue;
            }
            if (state.has_crosswalk.load() && !detected_crosswalk) { // 人行横道停车
                i = init_pwm;
                gpio.setPWM(motor_pin, i);
                gpio.setDelay(200 * 1000);
                gpio.setPWM(motor_pin, i - 500);
                gpio.setDelay(800 * 1000);
                gpio.setPWM(motor_pin, i);
                gpio.setDelay(4000 * 1000);
                detected_crosswalk = true;
            }
            if (i < target_pwm) { // 加速
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
        gpio.setDelay(200);
    }

    GPIOHandler& gpio;
    int motor_pin;
    int init_pwm;
    int target_pwm;
};
