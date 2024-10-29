#pragma once

#include <pigpio.h>
#include <unistd.h>
#include <string>
#include <atomic>

#include "Logger.hpp"
#include "Struct.hpp"

extern std::atomic<bool> isRunning, cameraOpened;      // isRunning: 控制程序运行, cameraOpened: 控制电机启动

// GPIOHandler: 负责 GPIO 操作
class GPIOHandler {
public:
    GPIOHandler() {
        if (gpioInitialise() < 0) {     // 初始化 GPIO
            Logger::getLogger()->error("GPIO 初始化失败");
            initialized = false;
        }
        else {
            Logger::getLogger()->info("GPIO 初始化成功");
            initialized = true;
        }
        
    }

    ~GPIOHandler() {
        cleanup();
    }

    void cleanup() {
        if (initialized) {
            gpioTerminate();        // 终止 GPIO
            Logger::getLogger()->info("GPIO 资源已释放");
            initialized = false;
        }
    }

    void setMode(int pin, int mode) {
        if (initialized) gpioSetMode(pin, mode);
    }

    void setPWMfrequency(int pin, int frequency) {
        if (initialized) gpioSetPWMfrequency(pin, frequency);
    }

    void setPWMrange(int pin, int range) {
        if (initialized) gpioSetPWMrange(pin, range);
    }

    void setPWM(int pin, int pwm) {
        if (initialized) gpioPWM(pin, pwm);
    }

    void setDelay(int delay) {
        if (initialized) gpioDelay(delay);
    }

private:
    bool initialized = false;
};


// ServoController: 舵机控制器
class ServoController {
public:
    ServoController(GPIOHandler* gpio, int servo_pin, int width);
    void reset();
    double angleToDutyCycle(double angle);
    void setServoAngle(double center);
    void changeLane(int direction);
    void coneDetour(int* detectedCone, double coneCenter, Lane lane);

private:
    void initializeServo() {    // 初始化舵机
        gpio->setMode(servo_pin, PI_OUTPUT);
        gpio->setPWMfrequency(servo_pin, 50);
        gpio->setPWMrange(servo_pin, 100);
        gpio->setPWM(servo_pin, angleToDutyCycle(55));
        gpio->setDelay(1500 * 1000);
        gpio->setPWM(servo_pin, angleToDutyCycle(145));
        gpio->setDelay(1500 * 1000);
        gpio->setPWM(servo_pin, angleToDutyCycle(100));
        gpio->setDelay(1500 * 1000);
    }

    GPIOHandler* gpio;
    int servo_pin;
    double error = 0.0;
    double last_error = 0.0;
    double kp = 0.33;
    double kd = 0.11;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
    int width;
};


// MotorController: 电机控制器
class MotorController {
public:
    MotorController(GPIOHandler* gpio, int motor_pin, int init_pwm, int target_pwm);
    void stop();
    void moveForward(State& state);

private:
    void initializeMotor() {    // 初始化电机
        gpio->setMode(motor_pin, PI_OUTPUT);
        gpio->setPWMfrequency(motor_pin, 200);
        gpio->setPWMrange(motor_pin, 40000);
        gpio->setPWM(motor_pin, init_pwm);
        gpio->setDelay(200);
    }

    GPIOHandler* gpio;
    int motor_pin;
    int init_pwm;
    int target_pwm;
};
