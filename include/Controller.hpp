#pragma once

#include <pigpio.h>
#include <string>
#include <atomic>

class Controller {
public:
    Controller(int servo_pin, int pwm_pin, int init_pwm, int target_pwm);
    void moveforward(std::atomic<bool>& flag, std::atomic<bool>& has_blueboard) const;
    double angleToDutyCycle(double angle) const;
    void pidControl(double center, int width) const;
    void changeDirection(int direction, int width) const;
private:
    int init_pwm;
    int target_pwm;
    int servo_pin;
    int pwm_pin;
    double kp = 0.5;
    double kd = 0.11;
    mutable double last_error = 0.0;
    mutable double error = 0.0;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
};