#pragma once

#include <pigpio.h>
#include <string>

class Controller {
public:
    Controller(int servo_pin, int pwm_pin);
    void initServo() const;
    void initMotor() const;
    void moveforward() const;
    double angleToDutyCycle(double angle) const;
    void pidControl(double center, int width) const;
private:
    int servo_pin;
    int pwm_pin;
    double kp = 0.5;
    double kd = 0.11;
    mutable double last_error = 0.0;
    mutable double error = 0.0;
    double angle_outmax = 45.0;
    double angle_outmin = -45.0;
};