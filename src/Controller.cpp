#include "Controller.hpp"

/******************************舵机******************************/
void ServoController::reset() {
    gpio->setPWM(servo_pin, angleToDutyCycle(100));
    gpio->setDelay(200 * 1000);
    Logger::getLogger()->info("舵机归位, ServoController 销毁成功");
}

// 设置舵机的角度
double ServoController::angleToDutyCycle(double angle) {
    return 2.5 + (angle / 180.0) * 10.0;
}

void ServoController::setServoAngle(double center) {
    error = center - width / 2.0;
    double error_angle = kp * error + kd * (error - last_error);
    if (error_angle > angle_outmax) {
        error_angle = angle_outmax;
    } else if (error_angle < angle_outmin) {
        error_angle = angle_outmin;
    }
    double angle = 100 - error_angle;
    Logger::getLogger()->debug("中心偏差: " + std::to_string(error) + ", 舵机角度: " + std::to_string(angle));
    last_error = error;
    gpio->setPWM(servo_pin, angleToDutyCycle(angle));
    gpio->setDelay(200);
    gpio->setPWM(servo_pin, angleToDutyCycle(angle));
}

void ServoController::changeLane(int direction) {
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
    gpio->setPWM(servo_pin, angleToDutyCycle(first_angle));      // 舵机转到第一个角度
    gpio->setDelay(800 * 1000);
    gpio->setPWM(servo_pin, angleToDutyCycle(100));              // 舵机归位
    gpio->setDelay(400 * 1000);
    gpio->setPWM(servo_pin, angleToDutyCycle(second_angle));     // 舵机转到第二个角度
    gpio->setDelay(800 * 1000);
}

void ServoController::coneDetour(int* detectedCone, double coneCenter, Lane lane) {
    Logger::getLogger()->info("检测到锥桶, 开始第" + std::to_string(*detectedCone + 1) + "次绕行...");
    double detourCenter;
    if (*detectedCone % 2 == 0) {   // 第一、三个锥桶右侧绕行
        detourCenter = (coneCenter + lane.right_line.center.x) / 2.0;
    }
    else {                          // 第二个锥桶左侧绕行
        detourCenter = (coneCenter + lane.left_line.center.x) / 2.0;
    }
    Logger::getLogger()->debug("detourCenter: " + std::to_string(detourCenter));
    setServoAngle(detourCenter);
    gpio->setDelay(400 * 1000);
    gpio->setPWM(servo_pin, angleToDutyCycle(100));
    gpio->setDelay(300 * 1000);
    setServoAngle(width - detourCenter);
    gpio->setDelay(400 * 1000);
    (*detectedCone)++;
}

void ServoController::stopToArea(char letter) {
    Logger::getLogger()->info("检测到蓝色区域, 停车到" + std::string(1, letter) + "区域...");
    int first_angle, second_angle;
    if (letter == 'A') {
        first_angle = 130;
        second_angle = 80;
    }
    else if (letter == 'B') {
        first_angle = 80;
        second_angle = 120;
    }
    gpio->setPWM(servo_pin, angleToDutyCycle(first_angle));
    gpio->setDelay(300 * 1000);
    gpio->setPWM(servo_pin, angleToDutyCycle(100));
    gpio->setDelay(200 * 1000);
    // gpio->setPWM(servo_pin, angleToDutyCycle(second_angle));
    // gpio->setDelay(200 * 1000);
}


/******************************电机******************************/
void MotorController::stop() {
    gpio->setPWM(motor_pin, init_pwm);
    gpio->setDelay(200 * 1000);
    Logger::getLogger()->info("电机归零, MotorController 销毁成功");
}

void MotorController::moveForward(State& state) {
    int i = init_pwm;
    bool detected_crosswalk = false;
    while (isRunning.load()) {
        if (!cameraOpened.load()) {                                 // 摄像头未打开
            usleep(1000);
            continue;
        }
        if (state.has_blueboard.load()) {                           // 蓝板停车
            i = init_pwm;
            gpio->setPWM(motor_pin, i);
            gpio->setDelay(200);
            continue;
        }
        if (state.has_crosswalk.load() && !detected_crosswalk) {    // 人行横道停车
            i = init_pwm;
            gpio->setPWM(motor_pin, i);
            gpio->setDelay(200 * 1000);
            gpio->setPWM(motor_pin, i - 300);
            gpio->setDelay(800 * 1000);
            gpio->setPWM(motor_pin, i);
            gpio->setDelay(1000 * 1000);
            detected_crosswalk = true;
            sleep(6);
        }
        if (i < target_pwm) {                                       // 加速
            i += 100;
        }
        gpio->setPWM(motor_pin, i);
        gpio->setDelay(200);
    }
}