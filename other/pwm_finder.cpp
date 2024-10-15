#include <pigpio.h>
#include <iostream>
#include <unistd.h>

void moveforward(int pwm_pin) {
    gpioInitialise();
    // 初始化电机
    gpioSetMode(pwm_pin, PI_OUTPUT);
    gpioSetPWMfrequency(pwm_pin, 200);
    gpioSetPWMrange(pwm_pin, 40000);
    std::cout << "电机初始化完成" << std::endl;
    std::cout << "前进!!!" << std::endl;
    for (int i = 12000; i < 13500; i += 100) {
        std::cout << "PWM值:" << i << std::endl;
        gpioPWM(pwm_pin, i);
        usleep(200 * 1000);
    }
}

int main() {
    system("sudo killall pigpiod");
    system("sudo cp /home/pi/.Xauthority /root/");
    int pwm_pin = 13;
    moveforward(pwm_pin);
    gpioTerminate();
    return 0;
}