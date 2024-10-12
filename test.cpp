#include <iostream>
#include <pigpio.h>

int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) {
        std::cout << "GPIO Initialization failed" << std::endl;
        return 1;
    }
    int pwmPin = 13;
    gpioSetMode(pwmPin, PI_OUTPUT);
    gpioSetPWMfrequency(pwmPin, 200);
    gpioSetPWMrange(pwmPin, 40000);
    for (int i = 10000; i <= 12000; i+=100) {
        gpioPWM(pwmPin, i);
        time_sleep(3);
        std::cout << i << std::endl;
    }
    gpioTerminate();
}