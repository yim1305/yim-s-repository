#include "GPIO.h"

GPIO::GPIO(int pinNumber) : pin(pinNumber) {
    if (gpioInitialise() < 0) {
        std::cerr << "initialization failed!" << std::endl;
    } else {
        gpioSetMode(pin, PI_OUTPUT);
    }
}

void GPIO::GPIO_On(){
    gpioWrite(pin, PI_ON);
}

void GPIO::GPIO_Off(){
    gpioWrite(pin, PI_OFF);
}

GPIO::~GPIO(){
    gpioTerminate();
}
