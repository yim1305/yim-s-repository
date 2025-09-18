#ifndef GPIO_H
#define GPIO_H

#include <pigpio.h>
#include <iostream>

class GPIO{

private:
    int pin;

public:
    // initialize GPIO pin
    GPIO(int pinNumber);

    // turn mosfet on
    void GPIO_On();

    // turn mosfet off
    void GPIO_Off();

    // terminate + clean up
    ~GPIO();
};

#endif
