#pragma once
/* ------------------------------------------------------------------------------------------
 * Base i2c class:
 *  This class will allow access to the i2c file to communicate with a device.
 *  When trying to communicate with a specific peripheral, an instance of this class must
 *  be created (i2c* device = new i2c(deviceAddr);). You can then read and write 'directly'
 *  to the device using i2cRead and i2cWrite. Don't forget to delete it with the destructor
 *  when done.
 *  NOTE: This uses std::this_thread::sleep_for() to sleep while it waits for the signal to
 *      physically reach the devices and/or come back.
 *  WARNING: this WILL throw runtime errors if the file doesn't open. try/catch them
 *  WARNING: bc this involves reading/writing to file, close it (~i2c) before opening it
 *      for another device (do not have two instances open at the same time with r/w)
 ---------------------------------------------------------------------------------------------- */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
extern "C"{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <stdint.h>
#include <unistd.h>


#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>
#include <vector>

#define I2C_FILE "/dev/i2c-1"

class i2c {
private:
    int i2c_fd;

public:
    i2c(uint8_t deviceAddr);
    ~i2c();

    uint8_t i2cRead(uint8_t regAddr);
    char i2cWrite(uint8_t regAddr, uint8_t data);
    std::vector<uint8_t> i2cReadCustom(uint8_t regAddr, uint8_t bytes);
};


