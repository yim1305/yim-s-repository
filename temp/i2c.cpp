#include "i2c.h"

i2c::i2c(uint8_t deviceAddr) {
    //open the file
    i2c_fd = open(I2C_FILE, O_RDWR);
    if(i2c_fd < 0){
        //throw error, file did not open correctly
        throw std::runtime_error("Error on opening I2C device file");
    }

    //set the address
    if(ioctl(i2c_fd, I2C_SLAVE, deviceAddr) < 0){
        //throw error, did not set device
        throw std::runtime_error("Error on setting I2C device address");
    }

}

i2c::~i2c() {
    close(i2c_fd);
}

char i2c::i2cWrite(uint8_t regAddr, uint8_t data) {
    //check file is open
    if(i2c_fd<0){
        //throw error file did not open correctly
        throw std::runtime_error("Error I2C file is not specified");
    }

    uint8_t buf[2] = {regAddr, data};
    write(i2c_fd, buf, sizeof(buf));

    //wait for write to go through
    std::this_thread::sleep_for(std::chrono::microseconds(3));

    return 0x00; //return 0 on success
}

uint8_t i2c::i2cRead(uint8_t regAddr) {
    //check file is open
    if(i2c_fd<0){
        //throw error
        throw std::runtime_error("Error I2C file is not specified");
    }

    //write to device to tell it to prepare for read
    write(i2c_fd, &regAddr, 1);
    //wait for it to go through
    std::this_thread::sleep_for(std::chrono::microseconds(3)); //verify this time works consistently

    uint8_t val;
    read(i2c_fd, &val, sizeof(val));
    //wait for it to come back
    std::this_thread::sleep_for(std::chrono::microseconds(3));

    return val;
}

std::vector<uint8_t> i2c::i2cReadCustom(uint8_t regAddr, uint8_t bytes) {
    if(i2c_fd<0){
        //throw error
        throw std::runtime_error("Error I2C file is not specified");
    }
    std::vector<uint8_t> results;
    write(i2c_fd, &regAddr, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(3));
    uint8_t* buff = new uint8_t[bytes];
    int bytesread = read(i2c_fd, buff, bytes);

    //std::cout << "bytesread: " << bytesread << std::endl;
    if(bytesread != bytes){
        throw std::runtime_error("Error reading file");
    }
    std::this_thread::sleep_for(std::chrono::microseconds(bytes));

    for(int i = 0; i<bytes; i++){
        results.push_back(buff[i]);
    }
    delete[] buff;
    return results;
}
