#pragma once

#include "i2c.h"
#include <cmath>
#include <iostream>
//#include <pigpio.h>
#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#include <vector>
#include <bitset>
using namespace std;

//define address
#define BARO_ADDR 0x77

typedef enum {
    CHIP_ID = 0x00,
    REV_ID = 0x01,
    ERR_REG = 0x02,
    STATUS = 0x03,

    //pressure data
    DATA_0 = 0x04, //press XLSB 7-0
    DATA_1 = 0x05, //press LSB 15-8
    DATA_2 = 0x06, //press MSB 23-16
    //temp data
    DATA_3 = 0x07, //temp XLSB 7-0
    DATA_4 = 0x08, //temp LSB 15-8
    DATA_5 = 0x09, //temp MSB 23-16
    //sensor time
    SENSORTIME_0 = 0x0C,
    SENSORTIME_1 = 0x0D,
    SENSORTIME_2 = 0x0E,
    //sensor status flags
    EVENT = 0x10,
    INT_STATUS = 0x11,
    FIFO_LENGTH_0 = 0x12,
    FIFO_LENGTH_1 = 0x13,
    FIFO_DATA = 0x14,
    FIFO_WTM_0 = 0x15,
    FIFO_WTM_1 = 0x16,
    FIFO_CONFIG_1 = 0x17,
    FIFO_CONFIG_2 = 0x18,
    INT_CTRL = 0x19,
    IF_CONF = 0x1A,
    PWR_CTRL = 0x1B,
    OSR = 0x1C,
    ODR = 0x1D,
    CONFIG = 0x1F,

    //calibration data
    CAL_1 = 0X30,
    CAL_2 = 0X31,
    CAL_3 = 0X32,
    CAL_4 = 0X33,
    CAL_5 = 0X34,
    CAL_6 = 0X35,
    CAL_7 = 0X36,
    CAL_8 = 0X37,
    CAL_9 = 0X38,
    CAL_10 = 0X39,
    CAL_11 = 0X3A,
    CAL_12 = 0X3B,
    CAL_13 = 0X3C,
    CAL_14 = 0X3D,
    CAL_15 = 0X3E,
    CAL_16 = 0X3F,
    CAL_17 = 0X40,
    CAL_18 = 0X41,
    CAL_19 = 0X42,
    CAL_20 = 0X43,
    CAL_21 = 0X44,
    CAL_22 = 0X45,
    CAL_23 = 0X46,
    CAL_24 = 0X47,
    CAL_25 = 0X48,
    CAL_26 = 0X49,
    CAL_27 = 0X4A,
    CAL_28 = 0X4B,
    CAL_29 = 0X4C,
    CAL_30 = 0X4D,
    CAL_31 = 0X4E,
    CAL_32 = 0X4F,
    CAL_33 = 0X50,
    CAL_34 = 0X51,
    CAL_35 = 0X52,
    CAL_36 = 0X53,
    CAL_37 = 0X54,
    CAL_38 = 0X55,
    CAL_39 = 0X56,
    CAL_40 = 0X57,

    CMD = 0x7E
} barometer_reg_t;


void setupBaro();
bool checkAlive();

vector<float> getPressure();
float getTemp();

float getAlt(float seaLevel);
float getAlt(vector<float> pressTemp, float seaLevel);
