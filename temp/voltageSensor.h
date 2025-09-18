#pragma once

#include "i2c.h"
#include <vector>
#include <bitset>
#include <iostream>
using namespace std;

#define VOLTADDR (0x40) //default register
#define manu_ID (0x5449) //manufacturer ID, should be MANUFACTURER_ID reg
#define dev_ID (0x2270) //device ID, should be in DIE_ID reg

typedef enum{
    CONFIG_VOLT = 0x00,
    CURRENT = 0x01,
    BUS_VOLTAGE = 0x02,
    POWER = 0x03,

    MASK_ENABLE = 0x06,
    ALERT_LIMIT = 0x07,

    MANUFACTURER_ID = 0xFE,
    DIE_ID = 0xFF
} voltage_reg_t;

void setupVolt();
bool checkVoltAlive();

vector<float> getVoltData();
float getPower();
float getVoltage();
float getCurrent();
