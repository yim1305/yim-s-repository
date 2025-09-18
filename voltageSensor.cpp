#include "voltageSensor.h"

//unnecessary unless need to change configs
void setupVolt(){
    i2c* volt = new i2c(VOLTADDR);

    //configs
    //default configs (0x6127) should be fine. see manual if need diff:
    //https://www.ti.com/lit/ds/symlink/ina260.pdf
    //volt->i2cWrite(CONFIG_VOLT, 0x6127); //this function only writes 1 byte... would need a new function to write more than one byte

    delete volt;


}

bool checkVoltAlive(){
    i2c* volt = new i2c(VOLTADDR);
    vector<uint8_t> manufacturerID = volt->i2cReadCustom(MANUFACTURER_ID, 2);
    vector<uint8_t> deviceID = volt->i2cReadCustom(DIE_ID, 2);
    delete volt;
    uint16_t givenManuID = (manufacturerID[0] << 8) | manufacturerID[1];
    uint16_t givenDeviceID = (deviceID[0] << 8) | deviceID[1];
    if(givenManuID == manu_ID && givenDeviceID == dev_ID){
        return true;
    }
    return false;
}

vector<float> getVoltData(){
    i2c* volt = new i2c(VOLTADDR);
    vector<uint8_t> rawData = volt->i2cReadCustom(CURRENT, 6);
    delete volt;

    //convert all nums
    int16_t rawCurr = (rawData[0] << 8) | rawData[1];
    float curr = ((float)rawCurr * 1.25f) / 1000.0f;

    int16_t rawVolt = (rawData[2] << 8) | rawData[3];
    float voltage = (float)rawVolt * 1.25f / 1000.0f;

    int16_t rawPower = (rawData[4] << 8) | rawData[5];
    float power = (float)rawPower * 10.0f / 1000.0f;

    vector<float> results;
    results.push_back(curr);
    results.push_back(voltage);
    results.push_back(power);

    return results;
}

float getCurrent(){
    i2c* volt = new i2c(VOLTADDR);
    vector<uint8_t> rawCurrent = volt->i2cReadCustom(CURRENT, 2);
    delete volt;
    //data is in two's compliment
    int16_t togetherCurr = (rawCurrent[0] << 8) | rawCurrent[1];
    togetherCurr = ~(togetherCurr - 1); //untwo's compliment... is this actually necessary? try what happens without it.
    float curr = (float)togetherCurr * 1.25f; //in milliamps
    return curr / 1000.0f; //return in amps
}

float getVoltage(){
    i2c* volt_bus = new i2c(VOLTADDR);
    vector<uint8_t> rawVolt = volt_bus->i2cReadCustom(BUS_VOLTAGE, 2);
    delete volt_bus;
    int16_t rawVoltTog = (rawVolt[0] << 8) | rawVolt[1];
    rawVoltTog = ~(rawVoltTog - 1);
    float reportedV = (float)rawVoltTog * 1.25; //in millivolts
    return reportedV / 1000.0f; //return in volts

}

float getPower(){
    i2c* volt = new i2c(VOLTADDR);
    vector<uint8_t> power = volt->i2cReadCustom(POWER, 2);
    delete volt;
    int16_t rawPower = (power[0] << 8) | power[1];
    rawPower = ~(rawPower - 1);
    float reportedP = (float)rawPower * 10; //in milliwatts
    return (reportedP / 1000.0f); //return in watts
}
