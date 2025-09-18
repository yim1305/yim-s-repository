#include "barometer.h"


void setupBaro(){
    i2c* bus;
    try{
        bus = new i2c(BARO_ADDR);
    }
    catch(const runtime_error& err){
        throw err; //pass the error up
    }

    //setup in normal mode: 00110011
    bus->i2cWrite(PWR_CTRL, 0x33);
    delete bus;
}

bool checkAlive(){
    i2c* bus;
    try{
        bus = new i2c(BARO_ADDR);
    }
    catch(const runtime_error& err){
        throw err; //pass the error up
    }
    uint8_t id = bus->i2cRead(CHIP_ID);
    //cout << bitset<8>(id) << endl;
    delete bus;
    if(id == 0x60){
        return true;
    }
    return false;
}

vector<float> getPressure(){
    //pressure is stored as 3 bytes in the 3 pressure data registers
    //temp is the next three reg's
    i2c* baro;
    try{
        baro = new i2c(BARO_ADDR);
    }
    catch(const runtime_error& err){
        throw err; //pass the error up
    }

    //burst read the six data registers- XLSB, LSB, MSB
    vector<uint8_t> pressBuff;
    try{
        pressBuff = baro->i2cReadCustom(DATA_0, 6);
    }
    catch(const runtime_error& err){
        throw err; //pass it up
    }
    /*vector<uint8_t> pressBuff;
    pressBuff.push_back(baro->i2cRead(DATA_0));
    pressBuff.push_back(baro->i2cRead(DATA_1));
    pressBuff.push_back(baro->i2cRead(DATA_2));
    pressBuff.push_back(baro->i2cRead(DATA_3));
    pressBuff.push_back(baro->i2cRead(DATA_4));
    pressBuff.push_back(baro->i2cRead(DATA_5));*/


    //cout << "raw pressure reading from barometer register" << bitset<8>(pressBuff[0]) << endl;
    //calibration! get the calibration numbers TODO: check if these change or if we can poll only once in the beginning (part of setup)
    vector<uint8_t> rawCoeff;
    try{
        rawCoeff = baro->i2cReadCustom(CAL_2, 21);
    }
    catch(const runtime_error& err){
        throw err; //pass it up
    }
    /*vector<uint8_t> rawCoeff;
    rawCoeff.push_back(baro->i2cRead(CAL_1));
    rawCoeff.push_back(baro->i2cRead(CAL_2));
    rawCoeff.push_back(baro->i2cRead(CAL_3));
    rawCoeff.push_back(baro->i2cRead(CAL_4));
    rawCoeff.push_back(baro->i2cRead(CAL_5));
    rawCoeff.push_back(baro->i2cRead(CAL_6));
    rawCoeff.push_back(baro->i2cRead(CAL_7));
    rawCoeff.push_back(baro->i2cRead(CAL_8));
    rawCoeff.push_back(baro->i2cRead(CAL_9));
    rawCoeff.push_back(baro->i2cRead(CAL_10));
    rawCoeff.push_back(baro->i2cRead(CAL_11));
    rawCoeff.push_back(baro->i2cRead(CAL_12));
    rawCoeff.push_back(baro->i2cRead(CAL_13));
    rawCoeff.push_back(baro->i2cRead(CAL_14));
    rawCoeff.push_back(baro->i2cRead(CAL_15));
    rawCoeff.push_back(baro->i2cRead(CAL_16));
    rawCoeff.push_back(baro->i2cRead(CAL_17));
    rawCoeff.push_back(baro->i2cRead(CAL_18));
    rawCoeff.push_back(baro->i2cRead(CAL_19));
    rawCoeff.push_back(baro->i2cRead(CAL_20));
    rawCoeff.push_back(baro->i2cRead(CAL_21));*/

    //don't need any more data, delete the instance of the barometer
    delete baro;

    //combine and bit shift to make a 32 bit/4byte int
    uint32_t pressFull = (0x00 << 24) | (pressBuff[2] << 16) | (pressBuff[1] << 8) | (pressBuff[0] << 0);
    uint32_t tempFull = (0x00 << 24) | (pressBuff[5] << 16) | (pressBuff[4] << 8) | (pressBuff[3] << 0);

    //put them into correct types of numbers (datasheet 3.11.1, page 28)
    //16 bit unsigned short
    unsigned short int ct1 = (rawCoeff[1] << 8) | (rawCoeff[0] << 0);
    unsigned short int ct2 = (rawCoeff[3] << 8) | (rawCoeff[2] << 0);
    //8 bit signed
    signed char ct3 = rawCoeff[4];
    //16 bit signed short
    short int c1 = (rawCoeff[6] << 8) | (rawCoeff[5] << 0);
    short int c2 = (rawCoeff[8] << 8) | (rawCoeff[7] << 0);
    signed char c3 = rawCoeff[9];
    signed char c4 = rawCoeff[10];
    unsigned short int c5 = (rawCoeff[12] << 8) | (rawCoeff[11] << 0);
    unsigned short int c6 = (rawCoeff[14] << 8) | (rawCoeff[13] << 0);
    signed char c7 = rawCoeff[15];
    signed char c8 = rawCoeff[16];
    short int c9 = (rawCoeff[18] << 8) | (rawCoeff[17] << 0);
    signed char c10 = rawCoeff[19];
    signed char c11 = rawCoeff[20];

    //turn into calibration coefficients in floating point. datasheet 9.1, page 55
    //TODO: check that the ints divide properly into floats- i think the problem is here
    float t1 = (float)ct1 / (pow(2.0, -8.0));
    float t2 = (float)ct2 / (pow(2.0, 30.0));
    float t3 = (float)ct3 / (pow(2.0, 48.0));
    float p1 = ((float)c1 - pow(2.0, 14.0)) / pow(2.0, 20.0);
    float p2 = ((float)c2 - pow(2.0, 14.0)) / pow(2.0, 29.0);
    float p3 = (float)c3 / pow(2.0, 32.0);
    float p4 = (float)c4 / pow(2.0, 37.0);
    float p5 = (float)c5 / pow(2.0, -3.0);
    float p6 = (float)c6 / pow(2.0, 6.0);
    float p7 = (float)c7 / pow(2.0, 8.0);
    float p8 = (float)c8 / pow(2.0, 15.0);
    float p9 = (float)c9 / pow(2.0, 48.0);
    float p10 = (float)c10 / pow(2.0, 48.0);
    float p11 = (float)c11 / pow(2.0, 65.0);

    //calibrate the temperature- datasheet 9.2
    float partial_1 = ((float)tempFull - t1);
    float partial_2 = (partial_1 * t2);
    float realTemp = partial_2 + (partial_1 * partial_1) * t3;

    //calibrate the pressure into hPa - 9.3 pressure compensation
    float partial_data1 = p6 * realTemp;
    float partial_data2 = p7 * (realTemp * realTemp);
    float partial_data3 = p8 * (realTemp * realTemp * realTemp);
    float partial_out1 = p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = p2 * realTemp;
    partial_data2 = p3 * (realTemp * realTemp);
    partial_data3 = p4 * (realTemp * realTemp * realTemp);
    float partial_out2 = (float)pressFull * (p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)pressFull * (float)pressFull;
    partial_data2 = p9 + p10 * realTemp;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + (float)pressFull * (float)pressFull * (float)pressFull * p11;
    float realPress = partial_out1 + partial_out2 + partial_data4;

    //return pressure in hPa = 1 millibar
    vector<float> results;
    results.push_back(realTemp);
    results.push_back(realPress/100.0);

    //delete dynamically allocated memory
    //delete[] pressBuff;
    //delete[] rawCoeff;

    //return the results
    return results;

}

float getTemp(){
    //temp is stored as 3 bytes in the 3 temp data registers
    i2c* baro;
    try{
        baro = new i2c(BARO_ADDR);
    }
    catch(const runtime_error& err){
        throw err; //pass the error up
    }


    //burst read the three temp registers- XLSB, LSB, MSB
    vector<uint8_t>  tempBuff;
    try{
        tempBuff = baro->i2cReadCustom(DATA_3, 3);
    }
    catch(const runtime_error& err){
        throw err; //pass it up
    }
    /*vector<uint8_t> tempBuff;
    tempBuff.push_back(baro->i2cRead(DATA_3));
    tempBuff.push_back(baro->i2cRead(DATA_4));
    tempBuff.push_back(baro->i2cRead(DATA_5));*/
    uint32_t tempFull = (0x00 << 24) | ((tempBuff[2]) << 16) | ((tempBuff[1]) << 8) | ((tempBuff[0]) << 0);

    //get the temp calibration
    vector<uint8_t> calibBuff;
    try{
        calibBuff = baro->i2cReadCustom(CAL_2, 5);
    }
    catch (const runtime_error& err){
        throw err; //pass it up
    }
    /*vector<uint8_t> calibBuff;
    calibBuff.push_back(baro->i2cRead(CAL_1));
    calibBuff.push_back(baro->i2cRead(CAL_2));
    calibBuff.push_back(baro->i2cRead(CAL_3));
    calibBuff.push_back(baro->i2cRead(CAL_4));
    calibBuff.push_back(baro->i2cRead(CAL_5));*/

    //don't need any more data, delete the instance of the barometer
    delete baro;

    //put them into correct format
    //16 bit unsigned short
    unsigned short int ct1 = ((calibBuff[1]) << 8) | ((calibBuff[0]) << 0);
    unsigned short int ct2 = ((calibBuff[3]) << 8) | ((calibBuff[2]) << 0);
    //8 bit signed
    signed char ct3 = (calibBuff[4]);
    float t1 = (float)ct1 / (pow(2.0, -8.0));
    float t2 = (float)ct2 / (pow(2.0, 30.0));
    float t3 = (float)ct3 / (pow(2.0, 48.0));

    //calibrate the temperature- datasheet 9.2
    float partial_1 = ((float)tempFull - t1);
    float partial_2 = (partial_1 * t2);
    float realTemp = partial_2 + (partial_1 * partial_1) * t3;

    //delete dynamically allocated mem
    //delete[] tempBuff;
    //delete[] calibBuff;

    return realTemp;

}

float getAlt(float seaLevel){
    //get pressure/temp
    vector<float> data;
    try{
        data = getPressure();
    }
    catch(const runtime_error& err){
        throw err;
    }
    //cout << "pressure: " << data[1] << endl << "temperature: " << data[0] << endl;
    //convert from pressure and temp to altitude (meters)
    //https://en.wikipedia.org/wiki/Pressure_altitude
    float alt = 145366.45 * (1 - pow(data[1]/seaLevel, 0.190284));
    alt = alt * 3.28084; //return in feet
    return alt;
}

float getAlt(vector<float> pressTemp, float seaLevel){
    float alt = 44307.7 * (1 - pow(pressTemp[1]/seaLevel, 0.190284));
    alt = alt * 3.28084; //return in feet
    return alt;
}
