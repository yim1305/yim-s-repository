#include "imu.h"

void setupIMU(){
    //open the bus
    i2c* imu_bus = new i2c(IMU_ADDRESS_A);

    //write configs  0b01010100 acceleration normal mode, 16g
    imu_bus->i2cWrite(CTRL1_XL, 0x54);
    //gyroscope 0b01010000 gyro normal mode, TODO: ??? pg 62
    imu_bus->i2cWrite(CTRL2_G, 0x5C);


    //close bus
    delete imu_bus;
}

bool checkIMUAlive(){
    i2c* imu = new i2c(IMU_ADDRESS_A);
    uint8_t id = imu->i2cRead(WHO_AM_I);
    //cout << bitset<8>(id) << endl;
    delete imu;
    if(id != WHO_I_AM){
        return false;
    }
    return true;
}

std::vector<float> getAccelData(){
    //open bus
    i2c* imu = new i2c(IMU_ADDRESS_A);
    //collect data from imu
    vector<uint8_t> rawData = imu->i2cReadCustom(OUTX_L_G, 12);
    //6 for gyro and 6 for imu
    //each is expressed as a 16 bit word in two's compliment
    int16_t rawGyroX = (rawData[1] << 8) | rawData[0];
    //undo 2s compliment: -1 and flip
    //cout << "raw 2s compliment: " << bitset<16>(rawGyroX) << endl;
    rawGyroX = ~(rawGyroX - 1);
    //cout << "raw un2s complimented: " << bitset<16>(rawGyroX) << endl;
    int16_t rawGyroY = (rawData[3] << 8) | rawData[2];
    rawGyroY = ~(rawGyroY - 1);
    int16_t rawGyroZ = (rawData[5] << 8) | rawData[4];
    rawGyroZ = ~(rawGyroZ - 1);
    int16_t rawAccelX = (rawData[7] << 8) | rawData[6];
    rawAccelX = ~(rawAccelX - 1);
    int16_t rawAccelY = (rawData[9] << 8) | rawData[8];
    rawAccelY = ~(rawAccelY - 1);
    int16_t rawAccelZ = (rawData[11] << 8) | rawData[10];
    rawAccelZ = ~(rawAccelZ - 1);

    //make any necessary calibrations/adjustments
    //yim:
    //Accel Conversion to g
    float AccelX_g = (rawAccelX / 32768.0f) * 16.0f;
    float AccelY_g = (rawAccelY / 32768.0f) * 16.0f;
    float AccelZ_g = (rawAccelZ / 32768.0f) * 16.0f;

    //Angular Vel Conversion to DPS (degree per sec)
    float GyroX_dps = (rawGyroX / 32768.0f) * 500.0f;
    float GyroY_dps = (rawGyroY / 32768.0f) * 500.0f;
    float GyroZ_dps = (rawGyroZ / 32768.0f) * 500.0f;

    //close bus
    delete imu;
    vector<float> results;
    results.push_back(GyroX_dps);
    results.push_back(GyroY_dps);
    results.push_back(GyroZ_dps);
    results.push_back(AccelX_g);
    results.push_back(AccelY_g);
    results.push_back(AccelZ_g);

    //return results
    return results;
}

//necessary ? prob not since gotten in burst read ^^
/*std::vector<uint16_t> getGyroData(){
    //open bus
    i2c* imu = new i2c(IMU_ADDRESS_A);

    //collect data from imu

    //make any necessary calibrations/adjustments

    //close bus
    delete imu;
    //return results

}
*/
