#pragma once

#include "i2c.h"
#include <vector>
#include <bitset>
#include <iostream>
using namespace std;

//define address and id
#define IMU_ADDRESS_A (0x6A)
#define IMU_ADDRESS_B (0x6B)

#define WHO_I_AM (0x6A)

//list registers
typedef enum{
    //config
    FUNC_CFG_ACCESS = 0x01,
    SENSOR_SYNC_TIME_FRAME = 0x04,
    SENSOR_SYNC_RES_RATIO = 0x05,
    FIFO_CTRL1 = 0X06,
    FIFO_CTRL2 = 0X07,
    FIFO_CTRL3 = 0X08,
    FIFO_CTRL4 = 0X09,
    FIFO_CTRL5 = 0X0A,
    DRDY_PULSE_CFG_G = 0X0B,
    INT1_CTRL = 0X0D,
    INT2_CTRL = 0X0E,
    WHO_AM_I = 0X0F,
    MASTER_CONFIG = 0x1A,
    STATUS_REG = 0X1E,
    MASTER_CMD_CODE = 0X60,

    //control
    CTRL1_XL = 0X10,
    CTRL2_G = 0X11,
    CTRL3_C = 0X12,
    CTRL4_C = 0X13,
    CTRL5_C = 0X14,
    CTRL6_C = 0X15,
    CTRL7_G = 0X16,
    CTRL8_XL = 0X17,
    CTRL9_XL = 0X18,
    CTRL10_C = 0X19,

    //interrupts
    WAKE_UP_SRC = 0X1B,
    TAP_SRC = 0X1C,
    D6D_SRC = 0X1D,
    FUNC_SRC1 = 0X53,
    FUNC_SRC2 = 0X54,
    WRIST_TILT_IA = 0X55,
    TAP_CFG = 0X58,
    TAP_THS_6D = 0X59,
    INT_DUR2 = 0X5A,
    WAKE_UP_THS = 0X5B,
    WAKE_UP_DUR = 0X5C,
    FREE_FALL = 0X5D,
    MD1_CFG = 0X5E,
    MD2_CFG = 0X5F,

    //temperature output
    OUT_TEMP_L = 0X20,
    OUT_TEMP_H = 0X21,
    //gyro output
    OUTX_L_G = 0X22,
    OUTX_H_G = 0X23,
    OUTY_L_G = 0X24,
    OUTY_H_G = 0X25,
    OUTZ_L_G = 0X26,
    OUTZ_H_G = 0X27,
    //accel output
    OUTX_L_XL = 0X28,
    OUTX_H_XL = 0X29,
    OUTY_L_XL = 0X2A,
    OUTY_H_XL = 0X2B,
    OUTZ_L_XL = 0X2C,
    OUTZ_H_XL = 0X2D,

    //sensor hub output
    SENSORHUB1_REG = 0X2E,
    SENSORHUB2_REG = 0X2F,
    SENSORHUB3_REG = 0X30,
    SENSORHUB4_REG = 0X31,
    SENSORHUB5_REG = 0X32,
    SENSORHUB6_REG = 0X33,
    SENSORHUB7_REG = 0X34,
    SENSORHUB8_REG = 0X35,
    SENSORHUB9_REG = 0X36,
    SENSORHUB10_REG = 0X37,
    SENSORHUB11_REG = 0X38,
    SENSORHUB12_REG = 0X39,
    SENSORHUB13_REG = 0X4D,
    SENSORHUB14_REG = 0X4E,
    SENSORHUB15_REG = 0X4F,
    SENSORHUB16_REG = 0X50,
    SENSORHUB17_REG = 0X51,
    SENSORHUB18_REG = 0X52,

    //FIFO status
    FIFO_STATUS1 = 0X3A,
    FIFO_STATUS2 = 0X3B,
    FIFO_STATUS3 = 0X3C,
    FIFO_STATUS4 = 0X3D,
    //FIFO data output
    FIFO_DATA_OUT_L = 0X3E,
    FIFO_DATA_OUT_H = 0X3F,

    //timestamp output
    TIMESTAMP0_REG = 0X40,
    TIMESTAMP1_REG = 0X41,
    TIMESTAMP2_REG = 0X42,
    //step counter timestamp
    STEP_TIMESTAMP_L = 0X49,
    STEP_TIMESTAMP_H = 0X4A,
    //step counter output
    STEP_COUNTER_L = 0X4B,
    STEP_COUNTER_H = 0X4C,

    SENS_SYC_SPI_ERROR_CODE = 0X61,

    //external magnetometer raw data output (we don't have an external magnetometer)
    OUT_MAG_RAW_X_L = 0X66,
    OUT_MAG_RAW_X_H = 0X67,
    OUT_MAG_RAW_Y_L = 0X68,
    OUT_MAG_RAW_Y_H = 0X69,
    OUT_MAG_RAW_Z_L = 0X6A,
    OUT_MAG_RAW_Z_H = 0X6B,

    //accel user offset correction
    X_OFS_USR = 0X73,
    Y_OFS_USR = 0X74,
    Z_OFS_USR = 0X75

} imu_reg_t;

//setup / config
void setupIMU();
bool checkIMUAlive();

//get accel data
std::vector<float> getAccelData();

//get gyro data
//std::vector<uint16_t> getGyroData();
