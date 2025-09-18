//#pragma once

#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <memory>
#include <sstream>

using namespace std;


//Individual Structs
struct Baro_Raw {
    time_t time;
    long long millisecs;
    //std::string logLevel;
    float pressure;
    float altitude;
    float temperature;
    bool nextExists;

    Baro_Raw(time_t time, /*const std::string& level,*/ long long ms, float altitude, float pressure, float temp, bool n)
            : time(time), /*logLevel(level),*/ millisecs(ms), altitude(altitude), pressure(pressure), temperature(temp), nextExists(n) {}
    Baro_Raw()
    {
        time = 0;
        millisecs = 0;
        //logLevel = "Raw Data";
        altitude = 0.0f;
        pressure = 0.0f;
        temperature = 0.0f;
        nextExists = false;
    }
};


struct IMU_Raw
{
    time_t time;
    long long millisecs;
    //std::string logLevel;
    float imuX;
    float imuY;
    float imuZ;
    float imuGyX;
    float imuGyY;
    float imuGyZ;
    bool nextExists;


    IMU_Raw(time_t time, /*const std::string& level,*/ long long ms, float x, float y, float z, float gx, float gy, float gz, bool n)
            : time(time), /*logLevel(level),*/millisecs(ms), imuX(x), imuY(y), imuZ(z), imuGyX(gx), imuGyY(gy), imuGyZ(gz), nextExists(n) {}
    IMU_Raw()
    {
        time = 0;
        //logLevel = "Raw Data";
        millisecs = 0;
        imuX = 0;
        imuY = 0;
        imuZ = 0;
        imuGyX = 0;
        imuGyY = 0;
        imuGyZ = 0;
        nextExists = false;
    }
};

//TODO: baro filtered and baro raw are the same rn. and imu filtered is also the same as imu raw. so why two dif structs --> see if only using one struct works
struct Baro_Filtered {
    time_t time;
    long long millisecs;
    //std::string logLevel;
    float altitude;
    float pressure;
    float temperature;
    bool nextExists;
    double kfVel;

    Baro_Filtered(time_t time, /*const std::string& level,*/long long ms, float altitude, float pressure, float temp, double kfVel, bool n)
            : time(time), /*logLevel(level),*/millisecs(ms), altitude(altitude), pressure(pressure), temperature(temp), kfVel(kfVel), nextExists(n) {}
    Baro_Filtered()
    {
        time = {};
        //logLevel = "Filtered Data";
        millisecs = 0;
        altitude = 0.0f;
        pressure = 0.0f;
        temperature = 0.0f;
	kfVel = 0.0;
        nextExists = false;
    }
};
struct IMU_Filtered {
    time_t time;
    long long millisecs;
    //std::string logLevel;
    float imuX;
    float imuY;
    float imuZ;
    float imuGyX;
    float imuGyY;
    float imuGyZ;
    bool nextExists;


    IMU_Filtered(time_t time, /*const std::string& level,*/ long long ms, float x, float y, float z, float gx, float gy, float gz, bool n)
            : time(time), /*logLevel(level),*/millisecs(ms), imuX(x), imuY(y), imuZ(z), imuGyX(gx), imuGyY(gy), imuGyZ(gz), nextExists(n) {}
    IMU_Filtered()
    {
        time = 0;
        millisecs = 0;
        //logLevel = "Filtered Data";
        imuX = 0;
        imuY = 0;
        imuZ = 0;
        imuGyX = 0;
        imuGyY = 0;
        imuGyZ = 0;
        nextExists = false;
    }
};

struct Vel_Alt {
    time_t time;
    long long millisecs;
    //string logLevel;
    float velocity;
    bool nextExists;

    Vel_Alt(time_t time, long long ms, /*const string& level,*/ float v, bool n) : time(time), /*logLevel(level),*/ millisecs(ms), velocity(v), nextExists(n) {}
    Vel_Alt() {
        time = {};
        millisecs = 0;
        //logLevel = "Velocity Data";
        velocity = 0.0f;
    }
};

struct Vel_Accel {
    time_t time;
    long long millisecs;
    //string logLevel;
    //TODO: what data type???
    float vel;
    bool nextExists;

    Vel_Accel(time_t time, long long ms, /*const string& level,*/ float v, bool n) : time(time), millisecs(ms), /*logLevel(level),*/ vel(v), nextExists(n) {}
    Vel_Accel() {
        time = {};
        millisecs = 0;
        //logLevel = "Velocity Data";
        vel = 0.0f;
        nextExists = false;
    }
};


string getCurrentTimestamp();
string getCurrentTimestamp(time_t now, long long ms);
pair<time_t, long long> getTimeBoth();
string toCsvString(Baro_Raw enter);
string toCsvString(IMU_Raw enter);
string toCsvString(Baro_Filtered enter);
string toCsvString(IMU_Filtered enter);
void writeHeaderBaro(ofstream& csvFile);
void writeHeaderIMU(ofstream& csvFile);
void writeHeaderLog(ofstream& csvFile);
void addLogEntry(const string& message, ofstream& logfile);
void addBarEntry(Baro_Raw enter, ofstream& csvFile);
void addIMUEntry(IMU_Raw enter, ofstream& csvFile);
void close(ofstream& csvFile);
void writeToFile(ofstream& file, string entry);

//TODO: eliminate unused functions


//use this one:
/*
std::string getCurrentTimestamp(chrono::time_point<chrono::system_clock> now, long long ms) {
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
    ss << "." << (int)ms;
    return ss.str(); //results in YYYY-MM-DD_HH:MM:SS.mmm ex) 2024-11-08_18:15:40.683
}

chrono::time_point<chrono::system_clock> getTime(){
    auto now = std::chrono::system_clock::now();
    //auto in_time_t = std::chrono::system_clock::to_time_t(now);
    //cout << now << endl;
    //cout << in_time_t << endl;
    return now;
}

long long getMillisec(chrono::time_point<chrono::system_clock> now){
    auto transformed = now.time_since_epoch().count() / 1000000;
    auto millis = transformed % 1000;
    return millis;
    //return 0;
}
*/
//sike use this:


//NEW WRITE FUNCTIONS
