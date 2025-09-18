#include "logger.h"


string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
    auto transformed = now.time_since_epoch().count() / 1000000;
    auto millis = transformed % 1000;
    if((int)millis < 10 && (int)millis < 100){
        ss << ".00" << (int)millis;
    }
    else if((int)millis < 100 && (int)millis >= 10){
        ss << ".0" << (int)millis;
    }
    else{
        ss << "." << (int)millis;
    }
    return ss.str();
}

pair<time_t, long long> getTimeBoth(){
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto transformed = now.time_since_epoch().count() / 1000000;
    auto millis = transformed % 1000;
    return make_pair(in_time_t, millis);
}
string getCurrentTimestamp(time_t time, long long ms){
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%X");
    if((int)ms < 10 && (int)ms < 100){
        ss << ".00" << (int)ms;
    }
    else if((int)ms < 100 && (int)ms >= 10){
        ss << ".0" << (int)ms;
    }
    else{
        ss << "." << (int)ms;
    }
    return ss.str(); //results in YYYY-MM-DD_HH:MM:SS.mmm ex) 2024-11-08_18:15:40.683
}

std::string toCsvString(Baro_Raw enter)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << getCurrentTimestamp(enter.time, enter.millisecs) << ",Raw Data," << enter.altitude << "," << enter.pressure << "," << enter.temperature;
    return oss.str();
}
std::string toCsvString(IMU_Raw enter)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << getCurrentTimestamp(enter.time, enter.millisecs) << ",Raw Data," << enter.imuX << "," << enter.imuY << "," << enter.imuZ  << "," << enter.imuGyX << "," << enter.imuGyY << "," << enter.imuGyZ;
    return oss.str();
}
string toCsvString(Baro_Filtered enter){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << getCurrentTimestamp(enter.time, enter.millisecs) << ",Filtered Data," << enter.altitude << "," << enter.pressure << "," << enter.temperature;
    return oss.str();
}
string toCsvString(IMU_Filtered enter){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << getCurrentTimestamp(enter.time, enter.millisecs) << "," << enter.imuX << "," << enter.imuY << "," << enter.imuZ;
    return oss.str();
}


void writeHeaderBaro(ofstream& csvFile)
{
    std::string tab = "                        ";
    try {
        if (!csvFile.is_open())
        {
            throw std::ios_base::failure("File is not open");
        }
        csvFile << "==============================================================" << std:: endl;
        csvFile << "//" << tab << "SWAMP LAUNCH 2024-2025" << tab << "\\\\" << std::endl;
        csvFile << "==============================================================" << std:: endl;
        csvFile << "Time,Log Level,Barometer Altitude,Pressure,Temperature" << std::endl;

    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }
}

void writeHeaderIMU(std::ofstream& csvFile)
{
    std::string tab = "                        ";
    try {
        if (!csvFile.is_open())
        {
            throw std::ios_base::failure("File is not open");
        }
        csvFile << "==============================================================" << std:: endl;
        csvFile << "//" << tab << "SWAMP LAUNCH 2024-2025" << tab << "\\\\" << std::endl;
        csvFile << "==============================================================" << std:: endl;
        csvFile << "Time,Log Level,IMU X,IMU Y,IMU Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z" << std::endl;

    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }
}
void addLogEntry(const std::string& message, std::ofstream& logFile)
{
    try {
        if (!logFile.is_open())
        {
            throw std::ios_base::failure("Logfile is not open");
        }
        logFile << getCurrentTimestamp() << ": " << (message) << std::endl;

    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }
}


void writeHeaderLog(std::ofstream& csvFile)
{
    std::string tab = "                        ";
    try {
        if (!csvFile.is_open())
        {
            throw std::ios_base::failure("File is not open");
        }
        csvFile << "==============================================================" << std:: endl;
        csvFile << "//" << "\t\t   "  << "SWAMP LAUNCH 2024-2025"   << "\t\t   " << "\\\\" << std::endl;
        csvFile << "==============================================================" << std:: endl;
        csvFile << "Swamp Launch logging started at " << getCurrentTimestamp() << std::endl;
    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }
}


void addBarEntry(Baro_Raw enter, std::ofstream& csvFile)
{
    //Potential try & catch
    try {
        if (!csvFile.is_open())
        {
            throw std::ios_base::failure("File is not open");
        }
        csvFile << toCsvString(enter) << std::endl;

    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }

}
void addIMUEntry(IMU_Raw enter, std::ofstream& csvFile)
{
    //Potential try & catch
    try {
        if (!csvFile.is_open())
        {
            throw std::ios_base::failure("File is not open");
        }
        csvFile << toCsvString(enter) << std::endl;

    }
    catch(std::ios_base::failure& e)  {
        std::cerr << "Caught an ios_base::failure exception: " << e.what() << '\n';
    }

}

void close(std::ofstream& csvFile)
{
    csvFile.close();
}

void writeToFile(ofstream& file, string entry){
    file << entry;
}
