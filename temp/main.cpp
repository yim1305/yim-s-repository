/* NASA USLI 2025 Payload
 * CommuniGator Transmitter
 * H(igh frequency) O(mnidirectional) T(ransmitting) Pocket
 */

//includes
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
#include <functional>
#include <thread>

#include <vector>
#include <list>
#include <sstream>

#include "i2c.h"
#include "barometer.h"
#include "imu.h"
#include "GPIO.h"
#include "logger.h"
using namespace std;


void receiverThread(ofstream& logFile); //monitors for deployment signal, then servo legs open
void kalmanThreadTEMP(list<Baro_Raw>& rawBaroData, list<IMU_Raw>& rawIMUData, list<Baro_Filtered>& filteredBaroVec, list<IMU_Filtered>& filteredIMUVec, ofstream& logFile, ofstream& dataFile, ofstream& imudatafile); //takes in raw imu/baro vals
void altVelocityCalc(list<Baro_Filtered>& altData, list<Vel_Alt>& velVec, pair<time_t, long long>& launchTime, ofstream& logFile); //takes in filtered baro alt
void accelVelocityCalc(list<IMU_Filtered>& imuData, list<Vel_Accel>& velVec, ofstream& logFile); //takes in filtered imu
void stemnautOrientation(IMU_Raw imuLanded); //takes in filtered(?) accel data, returns... ?? not sure of data type yet

string startupTime;

int main(){
    //---------------- startup/setup stuff ----------------

    //get the current time of startup
    startupTime = getCurrentTimestamp();
    //set up file i/o for logging- log status messages
    string logTitle = "logs/swamp_launch_" + startupTime + ".log";
    string baroTitle = "logs/swamp_launch_data_barometer_" + startupTime + ".csv";
    string imuTitle = "logs/swamp_launch_data_imu_" + startupTime + ".csv";
    ofstream logFile(logTitle);
    ofstream baro_csvFile(baroTitle);
    ofstream imu_csvFile(imuTitle);
    writeHeaderLog(logFile);
    writeHeaderBaro(baro_csvFile);
    writeHeaderIMU(imu_csvFile);

    //TODO: CHANGE THESE TO LISTS WE ARE GOING TO MAINTAIN AS FEW AS POSSIBLE IN ACTIVE MEMORY (fast deletes from front)
    //set up the data buffer(s)- vector of tuples or structs? do time test creating a ton of them to see any time difs
        //initialize vector with size we will cut off to avoid reallocation time delays
    /*vector<Baro_Raw> baroBuff;
    baroBuff.reserve(1000); //1000 data points is ~19 seconds of taking data (from subscale)
    vector<IMU_Raw> imuBuff;
    imuBuff.reserve(1000);  //according to estimated vehicle descent time, should be ~5,000 data points for the entire flight launch to landing
    //TODO: vectors to hold filtered data and velocity data (require structs for each) (4 structs min- filtered imu, filtered baro, alt v, accel v)
    vector<Baro_Filtered> filteredBaroBuff;
    filteredBaroBuff.reserve(5000);
    vector<IMU_Filtered> filteredIMUBuff;
    filteredIMUBuff.reserve(5000);
    vector<Vel_Alt> velocityFromBaroBuff;
    velocityFromBaroBuff.reserve(5000);
    vector<Vel_Accel> velocityFromIMUBuff;
    velocityFromIMUBuff.reserve(5000);
     */

    //List forms
    list<Baro_Raw> baroBuff;
    list<IMU_Raw> imuBuff;
    list<Baro_Filtered> filteredBaroBuff;
    list<IMU_Filtered> filteredIMUBuff;
    list<Vel_Alt> velocityFromBaroBuff;
    list<Vel_Accel> velocityFromIMUBuff;



    //create and validate i2c connections- each device should have its own class
        //imu
    bool imuAlive = false;
    try{
        setupIMU();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if(!checkIMUAlive()){
            addLogEntry("IMU failed check, is not alive :(",logFile);
        }
        else{
            imuAlive = true;
            addLogEntry("IMU alive and well", logFile);
        }
    }
    catch(const runtime_error& err){
        //TODO: handle error?
        addLogEntry(err.what(), logFile);
    }

        //barometer
    bool barometerAlive = false;
    float seaLevel = 1013.25; //default sea level pressure
    try{
        setupBaro();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if(!checkAlive()){
            addLogEntry("Barometer failed to initiate :/", logFile);
        }
        else{
            barometerAlive = true;
            addLogEntry("Barometer alive, calculating sea level now", logFile);
            //cycle through a couple just in case first couple are nonsense
            float total = 0;
            float tally = 1; //avoid divide by 0 error
            for(int i=0; i<5; i++){
                vector<float> baroData;
                try{
                    baroData = getPressure();
                }
                catch(const runtime_error& err){
                    //log barometer error
                    //try to reboot barometer, otherwise log error and shut down barometer
                    barometerAlive = false;
                }
                if(baroData[1] > 600 && baroData[1] < 1300){ //ensure not garbage values
                    total += baroData[1];
                    tally += 1;
                }

            }
            if(tally == 6){
                tally = 5;
            }
            seaLevel = total / tally; // avg sea level over how many takes that make sense (no nonsense values)
        }
    }
    catch(const runtime_error& err){
        //TODO: handle error?
        addLogEntry(err.what(), logFile);
    }

    //if both are down, that is bad. it will not be able to tell that launch or landing has happened
    if(!barometerAlive && !imuAlive){
        //TODO: try again??
        //log the badness
        addLogEntry("Both IMU and Barometer died. Womp womp, payload cannot work.", logFile);
    }

        //TODO: voltage sensor


    //maybe check the 'post launch' systems that will be turned off until landing? may forego this.
        //MOSFET systems- in order to test connections with the next three things, must use the mosfets to give them power
        //antenna servo motor- confirm connection?
        //payload leg latch servo- put them into the locking position
        //radio transmitter/receiver- confirm connection/working?
        //close connections and use mosfets to STOP sending power to these systems

    //------------------ Pre Flight Loop --------------------
    //begin The Pre Flight Loop - checking imu and barometer data to see if launch has been detected
    bool launched = false;
    while(!launched){
        pair<time_t, long long> timestamp = getTimeBoth();
        Baro_Raw b;
        IMU_Raw i;
        //don't poll barometer if it's dead
        if(barometerAlive){
            //collect data and add to buffer
            vector<float> baroData;
            try{
                baroData = getPressure();
            }
            catch(const runtime_error& err){
                //log barometer error
                //try to reboot barometer, otherwise log error and shut down barometer
                barometerAlive = false;
            }
            float alt = getAlt(baroData, seaLevel);
            b.time = timestamp.first;
            b.millisecs = timestamp.second;
            b.pressure = baroData[1];
            b.temperature = baroData[0];
            b.altitude = alt;
            baroBuff.push_back(b);
            //set nextexists to true on previous one so it knows there's more
            if(baroBuff.size() > 1){
                prev(prev(baroBuff.end()))->nextExists = true;
            }

            //check if launch has occurred
            if(b.altitude > 20){  //above 20 meters
                //TODO: change condition of launch to be better- something about rapid increase, as well as the threshold
                launched = true;
            }

            //delete first [couple] elements if at capacity (1000)
            if(!launched && baroBuff.size() > 30){ //.size is constant complexity
                baroBuff.pop_front();
                //remove first 500 elements bc not needed
                //also decreases amount of resizing actions by decreasing frequency
            }

        }
        //don't poll imu if it's dead
        if(imuAlive){
            //collect data and add to buffer
            vector<float> imuData = getAccelData();
            i.time = timestamp.first;
            i.millisecs = timestamp.second;
            i.imuX = imuData[3];
            i.imuY = imuData[4];
            i.imuZ = imuData[5];
            i.imuGyX = imuData[0];
            i.imuGyY = imuData[1];
            i.imuGyZ = imuData[2];
            imuBuff.push_back(i);

            //check if launch has occurred
            if(abs(i.imuX) > 6){ //greater than 3 Gs TODO: find good g value as threshold and figure out which axis is down (-X is down)
                launched = true;
            }

            //delete old buffer values
            if(!launched && imuBuff.size() > 30){
                imuBuff.pop_front();
                //same as barometer reasoning
            }

        }

    }

    //----------------- Flight -------------------
    //record flight time and log it
    pair<time_t, long long> launchTime = getTimeBoth();
    //log it
    addLogEntry("Launched at: " + getCurrentTimestamp(launchTime.first, launchTime.second), logFile);

    //TODO: initiate threads and pass everything needed through (update to make sure have everything)
    //TODO: kalman thread
    thread kalman(kalmanThreadTEMP, ref(baroBuff), ref(imuBuff), ref(filteredBaroBuff), ref(filteredIMUBuff), ref(logFile), ref(baro_csvFile), ref(imu_csvFile));
    //alt v thread
    thread altV(altVelocityCalc, ref(filteredBaroBuff), ref(velocityFromBaroBuff), ref(launchTime), ref(logFile));
    //TODO: accel v thread
    //thread accelV(accelVelocityCalc, ref(filteredIMUBuff), ref(velocityFromIMUBuff), ref(logFile));
    //TODO: receiver thread- if necessary this is the one that can be cut and added to main thread
    //thread receiver(receiverThread, ref(logFile));

    //on main: continue to loop and collect data and check for landing
    //TODO: periodically log data to files so we don't lose everything if it dies midway
    bool landed = false;
    pair<time_t, long long> landingTime;
    while(!landed){
        //collect data
        pair<time_t, long long> timestamp = getTimeBoth();
        Baro_Raw b;
        IMU_Raw i;
        //don't poll barometer if it's dead
        if(barometerAlive){
            //collect data and add to buffer
            vector<float> baroData;
            try{
                baroData = getPressure();
            }
            catch(const runtime_error& err){
                //log barometer error
                //try to reboot barometer, otherwise log error and shut down barometer
                barometerAlive = false;
            }
            float alt = getAlt(baroData, seaLevel);
            b.time = timestamp.first;
            b.millisecs = timestamp.second;
            b.pressure = baroData[1];
            b.temperature = baroData[0];
            b.altitude = alt;
            baroBuff.push_back(b);

            //check if landing has occurred
            if(b.altitude < 20){  //below 20 meters
                //TODO: change condition of landing to be better- something about it no longer decreasing, as well as a threshold
                landed = true;
                landingTime = timestamp;
            }


        }
        //don't poll imu if it's dead
        if(imuAlive){
            //collect data and add to buffer
            vector<float> imuData = getAccelData();
            i.time = timestamp.first;
            i.millisecs = timestamp.second;
            i.imuX = imuData[3];
            i.imuY = imuData[4];
            i.imuZ = imuData[5];
            i.imuGyX = imuData[0];
            i.imuGyY = imuData[1];
            i.imuGyZ = imuData[2];
            imuBuff.push_back(i);

            //check if landing has occurred
            if(abs(i.imuX) < 1.5){ //TODO: find good way to tell that it has stopped moving- this is BAD indicator
                landed = true;
                landingTime = timestamp;
            }
        }
    }

    //----------------- Post Launch -------------------
    //log landing time
    addLogEntry("Landed at: " + getCurrentTimestamp(landingTime.first, landingTime.second), logFile);
    //start timer for 5 minutes from timestamp (check later while listening on receiver)

    //TODO: release parachute and antenna with servo

    //Get some data points:
        //temp of landing site
    float landingTemp = getTemp();
        //TODO: power status
        //TODO: stemnaut orientation
    vector<float> landedOrientation = getAccelData();
    //stemnautOrientation();


    //mosfet turn on transmitter (if need warm up time, else do this later)


    //join threads - TODO: FIGURE OUT HOW LONG THIS TAKES
    kalman.join(); //rn doesn't really do anything but log the raw data
    altV.join(); //
    //accelV.join();
    //receiver.join();
    cout << "joined threads" << endl;



    //TODO: find other data points:
        //apogee (max filtered alt) //TODO: do this live in one of the threads
        //max velocity (max of v) //TODO: also do this live in one of the threads
        //landing velocity- use timestamp of v
        //landing G-forces- use timestamp of FILTERED g-forces
        //run survival algo

    //TODO: format data points as text:
        //1. Temp of landing site
    addLogEntry("Landing temperature: " + to_string(landingTemp), logFile);
        //2. Apogee reached
        //3. Battery check/power status
        //4. Orientation of STEMnauts
        //5. Time of landing
        //6. Max velocity
        //7. Landing velocity, g-forces
        //8. STEMnaut crew survivability
    cout << "ending program" << endl;
    //TODO: begin transmitting

    //TODO: monitor for stop signal

    //TODO: when heard, turn it all off and end (after finish logging)


    return 0;
}


//helper functions and threads

void stemnautOrientation(IMU_Raw imuLanded){
    //TODO: calculate orientation based on imu data and known imu location relative to stemnauts

}

void receiverThread(ofstream& logFile) {
    addLogEntry("In receiver thread", logFile);
}
void kalmanThreadTEMP(list<Baro_Raw>& rawBaroData, list<IMU_Raw>& rawIMUData, list<Baro_Filtered>& filteredBaroVec, list<IMU_Filtered>& filteredIMUVec, ofstream& logFile, ofstream& dataFile, ofstream& imudatafile) {
    addLogEntry("In Kalman thread", logFile);
    list<Baro_Raw>::iterator it = rawBaroData.begin();
    list<IMU_Raw>::iterator imuIter = rawIMUData.begin(); //imu and baro should have same number of entries (assuming all is well...)
    string imuBuffer = "";
    bool reachedEnd = false;
    int processed = 0;
    string logBuffer = "";

    while(!reachedEnd){
        filteredBaroVec.emplace_back(it->time, it->millisecs, it->altitude,it->pressure, it->temperature, false);
        processed++;
        if(processed > 50){
            rawBaroData.pop_front(); //start removing from the front of the list
        }
        //TODO: put in buffer to log it
        logBuffer += toCsvString(*it) + "\n";
        imuBuffer += toCsvString(*imuIter) + "\n";
        if(processed % 20 == 0){
            writeToFile(dataFile, logBuffer);
            logBuffer = ""; //flush the buffer
            writeToFile(imudatafile, imuBuffer);
            imuBuffer = "";
        }

        //checking for if reached end
        if(it->nextExists){
            it++;
        }
        else if(!it->nextExists){
            this_thread::sleep_for(chrono::milliseconds(500)); //TODO: test this time delay
            if(!it->nextExists){
                reachedEnd = true;
                cout << "reached end for real" << endl;
                //finish logging
                writeToFile(dataFile, logBuffer);
            }
        }
        if(imuIter->nextExists){ //would have waited in previous step. super not safe but that's a spring problem
            imuIter++;
        }
    }
}
void altVelocityCalc(list<Baro_Filtered>& altData, list<Vel_Alt>& velVec, pair<time_t, long long>& launchTime, ofstream& logFile) {
    addLogEntry("In alt vel thread", logFile);
    list<Baro_Filtered>::iterator it = altData.begin();
    bool reachedEnd = false;
    it++; //gotta start at 2
    bool launched = false;
    int processed = 0;
    string logBuffer = "";
    string name = startupTime + "_filteredBaroData.csv";
    ofstream ffile(name);
    ffile << "Timestamp,Type,Alt,Temperature,Pressure\n";

    while(!reachedEnd){
        //cout << it->altitude << endl;
        //cout << it->time % 100 << "    |    " << launchTime % 100 << endl;
        //cout << getCurrentTimestamp(it->time, it->millisecs) << "    |    " << getCurrentTimestamp(launchTime, ms) << endl;
        //TODO: should be exact same time, shouldn't need the % 100, but should double check anyway
        if(!launched && it->time % 100 == launchTime.first % 100 && it->millisecs == launchTime.second){
            launched = true;
        }
        if(!launched){
            float vel = 0;
            velVec.emplace_back(it->time, it->millisecs, vel, false);

            if(velVec.size() > 1){
                prev(prev(velVec.end()))->nextExists = true;
            }

        }
        else{
            //(get<1>(heightData[i]) - get<1>(heightData[i - 1])) / difftime(get<0>(heightData[i]), get<0>(heightData[i - 1]))})
            float msDiff = it->millisecs - prev(it)->millisecs;
            //cout << msDiff << endl;
            if(msDiff < 0){
                //it crossed over the second boundary
                //cout << it->millisecs << "    " <<  prev(it)->millisecs;
                msDiff = 1000 + msDiff;
                //cout << msDiff << endl;
                //cout << msDiff / 1000 << endl;
            }
            //convert to feet
            float vel = ((it->altitude * 3.28084f) - (prev(it)->altitude * 3.28084f)) / (msDiff / 1000);
            //cout << vel << endl;
            velVec.emplace_back(it->time, it->millisecs, vel, false);
            if(velVec.size() > 1){
                prev(prev(velVec.end()))->nextExists = true;
            }
        }

        processed++;
        if(processed > 50){
            altData.pop_front(); //start removing from the front of the list
        }
        //put in buffer to log it
        logBuffer += toCsvString(*it) + "\n";
        if(processed % 20 == 0){
            writeToFile(ffile, logBuffer);
            logBuffer = ""; //flush the buffer
        }

        //checking for if reached end
        if(it->nextExists){
            it++;
        }
        else if(!it->nextExists){
            this_thread::sleep_for(chrono::milliseconds(500)); //TODO: test this time delay
            if(!it->nextExists){
                reachedEnd = true;
                cout << "reached end for real" << endl;
                //finish logging
                writeToFile(ffile, logBuffer);
            }
        }
    }

}
void accelVelocityCalc(vector<IMU_Filtered>& imuData, vector<Vel_Accel>& velVec, ofstream& logFile) {
    addLogEntry("in accel vel thread", logFile);
}
