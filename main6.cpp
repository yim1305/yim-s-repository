
/* NASA USLI 2025 Payload
 * CommuniGator Transmitter
 * H(igh frequency) O(mnidirectional) T(ransmitting) Pocket
 */
//includes
#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <chrono>
#include <ctime>
#include <functional>
#include <thread>
#include <pigpio.h>
#include <vector>
#include <list>
#include <sstream>
#include <string.h>
#include <regex>

#include "i2c.h"
#include "barometer.h"
#include "imu.h"
#include "voltageSensor.h"
//#include "GPIO.h"
#include "logger.h"
#include "receiver.h"
#include "kalmanFilter.h"
using namespace std;

#define BUFF_CAPACITY 150
#define FLIGHT_TIMEOUT 80 //80 s
#define GPS_FLUSHTIME 4
#define GPS_RCVBUF_SIZE 501
//#define LANDING_TIMEOUT 150

//define GPS Printing
void printGPS(int gpsHandle, char* gpsBuffer);

//void receiverThread(ofstream& logFile); //monitors for deployment signal, then servo legs open
void kalmanThreadTEMP(list<Baro_Raw>& rawBaroData, list<IMU_Raw>& rawIMUData, list<Baro_Filtered>& filteredBaroVec, list<IMU_Filtered>& filteredIMUVec, ofstream& logFile, ofstream& dataFile, ofstream& imudatafile, double& stemSurv); //takes in raw imu/baro vals
void altVelocityCalc(list<Baro_Filtered>& altData, list<Vel_Alt>& velVec, pair<time_t, long long>& launchTime, ofstream& logFile, float& apogee, float& maxV, float& landingV); //takes in filtered baro alt
void accelVelocityCalc(list<IMU_Filtered>& imuData, list<Vel_Accel>& velVec, ofstream& logFile, float& landingG); //takes in filtered imu
vector<double> stemnautOrientation(vector<float> data); //takes in filtered(?) accel data, returns pitch and roll
string readData(string lat, string lon, float temp, float apogee, float maxV, float landingV, float landingG, double pitch, double roll, float btry, int srv, pair<time_t, long long> landingTime);


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

    //set up the data buffer(s)- list of structs
        //initialize vector with size we will cut off to avoid reallocation time delays
    //List forms (faster, better buffering)
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
	    if(total != 0){
	        seaLevel = total / tally; // avg sea level over how many takes that make sense (no nonsense values)
            }
     	    addLogEntry("Sealevel: " + to_string(seaLevel), logFile);
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
    bool voltAlive = false;
    try{
        if(!checkVoltAlive()){
            addLogEntry("Voltage sensor down :(", logFile);
        }
        else{
            voltAlive = true;
            addLogEntry("Voltage sensor alive!", logFile);
        }
    }
    catch(const runtime_error& err){
        addLogEntry(err.what(), logFile);
    }

    if(gpioInitialise() < 0){
	addLogEntry("failed to init gpio", logFile);
    }

    //check receiver NOW
    int handle = spiOpen(0,38400,0);
    if(handle < 0){
        cout << "failed to open spi" << endl;
        addLogEntry("failed to open receiver spi bus", logFile);
    }
    if(!setupReceiver(handle)){
        cout << "failed to setup receiver" << endl;
        addLogEntry("failed to setup receiver", logFile);
    }
    else{
        cout << "success setting up receiver" << endl;
        addLogEntry("receiver setup", logFile);
    }

    char gpsPort[] = "/dev/ttyUSB0";
    char gpsBuffer[GPS_RCVBUF_SIZE];
    char transmitterPort[] = "/dev/ttyAMA0";
    char transmitterBuffer[80];
    int gpsHandle = serOpen(gpsPort, 4800, 0);
    if(gpsHandle < 0){
        addLogEntry("error opening GPS serial port", logFile);
    }
    else{
        addLogEntry("GPS port connected", logFile);
    }


    //maybe check the 'post launch' systems that will be turned off until landing? may forego this.
        //MOSFET systems- in order to test connections with the next three things, must use the mosfets to give them power
        //antenna servo motor- confirm connection?
        //parachute servo- put them into the locking position
        //radio transmitter/receiver- confirm connection/working?
        //close connections and use mosfets to STOP sending power to these systems

    int parachutePin = 6;
    int antennaPin = 5;
    int parachuteClose = 833;
    int antennaClose = 975;
    int parachuteOpen = 1800;
    int antennaOpen = 1350;
    gpioSetMode(parachutePin, PI_OUTPUT);
    gpioSetMode(antennaPin, PI_OUTPUT);
    //TODO: servo status 'dance' for status


    gpioServo(parachutePin, parachuteClose);
    gpioServo(antennaPin, antennaClose);

    //------------------ Pre Flight Loop --------------------
    //begin The Pre Flight Loop - checking imu and barometer data to see if launch has been detected
    cout << "waiting for launch" << endl;
    float groundAtLaunch = 0;
    bool launched = false;
    auto gpsTimerStart = chrono::steady_clock::now();


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
            b.temperature = (baroData[0] * 9/5) + 32;
            b.altitude = alt;
            baroBuff.push_back(b);
            //set nextexists to true on previous one so it knows there's more
            if(baroBuff.size() > 1){
                prev(prev(baroBuff.end()))->nextExists = true;
            }

            //delete first [couple] elements if at capacity (1000)
            if(!launched && baroBuff.size() > BUFF_CAPACITY){ //.size is constant complexity
                baroBuff.pop_front();
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

	    if(imuBuff.size() > 1){
                prev(prev(imuBuff.end()))->nextExists = true;
            }
            //delete old buffer values
            if(!launched && imuBuff.size() > BUFF_CAPACITY){
                imuBuff.pop_front();
                //same as barometer reasoning
            }

        // GPS
        printGPS(gpsHandle, gpsBuffer);

        }
/*        //check if launch has occurred
        if(barometerAlive && b.altitude > 25){  //was 25. above 2.5 ft launch pad + ~10ft rocket = 10ft feet- with past 100 data points, will still have some prelaunch data
            //TODO: change condition of launch to be better- something about rapid increase, as well as the threshold
            launched = true;
        }
        //check if launch has occurred
        if(imuAlive && barometerAlive && abs(i.imuX) > 5 && b.altitude > 15){ //greater than 2 Gs (based on subscale), but also higher than x feet because there could be spikes
            launched = true;
        }
*/
//REAL launch conditions

        //check if launch has occurred
        if(barometerAlive && b.altitude > 35){  //above 30 feet- with past 100 data points, will still have some prelaunch data
            //TODO: change condition of launch to be better- something about rapid increase, as well as the threshold
            auto iter = prev(baroBuff.end());
            int c = 0;
	    int over = 0;
	    iter--;
	    if(b.altitude > 200){
		over++;
	    }
	    if(b.altitude - iter->altitude <= 30){
               for(int i=0; i<100; i++){ //check for 30 ft difference in points
                   if(b.altitude - iter->altitude >= 30){
                       c++; //haha
                   }
		 //  if(iter->altitude > 200){
		  //     over++;
	   	  // }
                   iter--;
               }
	    }
	   iter = prev(baroBuff.end());
	   for(int i=0; i<100;i++){
		if(iter->altitude > 200){
			over++;
		}
		iter--;
	   }
           if(c >= 15 || over >= 95){
                   launched = true;
	   }

        }
        //check if launch has occurred
        if(imuAlive && abs(i.imuX) > 7){ //greater than 2 Gs (based on subscale), but also higher than x feet because there could be spikes
            int c = 0;
            auto iter = prev(imuBuff.end());
            for(int i=0; i<30; i++){
                if(abs(iter->imuX) > 7){
                    c++;
                }
		iter--;
            }
            if(c > 20){
                launched = true;
            }
        }

	if(launched){
	   cout << "launched" << endl;
	}
        //check if time to flush the gps buffer
        auto gpsCurrentTimer = chrono::steady_clock::now();
        auto gpsTimePassed = chrono::duration_cast<chrono::seconds>(gpsCurrentTimer - gpsTimerStart);
        if(gpsTimePassed.count() >= GPS_FLUSHTIME && gpsHandle >= 0){
            //read everything from the buffer
            int bufferSize = serDataAvailable(gpsHandle);
            while(bufferSize > 0){ //pls never infinite loop- it should not but... its a while loop. if port fails while doing this should prob have a condition
                if(bufferSize >= GPS_RCVBUF_SIZE){
                    int read = serRead(gpsHandle, gpsBuffer, GPS_RCVBUF_SIZE-1); //fill our receive buffer
                    bufferSize -= read;
                }
                else{
                    int read = serRead(gpsHandle, gpsBuffer, bufferSize);
                    bufferSize -= read;
                }
            }
            gpsTimerStart = gpsCurrentTimer;
        }


    }

    groundAtLaunch = baroBuff.front().altitude;


    //----------------- Flight -------------------
    //record flight time and log it
    pair<time_t, long long> launchTime = getTimeBoth();
    //log it
    addLogEntry("Launched at: " + getCurrentTimestamp(launchTime.first, launchTime.second), logFile);
    float maxAlt = 0;
    float maxV = 0;
    double survivalRate = 100;
    float landingV = 0;
    float landingG = 0;
    bool releaseParachute = false;


    //TODO: initiate threads and pass everything needed through (update to make sure have everything)
    //TODO: kalman thread
    thread kalman(kalmanThreadTEMP, ref(baroBuff), ref(imuBuff), ref(filteredBaroBuff), ref(filteredIMUBuff), ref(logFile), ref(baro_csvFile), ref(imu_csvFile), ref(survivalRate));
    //alt v thread
    thread altV(altVelocityCalc, ref(filteredBaroBuff), ref(velocityFromBaroBuff), ref(launchTime), ref(logFile), ref(maxAlt), ref(maxV), ref(landingV));
    //TODO: accel v thread
    thread accelV(accelVelocityCalc, ref(filteredIMUBuff), ref(velocityFromIMUBuff), ref(logFile), ref(landingG));


    //on main: continue to loop and collect data and check for landing
    //TODO: periodically log data to files so we don't lose everything if it dies midway
    bool landed = false;
    pair<time_t, long long> landingTime;
    int count = 100;
    auto flightTimerStart = chrono::steady_clock::now();
    gpsTimerStart = chrono::steady_clock::now();


    while(!landed || count > 0){
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
            b.temperature = (baroData[0]*9/5) + 32;
            b.altitude = alt;
            baroBuff.push_back(b);
	    if(baroBuff.size() > 1){
                prev(prev(baroBuff.end()))->nextExists = true;
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
	    if(imuBuff.size() > 1){
                prev(prev(imuBuff.end()))->nextExists = true;
            }
        }

    // GPS
    printGPS(gpsHandle, gpsBuffer);

	auto currentTimer = chrono::steady_clock::now();
        auto flightTime = chrono::duration_cast<chrono::seconds>(currentTimer - flightTimerStart);
        cout << flightTime.count() << endl;
        if(flightTime.count() > FLIGHT_TIMEOUT && maxAlt > 4000){ //longer than 60 seconds, have flown above 4000 ft (60 for testing)
            //check if landing has occurred
            if(b.altitude < groundAtLaunch + 20 && (!landed || count >= 80)){  //TODO: replace landing threshold with ground at launch? that assumes landing on near flat ground, but is better than current...
                //TODO: change condition of landing to be better- something about it no longer decreasing, as well as a threshold CHECK NEW CODE
                //check past couple points
                cout << "potential baro landing" << endl;
                cout << b.altitude << endl;
                auto iter = prev(baroBuff.end());
                int countDown = 0;
                for(int j=0; j<300; j++){
                    if(iter->altitude > groundAtLaunch){
                        break;
                    }
                    else if(abs(iter->altitude - baroBuff.back().altitude) > 5){ //greater than 5ft diff in 10 datapoints = still moving
                        break;
                    }
                    iter--;
                    countDown++;
                }
                if(countDown >= 295){
                    landed = true;
                    landingTime = timestamp;
		    releaseParachute = true;
                    cout << "landed for baro yes parachute" << endl;
                }
		else if(countDown >= 290){
                    landed = true;
                    landingTime = timestamp;
                    cout << "landed for baro no parachute" << endl;
                }
            }
                //check if landing has occurred
                //all imu axis less than or about == 1g indicates no movement ?
                //g = sqrt(i.imuX * i.imuX + i.imuY * i.imuY + i.imuZ * i.imuZ);
            else if((!landed || count >= 80) && sqrt(i.imuX * i.imuX + i.imuY * i.imuY + i.imuZ * i.imuZ) < 1.1 && b.altitude < -100){ //TODO: test that this isn't horrible. this will basically never trigger.
                auto iter = prev(imuBuff.end());
                cout << "potential imu landing" << endl;
                cout << sqrt(i.imuX * i.imuX + i.imuY * i.imuY + i.imuZ * i.imuZ) << endl;
                int countDown = 0;
                for(int j=0; j<300; j++){
                    if(sqrt(iter->imuX * iter->imuX + iter->imuY * iter->imuY + iter->imuZ * iter->imuZ) > 1.1){
                        break;
                    }
                    iter--;
                    countDown++;
                }

                if(countDown >= 300){
                    landed = true;
                    landingTime = timestamp;
                    cout << "landed for imu" << endl;
		    releaseParachute = true;
                }
		else if(countDown >=290){
                    landed = true;
                    landingTime = timestamp;
                    cout << "landed for imu no parachute" << endl;
                }

            }
            if(landed){
                count--;
            }
        }
        //check for flushing gps buffer
        auto gpsCurrentTime = chrono::steady_clock::now();
        auto gpsTimePassed = chrono::duration_cast<chrono::seconds>(gpsCurrentTime - gpsTimerStart);
        if(gpsTimePassed.count() >= GPS_FLUSHTIME && gpsHandle >= 0){
            int bufferSize = serDataAvailable(gpsHandle);
            while(bufferSize > 0){ //figure out if this can ever infinite loop
                if(bufferSize >= GPS_RCVBUF_SIZE){
                    int read = serRead(gpsHandle, gpsBuffer, GPS_RCVBUF_SIZE-1);
                    bufferSize -= read;
                }
                else{
                    int read = serRead(gpsHandle, gpsBuffer, bufferSize);
                    bufferSize -= read;
                }
            }
            gpsTimerStart = gpsCurrentTime;
        }
  /*      if(!landed && flightTime.count() > LANDING_TIMEOUT){
            landed = true;
            landingTime = timestamp;
            cout << "timeout landing" << endl;
        }*/


    }
    cout << "landed" << endl;
    //----------------- Post Launch -------------------
    //log landing time
    addLogEntry("Landed at: " + getCurrentTimestamp(landingTime.first, landingTime.second), logFile);
    //start timer for 5 minutes from timestamp (check later while listening on receiver)
    auto timerStart = chrono::steady_clock::now();


    //TODO: release parachute  and antenna with servo
    cout << releaseParachute << endl;
    //releaseParachute = false;
    if(releaseParachute){
        gpioServo(parachutePin, parachuteOpen);
    }
    this_thread::sleep_for(chrono::seconds(1));
    gpioServo(antennaPin, antennaOpen);

    //Get some data points:
        //temp of landing site
    float landingTemp = (getTemp() * 9/5) + 32;
        //power status - convert to percent
    float voltage = getVoltage();
    //float current = getCurrent();
    //string msg = "Voltage: " + to_string(voltage) + "\nCurrent: " + to_string(current);
    int battery;
    cout << voltage << endl;
    if(abs(voltage) < 5.0 && abs(voltage) > 4.2){
        battery = int((abs(voltage) - 4.2) / 0.008);
    }

        //stemnaut orientation

    float avgLandedX = 0;
    float avgLandedY = 0;
    float avgLandedZ = 0;
    for(int i=0; i<50; i++){
        vector<float> landed = getAccelData();
        avgLandedX += landed[3];
        avgLandedY += landed[4];
        avgLandedZ += landed[5];
    }
    avgLandedX = avgLandedX / 50.0f;
    avgLandedY = avgLandedY / 50.0f;
    avgLandedZ = avgLandedZ / 50.0f;
    vector<float> landedOrientation;
    landedOrientation.push_back(avgLandedX);
    landedOrientation.push_back(avgLandedY);
    landedOrientation.push_back(avgLandedZ);
    vector<double> pitchRoll = stemnautOrientation(landedOrientation);

    //mosfet turn on transmitter (if need warm up time, else do this later)
    int mosfetPin = 27;
    gpioSetMode(mosfetPin, PI_OUTPUT);
    gpioWrite(mosfetPin, PI_ON); //give power
//    gpioSetMode(mosfetPin, PI_OFF);
    //join threads - TODO: FIGURE OUT HOW LONG THIS TAKES
    kalman.join(); //rn doesn't really do anything but log the raw data
    altV.join(); //
    accelV.join();
    //receiver.join();
    cout << "joined threads" << endl;
    addLogEntry("Threads Joined", logFile);

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
    addLogEntry("Apogee Recorded as: " + to_string(maxAlt), logFile);
        //3. Battery check/power status
    addLogEntry("Battery Status: " + to_string(voltage), logFile);
        //4. Orientation of STEMnauts
    addLogEntry("Pitch: " + to_string(pitchRoll.at(0)) + ", roll: " + to_string(pitchRoll.at(1)), logFile);
        //5. Time of landing
    addLogEntry("Time of landing: " + getCurrentTimestamp(landingTime.first, landingTime.second), logFile);
        //6. Max velocity
    addLogEntry("Maximum V: " + to_string(maxV), logFile);
        //7. Landing velocity, g-forces
    addLogEntry("Landing V: " + to_string(landingV), logFile);
    addLogEntry("Landing gs: " + to_string(landingG), logFile);
        //8. STEMnaut crew survivability
    addLogEntry("Stemnaut Survival Rate: " + to_string(survivalRate), logFile);

    //get nmea data
    string lat = "NOLOCK";
    string lon = "NOLOCK";
    //TODO: get gps data
    if(gpsHandle >= 0){
        //
        int available = serDataAvailable(gpsHandle);
        int read = 0;
        int gpsCount = 5; //try 5 times
        while(read <= 0 && gpsCount > 0){ //make sure we get something but don't infinite loop
            if(available < GPS_RCVBUF_SIZE){
                read = serRead(gpsHandle, gpsBuffer, available);

            }
            else{
                read = serRead(gpsHandle, gpsBuffer, GPS_RCVBUF_SIZE-1);
            }
            gpsCount--;
        }

        //now, what to do with the data- parse it
        string gpsmsg = string(gpsBuffer);
        stringstream gpss(gpsmsg);
        string gpsLine;
        if(gpsmsg.size() > 0 && gpsmsg.at(0) == '$'){
            getline(gpss, gpsLine, '$');
        }
        regex gpggaRegex("GPGGA,\\d{6}\\.\\d{3},(\\d{4}\\.\\d{4}),[NS],(\\d{5}\\.\\d{4}),[WE](,[\\d\\w\\.\\*-]*)*");
        while(getline(gpss, gpsLine, '$')){
            //if(gpsLine.size() >= 37 && gpsLine.substr(0,5) == "GPGGA"){
            if(regex_search(gpsLine, gpggaRegex)){
                cout << "matches regex, gpgga: " << gpsLine << endl;
                //what we need
                //format: $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
                //         1234567890123456789012345678901234567890
                //if no lock, will just be comma separated (nothing in commas)
                stringstream ggass(gpsLine);
                string garbage;
                string tempLat;
                string tempLon;
                getline(ggass, garbage, ','); //GPGGA
                getline(ggass, garbage, ','); //timestamp hhmmss.ss
                getline(ggass, tempLat, ','); //latitude llll.llll
		getline(ggass, garbage, ',');
                getline(ggass, tempLon, ','); //lon yyyyy.yyyy
                lat = tempLat.substr(0,7);
                lon = tempLon.substr(1,7);
            }
            else{
                //garbage
                cout << "garbage gps: " << gpsLine << endl;
            }
        }

    }



    string toSend = readData(lat, lon, landingTemp, maxAlt, maxV, landingV, landingG, pitchRoll.at(0), pitchRoll.at(1), voltage, survivalRate, landingTime);
    addLogEntry(toSend, logFile);
    cout << "compiled string: " << toSend << endl;

    //cout << "ending program" << endl;
    //TODO: begin transmitting
//    char sendBuf[80];
//    strcpy(sendBuf, toSend.c_str());

//try this:
    this_thread::sleep_for(std::chrono::seconds(2));
    strcpy(transmitterBuffer, toSend.c_str());
    int sizeofTransmission = toSend.size();
    //send to transmitter
    int transHandle = serOpen(transmitterPort, 9600, 0);
    bool transmitterAlive = false;
    if(transHandle < 0){
        addLogEntry("transmitter port failed to open", logFile);
    }
    else{
        transmitterAlive = true;
        int written = serWrite(transHandle, transmitterBuffer, sizeofTransmission);
        if(written != 0){
            cout << "error writing" << endl;
            addLogEntry("error writing to transmitter", logFile);
        }
        else{
            cout << "success writing " << string(transmitterBuffer) << endl;
        }
    }


    //TODO: monitor for stop signal- adjust if needed
    //listen for stop
    auto txTimerStart = chrono::steady_clock::now();
    if(handle >= 0){
       bool stopped = false;
       bool timerEnd = false;
       string rcvmsg = "";
       while(!stopped && !timerEnd){ //add condition of 5 minutes
           rcvmsg = recv(handle); //returns "timeout" in a timeout
           //rcvmsg should contain message if one was recevied.
	   if(rcvmsg == "STOP"){
		stopped = true;
	   }
           auto timerNow = chrono::steady_clock::now();
           auto diff = chrono::duration_cast<chrono::seconds>(timerNow - timerStart);
           if(diff.count() >= 300){ //it has been 5 minutes = 300 s
               stopped = true;
	       addLogEntry("Timed out of receiver", logFile);
           }
	   if(transHandle >= 0){
	       auto txDiff = chrono::duration_cast<chrono::seconds>(timerNow - txTimerStart);
	       if(txDiff.count() >= 30){
		   int written = serWrite(transHandle, transmitterBuffer, sizeofTransmission);
		   txTimerStart = timerNow;
	       }
	   }
       }
    }
    gpioServo(antennaPin, antennaClose);

    this_thread::sleep_for(chrono::seconds(5));


    //TODO: when heard, turn it all off and end (after finish logging)
    gpioWrite(mosfetPin, PI_OFF);

    logFile.close();
    baro_csvFile.close();
    imu_csvFile.close();
    serClose(gpsHandle);
    serClose(transHandle);
    spiClose(handle);
    gpioTerminate();


    cout << "ending program" << endl;

    return 0;
}


//helper functions and threads

void printGPS(int gpsHandle, char* gpsBuffer) {
    if(gpsHandle < 0) return; // no gps
    
    int available = serDataAvailable(gpsHandle);
    if(available <= 0) return; // nothing new

    int read = 0;
    if(available < GPS_RCVBUF_SIZE) {
        read = serRead(gpsHandle, gpsBuffer, available);
    } else {
        read = serRead(gpsHandle, gpsBuffer, GPS_RCVBUF_SIZE - 1);
    }
    if(read <= 0) return;

    // parse NMEA
    string gpsmsg = string(gpsBuffer, read);
    stringstream gpss(gpsmsg);
    string gpsLine;
    regex gpggaRegex("GPGGA,\\d{6}\\.\\d{2,},(\\d{4}\\.\\d+),[NS],(\\d{5}\\.\\d+),[EW](,[\\d\\w\\.\\*-]*)*");

    while(getline(gpss, gpsLine, '$')) {
        if(regex_search(gpsLine, gpggaRegex)) {
            stringstream ggass(gpsLine);
            string garbage, tempLat, tempLon;
            getline(ggass, garbage, ','); // GPGGA
            getline(ggass, garbage, ','); // timestamp
            getline(ggass, tempLat, ','); // latitude
            getline(ggass, garbage, ','); // N/S
            getline(ggass, tempLon, ','); // longitude

            cout << "GPS -> Lat: " << tempLat << " Lon: " << tempLon << endl;
            return; // only print first valid one
        }
    }
}


vector<double> stemnautOrientation(vector<float> data){
    //TODO: calculate orientation based on imu data and known imu location relative to stemnauts
    //vector<float> data = getAccelData();
    double Pitch = atan2(-data[1], sqrt(data[0]*data[0] + data[2]*data[2]));
    double Roll = atan2(-data[2], data[0]);  // Using atan2 for proper quadrant handling // data2 negative cuz imu upside down
    double PitchDegrees = Pitch * (180.0/M_PI);
    double RollDegrees = Roll * (180.0/M_PI);
    vector<double> result;
    result.push_back(PitchDegrees);
    result.push_back(RollDegrees);
    return result;
}

//void receiverThread(ofstream& logFile) {
//    addLogEntry("In receiver thread", logFile);
//}
void kalmanThreadTEMP(list<Baro_Raw>& rawBaroData, list<IMU_Raw>& rawIMUData, list<Baro_Filtered>& filteredBaroVec, list<IMU_Filtered>& filteredIMUVec, ofstream& logFile, ofstream& dataFile, ofstream& imudatafile, double& stemSurv) {
//    addLogEntry("In Kalman thread", logFile);
    list<Baro_Raw>::iterator it = rawBaroData.begin();
    list<IMU_Raw>::iterator imuIter = rawIMUData.begin(); //imu and baro should have same number of entries (assuming all is well...)
    string imuBuffer = "";
    bool reachedEnd = false;
    int processed = 0;
    string logBuffer = "";
    KalmanFilter kf;

    float last50 = 0;
    float imuxLast50 = 0;
    float imuyLast50 = 0;
    float imuzLast50 = 0;
    pair<time_t, long long> startGs;
    pair<time_t, long long> endGs;
    bool midGs = false;
    bool calc = false;
    while(!reachedEnd){
        //process the barometer data
        last50 += it->altitude;
        processed++;
        /*if(processed >= 51) {
            last50 = last50 - rawBaroData.front().altitude;
            filteredBaroVec.emplace_back(it->time, it->millisecs, last50/50.0f, it->pressure, it->temperature, false);
        }
        else{
            filteredBaroVec.emplace_back(it->time, it->millisecs, last50/processed, it->pressure, it->temperature, false);
        }*/

        //just moving it raw
        //filteredBaroVec.emplace_back(it->time, it->millisecs, it->altitude,it->pressure, it->temperature, false);
        kf.update(it->altitude, it->time, it->millisecs);
        filteredBaroVec.emplace_back(it->time, it->millisecs, kf.getPosition(), it->pressure, it->temperature, kf.getVelocity(), false);

        //TODO: imu filtering- bad, just get rid of it for flight
        imuxLast50 = imuIter->imuX;
        imuyLast50 = imuIter->imuY;
        imuzLast50 = imuIter->imuZ;
        if(processed >= 10){
           // imuxLast50 = imuxLast50;// - rawIMUData.front().imuX;
           // imuyLast50 = imuyLast50;/// - rawIMUData.front().imuY;
           // imuzLast50 = imuzLast50;// - rawIMUData.front().imuZ;
            filteredIMUVec.emplace_back(imuIter->time, imuIter->millisecs, imuxLast50, imuyLast50, imuzLast50, imuIter->imuGyX, imuIter->imuGyY, imuIter->imuGyZ, false);
        }
        else{
            filteredIMUVec.emplace_back(imuIter->time, imuIter->millisecs, imuxLast50, imuyLast50, imuzLast50, imuIter->imuGyX, imuIter->imuGyY, imuIter->imuGyZ, false);
        }


        //remove from queue after 50 have been processed to move up the front marker
	if(filteredBaroVec.size() > 1){
            prev(prev(filteredBaroVec.end()))->nextExists = true;
	}
        if(filteredIMUVec.size() > 1){
            prev(prev(filteredIMUVec.end()))->nextExists = true;
        }

//TODO: stemnaut survival- make g value matter more, tune affected amount
        //check gs  float g = sqrt(i->imuX * i->imuX + i->imuY * i->imuY + i->imuZ * i->imuZ);
        float gs = sqrt(imuIter->imuX * imuIter->imuX + imuIter->imuY * imuIter->imuY + imuIter->imuZ * imuIter->imuZ);
        float prevGs = sqrt(prev(imuIter)->imuX * prev(imuIter)->imuX + prev(imuIter)->imuY * prev(imuIter)->imuY + prev(imuIter)->imuZ * prev(imuIter)->imuZ);
    /*    if(gs > 7){
            if(!midGs){
                startGs = make_pair(imuIter->time, imuIter->millisecs);
                midGs = true;
            }

        }
        else if (gs < 7 && prevGs > 7){
            //do the calcs
            endGs = make_pair(imuIter->time, imuIter->millisecs);
            midGs = false;

            double difTime = difftime(endGs.first, startGs.first);
            difTime += ((double)endGs.second - (double)startGs.second) / 1000;
            cout << difTime << endl;
	    if(difTime > 1){
                stemSurv -= (difTime * prevGs) / 7;
            }
            else{
                stemSurv -= (difTime * prevGs);
            }

        }
*/
//alternate stemnaut calc
	if(prevGs < 7 && gs >= 7){
		cout << "start over 7 gs" << endl;
		midGs = true;
		startGs = make_pair(imuIter->time, imuIter->millisecs);
	}
	else if(prevGs >= 7 && gs < 7 && midGs){
		midGs = false;
		cout << "stopped being over 7gs" << endl;
		endGs = make_pair(imuIter->time, imuIter->millisecs);
		calc = true;
	}
	if(calc){

		double difTime = difftime(endGs.first, startGs.first);
		difTime += ((double)endGs.second - (double)startGs.second) / 1000.0;
		if(difTime < 100){
			cout << "other diftime: " << difTime << endl;
 			stemSurv -= difTime * gs * 1.5;
			if(stemSurv <= 0){
				stemSurv = 0;
			}
		}
		calc = false;
		cout << "calculated dif: " << difTime << "stemsurv: " << stemSurv << endl;
	}

//check gs
/*        if(imuIter->imuX > 7 || imuIter->imuY > 7 || imuIter->imuZ > 7){
            if(prev(imuIter)->imuX < 7 && prev(imuIter)->imuY < 7 && prev(imuIter)->imuZ < 7){
                //calculate damage and reset count
		if(gLowCount > 100){
                	stemSurv -= gLowCount / 100;
		}
		else{
			stemSurv -= gLowCount / 25;
		}
                gLowCount = 0;
            }
            gLowCount++;
        }
        else if(imuIter->imuX > 9 || imuIter->imuY > 9 || imuIter->imuZ > 9){
            if(prev(imuIter)->imuX < 9 && prev(imuIter)->imuY < 9 && prev(imuIter)->imuZ < 9){
                stemSurv -= gLowCount / 75;
                gHighCount = 0;
            }
            gHighCount++;

        }*/
        if(processed > 350){
            rawBaroData.pop_front(); //start removing from the front of the list
	    rawIMUData.pop_front();
        }
        //TODO: put in buffer to log it
        logBuffer += toCsvString(*it) + "\n";
        imuBuffer += toCsvString(*imuIter) + "\n";
        if(processed % 20 == 0){
//	    cout << logBuffer << endl;
//	    cout << imuBuffer << endl;
            writeToFile(dataFile, logBuffer);
            logBuffer = ""; //flush the buffer
            writeToFile(imudatafile, imuBuffer);
            imuBuffer = "";
        }

        //checking for if reached end
        if(!it->nextExists){
            this_thread::sleep_for(chrono::milliseconds(500)); //TODO: test this time delay
            if(!it->nextExists){
                reachedEnd = true;
                cout << "reached end for real" << endl;
                //finish logging
                writeToFile(dataFile, logBuffer);
		writeToFile(imudatafile, imuBuffer);
            }
            else{
                it++;
            }
        }
        else{
            it++;
        }
	//never reaching this code
        if(imuIter->nextExists){ //would have waited in previous step. super not safe but that's a spring problem
            imuIter++;
//	    cout << "imu next exists" << endl;
        }
    }
}

void altVelocityCalc(list<Baro_Filtered>& altData, list<Vel_Alt>& velVec, pair<time_t, long long>& launchTime, ofstream& logFile, float& apogee, float& maxV, float& landingV) {
    this_thread::sleep_for(chrono::seconds(2));
  //  addLogEntry("In alt vel thread", logFile);
    list<Baro_Filtered>::iterator it = altData.begin();
    bool reachedEnd = false;
    it++; //gotta start at 2
    bool launched = true;
    int processed = 0;
    string logBuffer = "";
    string name = "logs/" + startupTime + "_filteredBaroData.csv";
    ofstream ffile(name);
    ffile << "Timestamp,Type,Alt,Temperature,Pressure,Velocity,kfVel\n";

    while(!reachedEnd){
        //cout << it->altitude << endl;
        //cout << it->time % 100 << "    |    " << launchTime % 100 << endl;
        //cout << getCurrentTimestamp(it->time, it->millisecs) << "    |    " << getCurrentTimestamp(launchTime, ms) << endl;
        //TODO: should be exact same time, shouldn't need the % 100, but should double check anyway
        if(!launched && it->time >= launchTime.first  && it->millisecs >= launchTime.second){
            launched = true;
	    cout << "launched alt thread" << endl;
        }
        if(!launched){
	    //cout << "not launched" << endl;
            float vel = 0;
            velVec.emplace_back(it->time, it->millisecs, vel, false);

            if(velVec.size() > 1){
                prev(prev(velVec.end()))->nextExists = true;
            }

        }
        else{
            //(get<1>(heightData[i]) - get<1>(heightData[i - 1])) / difftime(get<0>(heightData[i]), get<0>(heightData[i - 1]))})
            float msDiff = it->millisecs - prev(it)->millisecs;
//            cout << msDiff << endl;
            if(msDiff < 0){
                //it crossed over the second boundary
                //cout << it->millisecs << "    " <<  prev(it)->millisecs;
                msDiff = 1000 +  msDiff;
                //cout << msDiff << endl;
                //cout << msDiff / 1000 << endl;
            }
            //convert to feet
            float vel = ((it->altitude) - (prev(it)->altitude)) / (msDiff/1000);
            //cout << vel << endl;
            velVec.emplace_back(it->time, it->millisecs, vel, false);
            if(velVec.size() > 1){
                prev(prev(velVec.end()))->nextExists = true;
            }
        }

        processed++;
        if(processed > 410){
            altData.pop_front(); //start removing from the front of the list
  	    velVec.pop_front();
        }
        //check max alt
        if(processed > BUFF_CAPACITY - 50 && it->altitude > apogee){
            apogee = it->altitude;
        }
        //check max v
        if(processed > BUFF_CAPACITY - 50 && it->kfVel > maxV){
            maxV = it->kfVel;
        }

        //put in buffer to log it
        stringstream ss;
        ss << fixed << setprecision(6) << velVec.back().velocity;
        string t = ss.str();
        stringstream sb;
        sb << fixed << setprecision(6) << it->kfVel;
        string kfV = sb.str();
        logBuffer += toCsvString(*it) + "," + t + "," + kfV + "\n";


        if(processed % 20 == 0){
            writeToFile(ffile, logBuffer);
            logBuffer = ""; //flush the buffer
        }

        //checking for if reached end
        if(!it->nextExists){
            this_thread::sleep_for(chrono::milliseconds(1000)); //TODO: test this time delay
            if(!it->nextExists){
                reachedEnd = true;
                cout << "reached end for real" << endl;
                //finish logging
                writeToFile(ffile, logBuffer);
		ffile.close();
 		auto i = altData.begin();
                while(i != altData.end()){
	 	    cout << "kfvel: " << i->kfVel << "  landingv: " << landingV << endl;
                    if(abs(i->kfVel) > landingV){
                        landingV = abs(i->kfVel);
                    }
                    i++;
                }
            }
            else{
                it++;
            }
        }
        else{
            it++;
        }
    }

}

void accelVelocityCalc(list<IMU_Filtered>& imuData, list<Vel_Accel>& velVec, ofstream& logFile, float& landingG) {
  //  addLogEntry("in accel vel thread", logFile);
    this_thread::sleep_for(chrono::seconds(2));
    list<IMU_Filtered>::iterator it = imuData.begin();
    bool reachedEnd = false;
    it++;
    int processed = 0;
    string logBuffer = "";
    string name = "logs/" + startupTime + "_filteredIMUData.csv";
    ofstream ffile(name);
    ffile << "Timestamp,AccelX,AccelY,AccelZ\n";

    while(!reachedEnd){
        //calculate v

        processed++;
        if(processed > 350){
            imuData.pop_front();
        }

        //log it
        logBuffer += toCsvString(*it) + "\n";
        if(processed % 20 == 0){
            writeToFile(ffile, logBuffer);
            logBuffer = "";
        }

        //check if reached end
        if(!it->nextExists){
            this_thread::sleep_for(chrono::milliseconds(1000));
            if(!it->nextExists){
                reachedEnd = true;
                cout << "reached end of imufiltered for real" << endl;
                writeToFile(ffile, logBuffer);
		ffile.close();
                //TODO: kept last 50, can go through for landing Gs
                auto i = imuData.begin();
                while(i != imuData.end()) {
                    float g = sqrt(i->imuX * i->imuX + i->imuY * i->imuY + i->imuZ * i->imuZ);
                    if (g > landingG) {
                        landingG = g;
                    }
                    i++;
                }
            }
            else{
                it++;
            }
        }
        else{
            it++;
        }

    }

}


string readData(string lat, string lon, float temp, float apogee, float maxV, float landingV, float landingG, double pitch, double roll, float btry, int srv, pair<time_t, long long> landingTime){
    //0: lat and lon
    string data = ":KQ4ZJC___:";
    data += lat + "," + lon + ",";

    //1: time of landing in HHMMSS
    string t = getCurrentTimestamp(landingTime.first, landingTime.second);
    //2025-02-14_19:45:11.725 is YYYY-MM-DD_HH:MM:SS.sss
    //---------------------------01234567890123456789012
    data += t.substr(11,2) + t.substr(14,2) + t.substr(17,2) + ",";

    //2: Apogee in xxxx ft
    t = to_string(abs((int)apogee));
    while(t.length() < 4){
        t = "0" + t;
    }
    if(t.length() > 4){
        t = "9999";
    }
    data += t + ",";

    //3: max vel in xxx ft/s
    t = to_string(abs((int)maxV));
    while(t.length() < 3){
        t = "0" + t;
    }
    if(t.length() > 3){ //if too much just make it fit 3 digits
        t = "999";
    }
    data += t + ",";

    //4: landing v in xxx ft/s
    t = to_string(abs((int)landingV));
    while(t.length() < 3){
        t = "0" + t;
    }
    if(t.length() > 3){
        t = "999";
    }
    data += t + ",";

    //4.5: landing g in xx.x G
    stringstream ss;
    ss << std::fixed << std::setprecision(1) << abs(landingG);
    t = ss.str();
    while(t.length() < 4){
        t = "0" + t;
    }
    data += t + ",";

    //5: battery voltage in x.xx V
    stringstream st;
    st << fixed << setprecision(2) << abs(btry);
    t = st.str();
    data += t + ",";

    //6: pitch in sxxx deg
    t = to_string(abs((int)(pitch)));
    while(t.length() < 3){
        t = "0" + t;
    }
    if(pitch < 0){
        t = "-" + t;
    }
    else{
        t = "+" + t;
    }
    data += t + ",";

    //6.5: roll in sxxx deg
    t = to_string(abs((int)(roll)));
    while(t.length() < 3){
        t = "0" + t;
    }
    if(roll < 0){
        t = "-" + t;
    }
    else{
        t = "+" + t;
    }
    data += t + ",";

    //7: survival percentage in xxx %
    t = to_string(srv);
    while(t.length() < 3){
        t = "0" + t;
    }
    data += t + ",KQ4ZJC\r";

    return data;
}
