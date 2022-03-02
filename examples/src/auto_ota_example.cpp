#include "dot_util.h"
#include "RadioEvent.h"
#include "BME280.h"

#if ACTIVE_EXAMPLE == AUTO_OTA_EXAMPLE

mDot *dot = NULL;
lora::ChannelPlan *plan = NULL;

// LED
DigitalOut redLED(PB_2);
// SERIAL
Serial pc(USBTX, USBRX);
// 89BSD
I2C i2c(I2C_SDA, I2C_SCL);
// BME280
BME280 sensor(I2C_SDA, I2C_SCL);

// LORAWAN CONFIGURATION
static uint8_t network_id[] = {0x00,0x25,0x0C,0x00,0x00,0x01,0x00,0x01};
static uint8_t network_key[] = {0xE3,0x3E,0x51,0xA7,0x40,0x56,0x2B,0xA6,0xD3,0x97,0x4C,0x40,0x2C,0xAD,0xAB,0x80};
static uint8_t frequency_sub_band = 1;
static bool public_network = true;
static uint8_t join_delay = 5;
static uint8_t ack = 1;
static bool adr = true;

//Function Prototypes
void mainControl(RadioEvent *);
void updateProgramBehavior(uint32_t);
void sensorReading1(char cmd[], char readADC[]);
void sensorReading2(char cmd[]);
int numberCheck1(int16_t coefficientCheck1);
int numberCheck2(int16_t coefficientCheck2);
bool get89BSD();
bool performGet();
bool getBME280();
void sleepxDot(int);
void deepSleepxDot(int);
void flashLED();
bool getReadings();
bool checkThresholds();
int compressReadingINT(float);
bool sendReadings();
void getNvm(int, int);
bool writeNvm(int, char *, int);
bool read_EEPROM_parameterKeyMatch();
bool read_EEPROM_allParameters();
bool write_EEPROM_allDefaultParameters();
bool write_EEPROM_LPcheckCounter();
bool write_EEPROM_rapidSendMode();
bool sendError();


// BME280 support variables
const int addr = 0xEE;
float atmPressure_hPa = 0;
float atmPressure_psi = 0;
float temperature_degC = 0;
float humidity_RH = 0;
float temperature_DEFAULT_thresholdHigh = 60;
float temperature_EEPROM_thresholdHigh = 0;
static uint16_t askAgainLimit_BME280 = 3;

// 89BSD variables
char data[3];
char dataNew[2];
float linePressure_atm = 0;
float linePressure_psi = 0;
float linePressure_DEFAULT_thresholdHigh = 80;
float linePressure_DEFAULT_thresholdLow = 1;
float linePressure_EEPROM_thresholdHigh = 0;
float linePressure_EEPROM_thresholdLow = 0;
static uint16_t askAgainLimit_89BSD = 3;

// Program variables (LP = Line Pressure)
// DEFAULT = saved in program flash. Changable only by recompiling code.
// EEPROM = saved in EEPROM xDot memory.  
char writeBuffer[8];
char readBuffer[8];
char config_DEFAULT_parametersKey[] = {0x24, 0xD0, 0x57, 0xDA};
int config_DEFAULT_LPcheckInterval = 60;          //seconds //1800
int config_DEFAULT_heartbeatInterval = 10;          // sample to send ratio (ie. 24 * 1800 = 43200secs => 12hrs, heartbeat every 12hrs)
int config_DEFAULT_LPcheckCounter = 0;
int config_DEFAULT_rapidSendMode = 1;              // when powered on first time, execute rapid send mode
int config_DEFAULT_rapidSendMode_LPcheckInterval = 60;  // seconds (ie. every 60 secs check line pressure)
int config_DEFAULT_rapidSendMode_LPcheckTarget = 5;     // target number of tranmissions needed to complete rapid send mode
bool newDownlinkDataAvaiable = false;
bool gotBME280 = false; 
bool got89BSD = false;
int getReadingsError = 0;

// EEPROM deepsleep stuff
int config_EEPROM_LPcheckInterval = 0;             //seconds
int config_EEPROM_heartbeatInterval = 0;           //Line Pressure checks
int config_EEPROM_LPcheckCounter = 0;              // tracks number of LPchecks inbetween heartbeats
int config_EEPROM_rapidSendMode = 0;               // holds "rapid send mode" setting across deepsleeps

//LowPowerTimer t;
Timer t;

int main()
{

    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    pc.baud(115200);
    i2c.frequency(400000);

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);
    plan = new lora::ChannelPlan_US915();
    assert(plan);
    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    // do not do the following if waking from deepsleep
    if (!dot->getStandbyFlag())
    {
        redLED = 1;
        wait(0.05);
        redLED = 0;
        wait(0.1);
        redLED = 1;
        wait(0.05);
        redLED = 0;

        logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);
        //dot->setLogLevel(mts::MTSLog::TRACE_LEVEL);

        // update configuration if necessary
        // in AUTO_OTA mode the session is automatically saved, so saveNetworkSession and restoreNetworkSession are not needed
        if (dot->getJoinMode() != mDot::AUTO_OTA)
        {
            logInfo("changing network join mode to AUTO_OTA");
            if (dot->setJoinMode(mDot::AUTO_OTA) != mDot::MDOT_OK)
            {
                logError("failed to set network join mode to AUTO_OTA");
            }
        }
        // in OTA and AUTO_OTA join modes, the credentials can be passed to the library as a name and passphrase or an ID and KEY
        // only one method or the other should be used!
        // network ID = crc64(network name)
        // network KEY = cmac(network passphrase)
        update_ota_config_id_key(network_id, network_key, frequency_sub_band, public_network, ack);

        // configure network link checks
        // network link checks are a good alternative to requiring the gateway to ACK every packet and should allow a single gateway to handle more Dots
        // check the link every count packets
        // declare the Dot disconnected after threshold failed link checks
        // for count = 3 and threshold = 5, the Dot will ask for a link check response every 5 packets and will consider the connection lost if it fails to receive 3 responses in a row
        update_network_link_check_config(3, 5);

        // enable or disable Adaptive Data Rate
        dot->setAdr(adr);

        // Configure the join delay
        dot->setJoinDelay(join_delay);

        // save changes to configuration
        logInfo("saving configuration");
        if (!dot->saveConfig())
        {
            logError("failed to save configuration");
        }

        // display configuration
        display_config();

        wait(5);
    }

    // Do the following independent of wake cause
    while(true){
        mainControl(&events);
    }
}

void mainControl(RadioEvent *events)
{
    t.reset();
    t.start();
    printf("\r\n");
    if (getReadings()){
        if (config_DEFAULT_rapidSendMode == 1){
            // rapid send mode
            logInfo("[MAIN] Operating in rapid send mode");
            if (sendReadings()){
                flashLED();
                config_DEFAULT_LPcheckCounter += 1;
                logInfo("[MAIN] LPcheckCount: %i, LPcheckTarget: %i", config_DEFAULT_LPcheckCounter, config_DEFAULT_rapidSendMode_LPcheckTarget);
                newDownlinkDataAvaiable = events->newDownlink;
                if (!newDownlinkDataAvaiable){ 
                    // no new config data within downlink, continue as usual
                    if (config_DEFAULT_LPcheckCounter < config_DEFAULT_rapidSendMode_LPcheckTarget){
                        // still running in rapid send mode
                        sleepxDot(config_DEFAULT_rapidSendMode_LPcheckInterval);
                    }
                    else{
                        // finished with "rapid send mode"
                        // clear line pressure check counter and "rapid send mode" values
                        config_DEFAULT_LPcheckCounter = 0;
                        config_DEFAULT_rapidSendMode = 0;
                        logInfo("[MAIN] completed rapid send mode");
                        logInfo("[MAIN] starting normal operation");
                        // sleep using normal line pressure check interval
                        sleepxDot(config_DEFAULT_LPcheckInterval);
                    }
                }              
            }
        }
        else{
            // normal operating mode
            logInfo("[MAIN] Operating in normal mode");
            if (config_DEFAULT_LPcheckCounter >= config_DEFAULT_heartbeatInterval-1){
                // send heartbeat 
                if (sendReadings()){
                    flashLED();
                    config_DEFAULT_LPcheckCounter = 0;
                    // check for any downlink data //
                    newDownlinkDataAvaiable = events->newDownlink;
                    if (!newDownlinkDataAvaiable){
                        // no new downlink data, continue as usual
                        sleepxDot(config_DEFAULT_LPcheckInterval);        
                    }
                }
            }
            else{
                // take reading and only send if breaks alarm threshold
                config_DEFAULT_LPcheckCounter += 1;
                logInfo("[MAIN] LPcheckCount: %i, LPcheckTarget: %i", config_DEFAULT_LPcheckCounter, config_DEFAULT_heartbeatInterval);
                if (checkThresholds()){
                    // one or more thresholds crossed, send data out immediatly //
                    if (sendReadings()){
                        flashLED();
                        newDownlinkDataAvaiable = events->newDownlink;
                        if (!newDownlinkDataAvaiable){
                            // no new downlink data, continue as usual
                            sleepxDot(config_DEFAULT_LPcheckInterval);
                        }
                    }
                }else{
                    sleepxDot(config_DEFAULT_LPcheckInterval);
                }
            }
        }
    }
    else{
        // unable to get readings
        logInfo("[MAIN] unable to get readings");
        getReadingsError += 1;
        if (getReadingsError >= 3){
            if (sendError()){
                logInfo("Error info sent");
                getReadingsError = 0;
                newDownlinkDataAvaiable = events->newDownlink;
                if (!newDownlinkDataAvaiable){
                    sleepxDot(config_DEFAULT_LPcheckInterval);
                }
            }
        }
        else{
            wait(1);
        }
    }

    if (newDownlinkDataAvaiable){
        newDownlinkDataAvaiable = false;
        events->newDownlink = false;
        logInfo("[MAIN] updating program config with new downlink data");
        //logInfo("[MAIN] downlink length: %i", strlen(events->downlinkMsg));

        // reset existing counters/variables
        config_DEFAULT_rapidSendMode = 0;
        config_DEFAULT_LPcheckCounter = 0;

        // apply new config
        config_DEFAULT_LPcheckInterval = events->downlinkMsg[0];
        config_DEFAULT_LPcheckInterval = (config_DEFAULT_LPcheckInterval << 8) | events->downlinkMsg[1];
        //config_DEFAULT_LPcheckInterval = (config_DEFAULT_LPcheckInterval << 8) | events->downlinkMsg[2];
        logInfo("[PRGM] config_DEFAULT_LPcheckInterval: %i", config_DEFAULT_LPcheckInterval);
        
        config_DEFAULT_heartbeatInterval = events->downlinkMsg[2];
        config_DEFAULT_heartbeatInterval = (config_DEFAULT_heartbeatInterval << 8) | events->downlinkMsg[3];
        //config_DEFAULT_heartbeatInterval = (config_DEFAULT_heartbeatInterval << 8) | events->downlinkMsg[7];
        logInfo("[PRGM] config_DEFAULT_heartbeatInterval: %i", config_DEFAULT_heartbeatInterval);

        uint16_t temporaryBuffer = events->downlinkMsg[4];
        temporaryBuffer = (temporaryBuffer << 8) | events->downlinkMsg[5];
        linePressure_DEFAULT_thresholdHigh = ((float)temporaryBuffer) * 0.01;
        logInfo("[PRGM] linePressure_DEFAULT_thresholdHigh: %f", linePressure_DEFAULT_thresholdHigh);

        temporaryBuffer = events->downlinkMsg[6];
        temporaryBuffer = (temporaryBuffer << 8) | events->downlinkMsg[7];
        linePressure_DEFAULT_thresholdLow = ((float)temporaryBuffer) * 0.01;
        logInfo("[PRGM] linePressure_DEFAULT_thresholdLow: %f", linePressure_DEFAULT_thresholdLow);

        temporaryBuffer = events->downlinkMsg[8];
        temporaryBuffer = (temporaryBuffer << 8) | events->downlinkMsg[9];
        temperature_DEFAULT_thresholdHigh = ((float)temporaryBuffer) * 0.01;
        logInfo("[PRGM] temperature_DEFAULT_thresholdHigh: %f", temperature_DEFAULT_thresholdHigh);

        logInfo("[MAIN] updated program config with new downlink data");
        logInfo("[MAIN] restarting program");
        
    }
    
}

bool sendError(){

    uint32_t ret;
    std::vector<uint8_t> tx_data;

    if (got89BSD){
        // only have line pressure reading, send code 01 and line pressure reading
        tx_data.push_back(01);
        int linePressure_2bytes = compressReadingINT(linePressure_psi);
        tx_data.push_back((linePressure_2bytes >> 8) & 0xFF);
        tx_data.push_back(linePressure_2bytes & 0xFF);
    }
    else if (gotBME280){
        // only have BME280 readings, send code 02 plus BME280 readings
        tx_data.push_back(02);
        int atmPressure_2bytes = compressReadingINT(atmPressure_psi);
        int temperature_2btyes = compressReadingINT(temperature_degC);
        int humidity_2bytes = compressReadingINT(humidity_RH);
        tx_data.push_back((atmPressure_2bytes >> 8) & 0xFF);
        tx_data.push_back(atmPressure_2bytes & 0xFF);
        tx_data.push_back((temperature_2btyes >> 8) & 0xFF);
        tx_data.push_back(temperature_2btyes & 0xFF);
        tx_data.push_back((humidity_2bytes >> 8) & 0xFF);
        tx_data.push_back(humidity_2bytes & 0xFF);
    }
    else{
        // neither readings are working, send code 03 along with heartbeat interval 
        tx_data.push_back(03);
        tx_data.push_back((config_DEFAULT_heartbeatInterval >> 8) & 0xFF);
        tx_data.push_back(config_DEFAULT_heartbeatInterval & 0xFF);
    }

    // Attach interval info on end in any case
    //tx_data.push_back((config_DEFAULT_LPcheckInterval >> 16) & 0xFF);
    tx_data.push_back((config_DEFAULT_LPcheckInterval >> 8) & 0xFF);
    tx_data.push_back(config_DEFAULT_LPcheckInterval & 0xFF);
    
    logInfo("SEND PAYLOAD: %s", tx_data);
    logInfo("VECTOR LENGTH: %d", tx_data.size());

    //JOIN NETWORK IF NOT ALREADY JOINED
    if (!dot->getNetworkJoinStatus())
    {
        //IF NETWORK LOST, TURN OFF POWER HUNGRY THINGS.  THIS WAY POWER WILL BE CONSERVED WHEN JOIN NETWORK FUNCTION TAKES OVER.
        logInfo("[LORAWAN] network not joined, joining..");
        redLED = 0;
        join_network();
    }
    
    ret = dot->send(tx_data);
    if (ret != mDot::MDOT_OK)
    {
        logInfo("[LORAWAN] Send failed");
        logInfo("[LORAWAN] %s", mDot::getReturnCodeString(ret).c_str());
        return false;
    }
    else
    {
        logInfo("[LORAWAN] Send success");
        return true;
    }

}



bool checkThresholds()
{
    if (linePressure_psi < linePressure_DEFAULT_thresholdHigh){
        if (linePressure_psi > linePressure_DEFAULT_thresholdLow){
            if (temperature_degC < temperature_DEFAULT_thresholdHigh){
                return false;
            }
        }
    }
    return true;
}


bool write_EEPROM_LPcheckCounter(){
    char parms[3];
    parms[0] = (config_EEPROM_LPcheckCounter >> 16) & 0xFF;
    parms[1] = (config_EEPROM_LPcheckCounter >> 8) & 0xFF;
    parms[2] = config_EEPROM_LPcheckCounter & 0xFF; 
    if (writeNvm(0x000B, parms, 3)){
        logInfo("[EEPROM] LPcheckCounter write success");
    }
    else{
        logInfo("[EEPROM] failed to write LPCheckCounter");
    }
}

bool write_EEPROM_rapidSendMode(){
    char parms[1];
    parms[0] = config_EEPROM_rapidSendMode;
    if (writeNvm(0x0004, parms, 1)){
        logInfo("[EEPROM] rapidSendMode write success");
    }
    else{
        logInfo("[EEPROM] failed to write rapidSendMode");
    }
}


bool read_EEPROM_allParameters()
{
    getNvm(0x0000, 20);

    logInfo("[PRGM] key: %X, %X, %X, %X", readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);

    config_EEPROM_rapidSendMode = readBuffer[4];
    logInfo("[PRGM] config_EEPROM_rapidSendMode: %i", readBuffer[4]);

    config_EEPROM_LPcheckInterval = readBuffer[5];
    config_EEPROM_LPcheckInterval = (config_EEPROM_LPcheckInterval << 8) | readBuffer[6];
    config_EEPROM_LPcheckInterval = (config_EEPROM_LPcheckInterval << 8) | readBuffer[7];
    logInfo("[PRGM] config_EEPROM_LPcheckInterval: %i", config_EEPROM_LPcheckInterval);

    config_EEPROM_heartbeatInterval = readBuffer[8];
    config_EEPROM_heartbeatInterval = (config_EEPROM_heartbeatInterval << 8) | readBuffer[9];
    config_EEPROM_heartbeatInterval = (config_EEPROM_heartbeatInterval << 8) | readBuffer[10];
    logInfo("[PRGM] config_EEPROM_heartbeatInterval: %i", config_EEPROM_heartbeatInterval);

    config_EEPROM_LPcheckCounter = readBuffer[11];
    config_EEPROM_LPcheckCounter = (config_EEPROM_LPcheckCounter << 8) | readBuffer[12];
    config_EEPROM_LPcheckCounter = (config_EEPROM_LPcheckCounter << 8) | readBuffer[13];
    logInfo("[PRGM] config_EEPROM_LPcheckCounter: %i", config_EEPROM_LPcheckCounter);

    uint16_t temporaryBuffer = readBuffer[14];
    temporaryBuffer = (temporaryBuffer << 8) | readBuffer[15];
    linePressure_EEPROM_thresholdHigh = ((float)temporaryBuffer) * 0.01;
    logInfo("[PRGM] linePressure_EEPROM_thresholdHigh: %f", linePressure_EEPROM_thresholdHigh);

    temporaryBuffer = readBuffer[16];
    temporaryBuffer = (temporaryBuffer << 8) | readBuffer[17];
    linePressure_EEPROM_thresholdLow = ((float)temporaryBuffer) * 0.01;
    logInfo("[PRGM] linePressure_EEPROM_thresholdLow: %f", linePressure_EEPROM_thresholdLow);

    temporaryBuffer = readBuffer[18];
    temporaryBuffer = (temporaryBuffer << 8) | readBuffer[19];
    temperature_EEPROM_thresholdHigh = ((float)temporaryBuffer) * 0.01;
    logInfo("[PRGM] temperature_EEPROM_thresholdHigh: %f", temperature_EEPROM_thresholdHigh);
}

bool write_EEPROM_allDefaultParameters()
{
    char parms[20];

    // write EEPROM parameter key
    parms[0] = config_DEFAULT_parametersKey[0];
    parms[1] = config_DEFAULT_parametersKey[1];
    parms[2] = config_DEFAULT_parametersKey[2];
    parms[3] = config_DEFAULT_parametersKey[3];

    // set rapidSendMode to true
    parms[4] = config_DEFAULT_rapidSendMode;

    // write the default line pressure check interval in seconds
    parms[5] = (config_DEFAULT_LPcheckInterval >> 16) & 0xFF;
    parms[6] = (config_DEFAULT_LPcheckInterval >> 8) & 0xFF;
    parms[7] = config_DEFAULT_LPcheckInterval & 0xFF;

    // write the default heartbeat interval in number of consecutive line pressure checks
    parms[8] = (config_DEFAULT_heartbeatInterval >> 16) & 0xFF;
    parms[9] = (config_DEFAULT_heartbeatInterval >> 8) & 0xFF;
    parms[10] = config_DEFAULT_heartbeatInterval & 0xFF;

    // initilize the line pressure check counter to 0
    parms[11] = (config_DEFAULT_LPcheckCounter >> 16) & 0xFF;
    parms[12] = (config_DEFAULT_LPcheckCounter >> 8) & 0xFF;
    parms[13] = config_DEFAULT_LPcheckCounter & 0xFF;

    // write default line pressure alarm threshold
    int temporaryBuffer = compressReadingINT(linePressure_DEFAULT_thresholdHigh);
    parms[14] = (temporaryBuffer >> 8) & 0xFF;
    parms[15] = temporaryBuffer & 0xFF;

    // write default line pressure alarm threshold
    temporaryBuffer = compressReadingINT(linePressure_DEFAULT_thresholdLow);
    parms[16] = (temporaryBuffer >> 8) & 0xFF;
    parms[17] = temporaryBuffer & 0xFF;

    // write default temperature alarm threshold
    temporaryBuffer = compressReadingINT(temperature_DEFAULT_thresholdHigh);
    parms[18] = (temporaryBuffer >> 8) & 0xFF;
    parms[19] = temporaryBuffer & 0xFF;

    printf("[parms to write] ");
    for (int i = 0; i < 20; i++)
    {
        printf("%X ", parms[i]);
        //logInfo("[parms_to_write] %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X ", parms[0], parms[1], parms[2], parms[3], parms[4], parms[5], parms[6], parms[1], parms[0], parms[1], parms[0], parms[1], parms[0], parms[1], parms[0], parms[1], parms[0], parms[1], parms[0], parms[1],);
    }
    printf("\r\n");

    if (writeNvm(0x0000, parms, 20)){
        logInfo("EEPROM all default parameters write success");
        return true;
    }
    return false;
}

bool read_EEPROM_parameterKeyMatch()
{
    getNvm(0x0000, 4);
    bool parameterKeyMatch = true;
    for (int i = 0; i < 4; i++)
    {
        if (readBuffer[i] != config_DEFAULT_parametersKey[i])
        {
            parameterKeyMatch = false;
        }
    }
    return parameterKeyMatch;
}

int compressReadingINT(float value)
{
    int output = (int)(value / 0.01);
    logInfo("[compress] before: %f, after: %i", value, output);
    return output;
}

void getNvm(int address, int length)
{
    logInfo("[EEPROM] read %i byte(s) starting at address %i(%X)", length, address, address);
    dot->nvmRead(address, readBuffer, length);
    printf("[INFO] [EEPROM read] ");
    for (int i = 0; i < length; i++)
    {
        printf("%X ", readBuffer[i]);
    }
    printf("\r\n");
}


bool writeNvm(int address, char *writeBuffer, int length)
{
    //logInfo("[EEPROM] write %i byte(s) starting at address %i(%X)", length, address, address);
    if (dot->nvmWrite(address, writeBuffer, length)){
        return true;
    }
    return false;
}

bool getReadings()
{
    gotBME280 = false;
    got89BSD = false;

    if (gotBME280 != true)
    {
        if (getBME280())
        {
            gotBME280 = true;
        }
    }
    if (got89BSD != true)
    {
        if (get89BSD())
        {
            got89BSD = true;
        }
    }
    if ((gotBME280 != true) || (got89BSD != true)){
        return false;
    }
    return true;
}

void flashLED()
{
    redLED = 1;
    wait(0.05);
    redLED = 0;
}

// in deepsleep mode, IOs float, RAM is lost, and application will start from beginning after waking up
void deepSleepxDot(int delay_s)
{
    logInfo("[deepsleep] %i secs", delay_s);
    redLED = 0;
    while (true){
        dot->sleep(delay_s, mDot::RTC_ALARM, true);
        logInfo("[deepsleep] waiting until xDot is idle");
        wait(1);
    }
}

// in sleep mode, IO state is maintained, RAM is retained, and application will resume after waking up
void sleepxDot(int delay_s)
{
    t.stop();
    redLED = 0;
    int newDelay_s = delay_s - t.read();
    logInfo("[sleep] target: %i secs, adjusted: %i secs", delay_s, newDelay_s);
    sleep_save_io();
    sleep_configure_io();
    dot->sleep(newDelay_s, mDot::RTC_ALARM, false);
    logInfo("[sleep] restoring IO");
    sleep_restore_io();
}

bool sendReadings()
{
    uint32_t ret;
    std::vector<uint8_t> tx_data;

    int linePressure_2bytes = compressReadingINT(linePressure_psi);
    int atmPressure_2bytes = compressReadingINT(atmPressure_psi);
    int temperature_2btyes = compressReadingINT(temperature_degC);
    int humidity_2bytes = compressReadingINT(humidity_RH);

    tx_data.push_back(00);
    tx_data.push_back((linePressure_2bytes >> 8) & 0xFF);
    tx_data.push_back(linePressure_2bytes & 0xFF);
    tx_data.push_back((atmPressure_2bytes >> 8) & 0xFF);
    tx_data.push_back(atmPressure_2bytes & 0xFF);
    tx_data.push_back((temperature_2btyes >> 8) & 0xFF);
    tx_data.push_back(temperature_2btyes & 0xFF);
    tx_data.push_back((humidity_2bytes >> 8) & 0xFF);
    tx_data.push_back(humidity_2bytes & 0xFF);
    //tx_data.push_back((config_DEFAULT_LPcheckInterval >> 16) & 0xFF);
    tx_data.push_back((config_DEFAULT_LPcheckInterval >> 8) & 0xFF);
    tx_data.push_back(config_DEFAULT_LPcheckInterval & 0xFF);

    logInfo("SEND PAYLOAD: %s", tx_data);
    logInfo("VECTOR LENGTH: %d", tx_data.size());

    //JOIN NETWORK IF NOT ALREADY JOINED
    if (!dot->getNetworkJoinStatus())
    {
        //IF NETWORK LOST, TURN OFF POWER HUNGRY THINGS.  THIS WAY POWER WILL BE CONSERVED WHEN JOIN NETWORK FUNCTION TAKES OVER.
        logInfo("[LORAWAN] network not joined, joining..");
        redLED = 0;
        join_network();
    }
    
    ret = dot->send(tx_data);
    if (ret != mDot::MDOT_OK)
    {
        logInfo("[LORAWAN] Send failed");
        logInfo("[LORAWAN] %s", mDot::getReturnCodeString(ret).c_str());
        return false;
    }
    else
    {
        logInfo("[LORAWAN] Send success");
        return true;
    }
}

bool getBME280()
{
    int askAgain = askAgainLimit_BME280;
    int timeoutCounter = 10;

    while ((askAgain > 0) && (timeoutCounter > 0))
    {
        bool badReading = false;
        //Calculating Pressure,Humidity, and temp from BME280
        // set mode to forced to trigger one single reading from each sensor
        sensor.setForcedMode();
        // gather readings from the forced mode trigger
        temperature_degC = sensor.getTemperature();
        humidity_RH = sensor.getHumidity();
        atmPressure_hPa = sensor.getPressure();
        atmPressure_psi = atmPressure_hPa * 0.0145037738;
        logInfo("[SAMPLE_BME] %f(hPa), %f(psi), %f(C), %f(RH)", atmPressure_hPa, atmPressure_psi, temperature_degC, humidity_RH);

        // are readings within bounds?
        if ((atmPressure_hPa < -1.0) || (atmPressure_hPa > 5000.0))
        {
            badReading = true;
            logInfo("[atmPressure_hPa] out of range");
        }
        if ((temperature_degC < -50.0) || (temperature_degC > 200.0))
        {
            badReading = true;
            logInfo("[temperature_degC] out of range");
        }
        if ((humidity_RH < -10.0) || (humidity_RH > 110.0))
        {
            badReading = true;
            logInfo("[humidity_RH] out of range");
        }
        if (badReading != true)
        {
            timeoutCounter = 10;
            askAgain -= 1;
        }
        else
        {
            timeoutCounter -= 1;
        }
        wait(0.05);
    }
    if ((askAgain == 0) && (timeoutCounter > 0))
    {
        return true;
    }
    else if (timeoutCounter == 0)
    {
        return false;
    }
}

bool get89BSD()
{
    int askAgain = askAgainLimit_89BSD;
    int timeoutCounter = 5;

    while ((askAgain > 0) && (timeoutCounter > 0))
    {
        bool badReading = false;

        performGet();

        logInfo("[SAMPLE_BSD] %f(atm), %f(psi)", linePressure_atm, linePressure_psi);

        if ((linePressure_atm < -2.0) || (linePressure_atm > 5.0))
        {
            badReading = true;
            logInfo("[linePressure_atm] out of range");
        }

        if (badReading != true)
        {
            timeoutCounter = 10;
            askAgain -= 1;
        }
        else
        {
            timeoutCounter -= 1;
        }
        wait(0.05);
    }
    if ((askAgain == 0) && (timeoutCounter > 0))
    {
        return true;
    }
    else if (timeoutCounter == 0)
    {
        return false;
    }
}

bool performGet()
{
    int16_t coefficientValue1, coefficientValue2, coefficientC0, coefficientC1, coefficientC2, coefficientC3, coefficientC4, coefficientC5, coefficientC6;
    char cmdD1[1], cmdD2[1], cmdC0[1], cmdC1[1], cmdC2[1], cmdC4[1], cmdC5[1];
    char readADC1[1], readADC2[1];
    uint32_t hex1, hex2, hex3;
    uint32_t temphex1, temphex2, temphex3;
    int16_t c01, c02, c11, c12, c21, c22, c23, c41, c42, c51, c52, c53, c54;
    uint32_t D1, D2;
    double Y, X, Z, P;

    //Set the correct I2C addresses to get values for D1 conversions (pressure)
    cmdD1[0] = 0x40;
    readADC1[0] = 0x00;
    sensorReading1(cmdD1, readADC1);

    //Assigning values to variables with data from the array
    hex1 = data[0];
    hex2 = data[1];
    hex3 = data[2];

    // Bit Shifting
    hex1 = hex1 << 16;
    hex2 = hex2 << 8;

    // ORing all hex bit shifted variables to get value D1
    D1 = hex1 | hex2 | hex3;

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Set the correct I2C addresses to get values for D2 conversions (bme_temperature_sample)
    cmdD2[0] = 0x50;
    readADC2[0] = 0x00;
    sensorReading1(cmdD2, readADC2);

    //Assigning values to variables with data from the array
    temphex1 = data[0];
    temphex2 = data[1];
    temphex3 = data[2];

    // Bit Shifting
    temphex1 = temphex1 << 16;
    temphex2 = temphex2 << 8;

    // ORing all hex bit shifted variables to get value D2
    D2 = temphex1 | temphex2 | temphex3;

    /*--------------------------------------------------------------------------------------------------------------------*/
    // Setting the address for the C0 coefficent
    cmdC0[0] = 0xA2;
    sensorReading2(cmdC0);

    // Storing values from array into variables
    c01 = dataNew[0];
    c02 = dataNew[1];

    //Bit Shifting
    c01 = c01 << 6;
    c02 = c02 >> 2;

    //ORing all hex bit shifted variables to get coefficent C0
    coefficientValue1 = c01 | c02;
    coefficientC0 = numberCheck1(coefficientValue1);

    /*--------------------------------------------------------------------------------------------------------------------*/
    // Setting the address for the C1 coefficent
    cmdC1[0] = 0xA4;
    sensorReading2(cmdC1);

    // Storing values from array into variables
    c11 = dataNew[0];
    c12 = dataNew[1];

    //Bit Shifting
    c11 = c11 << 4;
    c12 = c12 >> 4;

    //ORing all hex bit shifted variables to get coefficent C1
    coefficientValue1 = c11 | c12;
    coefficientC1 = numberCheck1(coefficientValue1);

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Setting the address for both C2 and C3 coefficents since all the bits needed are in one address
    cmdC2[0] = 0xA6;
    sensorReading2(cmdC2);

    // Storing values from array into variables
    c21 = dataNew[0];
    c22 = dataNew[1];
    c23 = dataNew[0];

    //Bit Shifting
    c21 = c21 >> 2;
    c23 = c23 << 14;
    c23 = c23 >> 6;

    //ORing hex bit shifted variables to get coefficent C2
    coefficientValue2 = c21;
    coefficientC2 = numberCheck2(coefficientValue2);

    //ORing hex bit shifted variables to get coefficent C3
    coefficientValue2 = c23 | c22;
    coefficientC3 = numberCheck2(coefficientValue2);

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Setting the address for the C4 coefficent
    cmdC4[0] = 0xA8;
    sensorReading2(cmdC4);

    //Storing values from array into variables
    c41 = dataNew[0];
    c42 = dataNew[1];
    c53 = dataNew[1];

    //Bit Shifting
    c41 = c41 << 2;
    c42 = c42 >> 6;

    //ORing all hex bit shifted variable to get coefficent C4
    coefficientValue2 = c41 | c42;
    coefficientC4 = numberCheck2(coefficientValue2);

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Setting the address for the C5 and C6 coefficents
    cmdC5[0] = 0xAA;
    sensorReading2(cmdC5);

    //Storing values from array into variables
    c51 = dataNew[0];
    c52 = dataNew[1];
    c54 = dataNew[0];

    //Bit Shifting
    c53 = c53 << 10;
    c53 = c53 >> 6;
    c51 = c51 >> 4;
    c54 = c54 << 12;
    c54 = c54 >> 6;
    c52 = c52 >> 2;

    //ORing hex bit shifted variable to get coefficent C5
    coefficientValue2 = c53 | c51;
    coefficientC5 = numberCheck2(coefficientValue2);

    //ORing hex bit shifted variable to get coefficent C6
    coefficientValue2 = c54 | c52;
    coefficientC6 = numberCheck2(coefficientValue2);

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Calculating Y
    X = D1 + (coefficientC0 * pow(2.0, 9.0)) + (coefficientC3 * pow(2.0, 15.0)) * (D2 / pow(2.0, 24.0)) + coefficientC4 * pow(2.0, 15.0) * ((D2 / pow(2.0, 24.0)) * D2 / pow(2.0, 24.0));

    Z = (coefficientC1 * pow(2.0, 11.0)) + (coefficientC5 * pow(2.0, 16.0)) * (D2 / pow(2.0, 24.0)) + (coefficientC6 * pow(2.0, 16.0)) * ((D2 / pow(2.0, 24.0)) * D2 / pow(2.0, 24.0));

    Y = X / Z;

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Calculating P using Y
    P = Y * (1 - coefficientC2 * (pow(2.0, 9.0) / pow(2.0, 24.0))) + coefficientC2 * (pow(2.0, 9.0) / pow(2.0, 24.0)) * Y * Y;

    /*--------------------------------------------------------------------------------------------------------------------*/
    //Calculating Pressure using P
    linePressure_atm = (P - 0.1) / 0.8 * (6);
    linePressure_psi = linePressure_atm * 14.503773773;

    return true;
}

void sensorReading1(char cmd[], char readADC[])
{
    //Writing and reading values from I2C line
    i2c.write(addr, cmd, 1);
    wait(0.07);
    i2c.write(addr, readADC, 1);
    wait(0.07);
    i2c.read(addr, data, 3);
}

void sensorReading2(char cmd[])
{
    //Writing to the I2C line
    i2c.write(addr, cmd, 1);
    wait(0.07);

    //Reading from the I2C line
    i2c.read(addr, dataNew, 2);
    wait(0.07);
}

int numberCheck1(int16_t coefficientCheck1)
{
    uint16_t checkVar14 = 49152;
    if (coefficientCheck1 > 8192)
    {
        //coefficientValue1 = coefficientValue1 | checkVar14;
        return coefficientCheck1 | checkVar14;
        //wait(0.5);
    }
    else
    {
        return coefficientCheck1;
        //wait(0.5);
    }
}

int numberCheck2(int16_t coefficientCheck2)
{
    uint16_t checkVar10 = 64512;
    if (coefficientCheck2 > 512)
    {
        //coefficientValue2 = coefficientValue2 | checkVar10;
        return coefficientCheck2 | checkVar10;
        //wait(0.5);
    }
    else
    {
        return coefficientCheck2;
        //wait(0.5);
    }
}

#endif
