#include "dot_util.h"
#include "RadioEvent.h"
#include "BME280.h"

#if ACTIVE_EXAMPLE == AUTO_OTA_EXAMPLE

// LORAWAN CONFIGURATION
static uint8_t network_id[] = {0x00, 0x25, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x01};
static uint8_t network_key[] = {0x15, 0x35, 0x78, 0x15, 0x13, 0xEE, 0x71, 0x92, 0xD2, 0x02, 0xA9, 0x4B, 0x72, 0xE4, 0x52, 0x91};
static uint8_t frequency_sub_band = 1;
static bool public_network = true;
static uint8_t join_delay = 5;
static uint8_t ack = 0;
static bool adr = true;

//Function Prototypes
void sensorReading1(char cmd[], char readADC[]);
void sensorReading2(char cmd[]);
void numberCheck1(int16_t coefficientCheck1);
void numberCheck2(int16_t coefficientCheck2);
bool sendReadings();
bool encodeReading();
bool get89BSD();
bool getBME280();
void realMain();
void sleepxDot();
void deepSleepxDot();


// PROGRAM GLOBAL VARIABLES
float temp = 0;
float humidity = 0;
float pressureBME = 0;
float pressure89BSD = 0;
float pressure89BSD_psi = 0;
float pressureBME_psi = 0;
char data[3];
char dataNew[2];
int16_t coefficientValue1, coefficientValue2, coefficientC0, coefficientC1, coefficientC2, coefficientC3, coefficientC4, coefficientC5, coefficientC6;
uint16_t checkVar14 = 49152;
uint16_t checkVar10 = 64512;

char cmdD1[1], cmdD2[1], cmdC0[1], cmdC1[1], cmdC2[1], cmdC4[1], cmdC5[1];
char readADC1[1], readADC2[1];
uint32_t hex1, hex2, hex3;
uint32_t temphex1, temphex2, temphex3;
int16_t c01, c02, c11, c12, c21, c22, c23, c41, c42, c51, c52, c53, c54;
uint32_t D1, D2;
double Y, X, Z, P;

static uint32_t delay_s;




//DigitalOut power_89BSD(PB_0);
//DigitalOut power_BME280(PB_2);
int on = 1;
int off = 0;

// deepsleep consumes slightly less current than sleep
// in sleep mode, IO state is maintained, RAM is retained, and application will resume after waking up
// in deepsleep mode, IOs float, RAM is lost, and application will start from beginning after waking up

mDot *dot = NULL;
lora::ChannelPlan *plan = NULL;

//Debug out to pc
Serial pc(USBTX, USBRX);
//89BSD I2C setup
I2C i2c(I2C_SDA, I2C_SCL);
const int addr = 0xEE;
//BME280 I2C setup
BME280 sensor(I2C_SDA, I2C_SCL);

int main()
{
    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    // modes PullUp/PullDown/PullNone/OpenDrain
    //power_89BSD.mode(OpenDrain);
    //power_89BSD.output();
    //power_BME280.mode(OpenDrain);
    //power_BME280.output();

    pc.baud(115200);

#if defined(TARGET_XDOT_L151CC)
    i2c.frequency(400000);
#endif

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);

    plan = new lora::ChannelPlan_US915();
    assert(plan);
    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    if (!dot->getStandbyFlag())
    {
        logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

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
    }

    while (true)
    {
        // start main program here
        realMain();
    }
    return 0;
}

void realMain()
{

    // MAIN

    // power on sensors
    //power_89BSD = on;
    //logInfo("power_89BSD = on, %d", on);
    //power_BME280 = on;
    //logInfo("power_BME280 = on, %d", on);
    //wait(.25);

    //take 5 readings
    for (int i = 0; i < 8; i++)
    {
        get89BSD();
        getBME280();

        logInfo("89BSD_pressure = %f, 89BSD_pressure_psi = %f, BME_pressure = %f, BME_pressure_psi = %f, temperature = %f, humidity = %f", pressure89BSD, pressure89BSD_psi, pressureBME, pressureBME_psi, temp, humidity);

        wait(3);
    }

    sendReadings();
 
    // power off sensors
    //power_89BSD = off;
    //logInfo("power_89BSD = off, %d", off);
    //power_BME280 = off;
    //logInfo("power_BME280 = off, %d", off);
    delay_s = 60;
    deepSleepxDot();
    logInfo();
}


// in deepsleep mode, IOs float, RAM is lost, and application will start from beginning after waking up
void deepSleepxDot()
{
    logInfo("deepsleep now");
    //logInfo("power_89BSD = %d, power_BME280 = %d", (uint8_t)power_89BSD, (uint8_t)power_BME280);
    dot->sleep(delay_s, mDot::RTC_ALARM, true);
    logInfo("shouldnt see this message");
}

// in sleep mode, IO state is maintained, RAM is retained, and application will resume after waking up
void sleepxDot()
{
    //logInfo("power_89BSD = %d, power_BME280 = %d", (uint8_t)power_89BSD, (uint8_t)power_BME280);
    logInfo("sleep %d secs, IO state maintained", delay_s);
    dot->sleep(delay_s, mDot::RTC_ALARM, false);
    logInfo("resume after sleep");
}


int encodeReading(float value)
{
    int16_t encodedValue_int;
    float fractionalComponent;
    float integralComponent;
    float encodedValue_float = (value / 0.01);
    fractionalComponent = modf(encodedValue_float, &integralComponent);
    if (fractionalComponent >= 0.5)
    {
        encodedValue_int = integralComponent + 1;
    }
    else if (fractionalComponent < 0.5)
    {
        encodedValue_int = integralComponent;
    }

    logInfo("value: %f, encodedValue(value/0.01): %f, integral: %f, fractional: %f, encodedValue_int: %d", value, encodedValue_float, integralComponent, fractionalComponent, encodedValue_int);

    return encodedValue_int;
}

bool sendReadings()
{
    uint32_t ret;
    std::vector<uint8_t> tx_data;

    int pressure89BSD_encoded = encodeReading(pressure89BSD_psi);
    int pressureBME_encoded = encodeReading(pressureBME_psi);
    int temp_encoded = encodeReading(temp);
    int humidity_encoded = encodeReading(humidity);

    tx_data.push_back(00);
    tx_data.push_back((pressure89BSD_encoded >> 8) & 0xFF);
    tx_data.push_back(pressure89BSD_encoded & 0xFF);
    tx_data.push_back((pressureBME_encoded >> 8) & 0xFF);
    tx_data.push_back(pressureBME_encoded & 0xFF);
    tx_data.push_back((temp_encoded >> 8) & 0xFF);
    tx_data.push_back(temp_encoded & 0xFF);
    tx_data.push_back((humidity_encoded >> 8) & 0xFF);
    tx_data.push_back(humidity_encoded & 0xFF);
    //tx_data.push_back((battery >> 8) & 0xFF);
    //tx_data.push_back(battery & 0xFF);
    //tx_data.push_back((delay_s >> 16) & 0xFF);
    //tx_data.push_back((delay_s >> 8) & 0xFF);
    //tx_data.push_back(delay_s & 0xFF);

    logInfo("SEND PAYLOAD: %s", tx_data);
    logInfo("VECTOR LENGTH: %d", tx_data.size());

    //JOIN NETWORK IF NOT ALREADY JOINED
    if (!dot->getNetworkJoinStatus())
    {
        //IF NETWORK LOST, TURN OFF POWER HUNGRY THINGS.  THIS WAY POWER WILL BE CONSERVED WHEN JOIN NETWORK FUNCTION TAKES OVER.
        //logInfo("dyn_power = %d", (uint8_t)dyn_power);
        //redLED = 0;
        join_network();
    }
    ret = dot->send(tx_data);
    if (ret != mDot::MDOT_OK)
    {
        logInfo("Send failed");
        return false;
    }
    else
    {
        logInfo("Send success");
        return true;
    }
}

bool getBME280()
{

    //Calculating Pressure,Humidity, and Temp from BME280

    // set mode to forced to trigger one single reading from each sensor
    sensor.setForcedMode();

    // gather readings from the forced mode trigger
    temp = sensor.getTemperature();
    humidity = sensor.getHumidity();
    pressureBME = sensor.getPressure();

    pressureBME_psi = pressureBME * 0.0145037738;

    return true;
}

bool get89BSD()
{

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
    //Set the correct I2C addresses to get values for D2 conversions (temp)
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
    numberCheck1(coefficientValue1);
    coefficientC0 = coefficientValue1;

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
    numberCheck1(coefficientValue1);
    coefficientC1 = coefficientValue1;

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
    numberCheck2(coefficientValue2);
    coefficientC2 = coefficientValue2;

    //ORing hex bit shifted variables to get coefficent C3
    coefficientValue2 = c23 | c22;
    numberCheck2(coefficientValue2);
    coefficientC3 = coefficientValue2;

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
    numberCheck2(coefficientValue2);
    coefficientC4 = coefficientValue2;

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
    numberCheck2(coefficientValue2);
    coefficientC5 = coefficientValue2;

    //ORing hex bit shifted variable to get coefficent C6
    coefficientValue2 = c54 | c52;
    numberCheck2(coefficientValue2);
    coefficientC6 = coefficientValue2;

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
    pressure89BSD = (P - 0.1) / 0.8 * (6);
    pressure89BSD_psi = pressure89BSD * 14.503773773;

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

void numberCheck1(int16_t coefficientCheck1)
{
    if (coefficientCheck1 > 8192)
    {
        coefficientValue1 = coefficientValue1 | checkVar14;
        wait(0.5);
    }
    else
    {
        wait(0.5);
    }
}

void numberCheck2(int16_t coefficientCheck2)
{
    if (coefficientCheck2 > 512)
    {
        coefficientValue2 = coefficientValue2 | checkVar10;
        wait(0.5);
    }
    else
    {
        wait(0.5);
    }
}

#endif 

