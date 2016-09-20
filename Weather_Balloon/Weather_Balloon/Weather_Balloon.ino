/*
  Serial.print((signed int)((DOF.temperature >> 8) + 25), DEC);

*/

#define DEBUG 1

#include <stdint.h>
#include <Wire.h>
//#include <SPI.h>
//#define USE_LCD

#ifdef USE_LCD
    #include "LCD.h"
    #include "LiquidCrystal_I2C.h"
#endif

#include "Adafruit_SI1145.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "MAG3110.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MS5xxx.h>
#include "Adafruit_SHT31.h"
#include "Sd2Card.h"
#include "Adafruit_TSL2561_U.h"
#include "SparkFunMAX17043.h"
#include "SparkFunLSM9DS1.h"
#include "TinyGPS++.h"


#ifdef USE_LCD
    // Pin numbers for LCD display *AT THE DISPLAY END* not for the Arduino.
    #define BACKLIGHT_PIN     3
    #define En_pin  2
    #define Rw_pin  1
    #define Rs_pin  0
    #define D4_pin  4
    #define D5_pin  5
    #define D6_pin  6
    #define D7_pin  7
#endif

// One Wire data is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// UV sensor is on analog A0
#define UV_PIN  A0

// CO2 analog pin
#define CO2_PIN A3

// Piezo pin
#define PIEZO_PIN   4

// Main compartment heater pin
#define MAIN_HEATER_PIN 9

#ifdef USE_LCD
    // LCD stuff
    #define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
    #define COLS      20
    #define ROWS       4
    #define PAUSE    400
#endif

/*  DS18B20 resolution / conversion times
 *  9 =  93.75 ms
 * 10 = 187.5  ms
 * 11 = 375    ms
 * 12 = 750    ms
*/
#define DS18B20_RESOLUTION 10

// Calculated correction factor for ADXL345
#define ADXL345_CORRECTION  0.911265

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//#define PRINT_CALCULATED
#define PRINT_RAW

typedef struct {
    // Temperature sensors
    float DS18B20_temp_c[1] = {0};

    // Humidity sensor
    float sht31_t, sht31_h = 0;

    // UV light output
    float uv = 0;

    // Battery voltage and state-of-charge
    float voltage, soc = 0;

    // Pressure and temp of pressure sensor
    double ms5607_pressure = 0, ms5607_temp = 0;

    // Infrared and broadband light readings
    struct {
        uint16_t ir, broadband = 0;
        uint32_t lux = 0;
    } tsl;

    struct {
        float x, y, z = 0;
    } accel;

    struct {
        float x, y, z = 0;
    } mag;

    struct {
        float x, y, z = 0;
    } gyro;

    // Geiger counts for X, Y, Z axis.
    struct {
        uint8_t x, y, z = 0;
    } geiger;

    struct {
        double lat, lng = 0;
        double altitude = 0;
        double speed, course = 0;

        uint32_t hdop = 0;

        uint32_t sats = 0;

        uint32_t date = 0;
        uint32_t time = 0;



    } gps;

} sensorData;

// Create global sensor structure
sensorData sensors;

// Light sensor
Adafruit_SI1145 uv = Adafruit_SI1145();

// Create accelerometer object with arbitrary ID
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DS18B20(&oneWire);

// Create magnetometer
//MAG3110 mag3110;

// Altimeter
MS5xxx ms5607(&Wire);

// Humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

MAX17043 fuel_gauge;

LSM9DS1 DOF;

/* Create GPS object
 * REQUIRES CUSTOM LARGER BUFFER FOR INCOMING UART
*/
TinyGPSPlus gps;

// GPS start and stop sequences to speak binary protocol to module
#define GPS_SBYTE_1  0xA0
#define GPS_SBYTE_2  0xA1
#define GPS_END_1    0x0D
#define GPS_END_2    0x0A

// GPS Message List
#define GPS_RESTART     0x01
#define GPS_CONFIG_NMEA 0x08
#define GPS_CONFIG_OUT  0x09
#define GPS_CONFIG_PWR  0x12

#define GPS_ACK         0x83
#define GPS_NACK        0x84

#ifdef USE_LCD
    // Create LCD
    LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
#endif

// Initialize sensors if we found them or not
bool SHT31_found = false;
bool ms5607_found = false;
bool uv_found = false;
bool mag3110_found = false;
bool DS18B20_found = false;
bool TSL2561_found = false;

// Nine degrees of freedom (accel, gyro and magnetometer)
bool LSM9DS1 = false;


// Start with heat off.
bool Heat_Enable = false;

void beep_piezo(unsigned int, unsigned long, unsigned int);
void configure_GPS_NMEA();



template <typename T, size_t N>

size_t countof( T (&array)[N] )

{

    return N;

}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configure_TSL2561(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  /*Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");*/
}



void setup()
{
    pinMode(PIEZO_PIN, OUTPUT);
    pinMode(MAIN_HEATER_PIN, OUTPUT);
    beep_piezo(1000, 1000, PIEZO_PIN);

    Serial.begin(9600);
    Serial.println(F("Restarting HAB Controller"));
    Serial.println(F("Initializing GPS..."));
    Serial1.begin(9600);

    configure_GPS_NMEA();
    Serial.print(F("Hardware Serial buffer size: "));
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    #ifdef USE_LCD
        lcd.begin (20,4); //  <<----- My LCD was 16x2
    #endif

    DS18B20.begin();

    if (DS18B20.getDeviceCount() > 0) {
        DS18B20_found = true;

        // Set to global desired resolution.
        DS18B20.setResolution(DS18B20_RESOLUTION);

        // Don't wait for conversion
        DS18B20.setWaitForConversion(false);

        // Send the command to get temperatures so that the conversion complete flag
        // can be used in the main loop
        DS18B20.requestTemperatures();
    }






    //mag3110.config();

    #ifdef USE_LCD
        // Switch on the backlight
        lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
        //lcd.setBacklight(255);
        lcd.backlight();

        lcd.home (); // go home
    #endif

    //lcd.autoscroll();
    // lcd.print("SainSmartI2C16x2");
     //lcd.print("Super duper long thingamabobish!!!");

    /*
    if(!accel.begin()) {

        Serial.println(F("Ooops, no ADXL345 detected ... Check your wiring!"));
    }
    */

    if(ms5607.connect()>0) {
        Serial.println(F("Error connecting to MS5607..."));
        // Do something about fail here.

    } else {
        // Load calibration from ROM of sensor.
        ms5607.ReadProm();

        // We found it
        ms5607_found = true;
    }

    /*
    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_6_25HZ);
    accel.setCorrection(ADXL345_CORRECTION);
    */

    // Start and check for humidity sensor
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println(F("Couldn't find SHT31"));
        // Do something here about fail.
    } else {
        // We found the  humidity sensor.
        SHT31_found = true;
    }

    /* Initialise the sensor */
    if(!tsl.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));

    } else {
        TSL2561_found = true;
        /* Setup the sensor gain and integration time */
        configure_TSL2561();
    }

    // Before initializing the DOF, there are a few settings
    // we may need to adjust. Use the settings struct to set
    // the device's communication mode and addresses:
    DOF.settings.device.commInterface = IMU_MODE_I2C;
    DOF.settings.device.mAddress = LSM9DS1_M;
    DOF.settings.device.agAddress = LSM9DS1_AG;

    // The above lines will only take effect AFTER calling
    // DOF.begin(), which verifies communication with the DOF
    // and turns it on.
    if (!DOF.begin()) {
        Serial.println(F("Failed to communicate with LSM9DS1."));
        Serial.println(F("Double-check wiring."));


    }

    // Start the fuel gauge
    fuel_gauge.begin();

    fuel_gauge.quickStart();

    // First analog reading can be inaccurate due to ADC capacitor needing to be primed, so start it up.
    analogRead(UV_PIN);

}

void beep_piezo(unsigned int freq, unsigned long millisecs, unsigned int pin) {
    // Buzzer harmonics seem to be around 2000 Hz for max volume. 3500-4000 also work.
    // How long to go high or low for frequency
    unsigned long squareWave_us = (1000000L / (2 * freq));
    // How many loops to beep for?
    unsigned int duration_loops = (unsigned int)(freq * (1000L / millisecs));

    Serial.print(F("Freq: "));
    Serial.print(freq);
    Serial.print(F(" Square: "));
    Serial.print(squareWave_us);
    Serial.print(F(" Duration: "));
    Serial.println(duration_loops);

    for (unsigned long i{0}; i < duration_loops; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(squareWave_us);
        digitalWrite(pin, LOW);
        delayMicroseconds(squareWave_us);
    }


}



void read_GPS() {
    /* TODO: Add check if time chip has been set from GPS values.
     * Better accuracy from reliable GPS settings.
     */
    //uint8_t available = Serial1.available();
    char read_char;
    //Serial.print(available);
    //Serial.print(F(" "));
    while(Serial1.available()) {
        read_char = Serial1.read();
            //gps.encode(Serial1.read());
        gps.encode(read_char);
        //Serial.print(read_char);
    }

    if (gps.location.isValid()) {
        sensors.gps.lat = gps.location.lat();
        sensors.gps.lng = gps.location.lng();
    } else {
        sensors.gps.lat = 0;
        sensors.gps.lng = 0;
    }

    if (gps.altitude.isValid()) {
        sensors.gps.altitude = gps.altitude.meters();
    } else {
        sensors.gps.altitude = 0;
    }

    if (gps.date.isValid()) {
        sensors.gps.date = gps.date.value();
    } else {
        sensors.gps.date = 0;
    }

    if (gps.time.isValid()) {
        sensors.gps.time = gps.time.value();
    } else {
        sensors.gps.time = 0;
    }

    if (gps.satellites.isValid()) {
        sensors.gps.sats = gps.satellites.value();
    } else {
        sensors.gps.sats = 0;
    }

    if (gps.hdop.isValid()) {
        sensors.gps.hdop = gps.hdop.value();
    } else {
        sensors.gps.hdop = 0;
    }

    if (gps.course.isValid()) {
        sensors.gps.course = gps.course.deg();
    } else {
        sensors.gps.course = -1;
    }

    if (gps.speed.isValid()) {
        sensors.gps.speed = gps.speed.mps();
    } else {
        sensors.gps.speed = -1;
    }

    //Serial.println(gps.failedChecksum());
}


void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  DOF.readAccel();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(DOF.calcAccel(DOF.ax), 2);
  Serial.print(", ");
  Serial.print(DOF.calcAccel(DOF.ay), 2);
  Serial.print(", ");
  Serial.print(DOF.calcAccel(DOF.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(DOF.ax);
  Serial.print(", ");
  Serial.print(DOF.ay);
  Serial.print(", ");
  Serial.println(DOF.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  DOF.readMag();

  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(DOF.calcMag(DOF.mx), 2);
  Serial.print(", ");
  Serial.print(DOF.calcMag(DOF.my), 2);
  Serial.print(", ");
  Serial.print(DOF.calcMag(DOF.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(DOF.mx);
  Serial.print(", ");
  Serial.print(DOF.my);
  Serial.print(", ");
  Serial.println(DOF.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}


void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  DOF.readGyro();

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(DOF.calcGyro(DOF.gx), 2);
  Serial.print(", ");
  Serial.print(DOF.calcGyro(DOF.gy), 2);
  Serial.print(", ");
  Serial.print(DOF.calcGyro(DOF.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(DOF.gx);
  Serial.print(", ");
  Serial.print(DOF.gy);
  Serial.print(", ");
  Serial.println(DOF.gz);
#endif
}

void readSensors() {

    uint16_t ir, broadband = 0;

    read_GPS();

    // To read from the accelerometer, you must first call the
    // readAccel() function. When this exits, it'll update the
    // ax, ay, and az variables with the most current data.
    DOF.readAccel();

    sensors.accel.x = DOF.calcAccel(DOF.ax);
    sensors.accel.y = DOF.calcAccel(DOF.ay);
    sensors.accel.z = DOF.calcAccel(DOF.az);

    // To read from the magnetometer, you must first call the
    // readMag() function. When this exits, it'll update the
    // mx, my, and mz variables with the most current data.
    DOF.readMag();

    sensors.mag.x = DOF.calcMag(DOF.mx);
    sensors.mag.y = DOF.calcMag(DOF.my);
    sensors.mag.z = DOF.calcMag(DOF.mz);

    // To read from the gyroscope, you must first call the
    // readGyro() function. When this exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    DOF.readGyro();

    sensors.gyro.x = DOF.calcGyro(DOF.gx);
    sensors.gyro.y = DOF.calcGyro(DOF.gy);
    sensors.gyro.z = DOF.calcGyro(DOF.gz);

    if (SHT31_found == true) {
        sensors.sht31_t = sht31.readTemperature();
        sensors.sht31_h = sht31.readHumidity();
    }


    if (ms5607_found == true && ms5607.Read_CRC4() == ms5607.Calc_CRC4()) {
        sensors.ms5607_pressure = ms5607.GetPres();
        sensors.ms5607_temp = ms5607.GetTemp();

        // Read the altitude sensor values into object for next round?
        ms5607.Readout();

    } else {
        sensors.ms5607_pressure = 0;
        sensors.ms5607_temp = 1000;
    }

    // Check if temp is available
    if (DS18B20.isConversionAvailable(0)) {
        sensors.DS18B20_temp_c[0] = DS18B20.getTempCByIndex(0);
        DS18B20.requestTemperaturesByIndex(0); // Send the command to get temperatures
    } else {
        sensors.DS18B20_temp_c[0] = 1000;
    }

    sensors.voltage = fuel_gauge.getVoltage();
    sensors.soc = fuel_gauge.getSOC();

    if (TSL2561_found == true) {
        // Get a new sensor event
        //sensors_event_t event;
        //tsl.getEvent(&event);
        //Serial.print(event.light);
        tsl.getLuminosity(&broadband, &ir);
        sensors.tsl.broadband = broadband;
        sensors.tsl.ir = ir;
        sensors.tsl.lux = tsl.calculateLux(broadband, ir);
    }

    sensors.uv = analogRead(UV_PIN);

}


uint8_t GPS_CRC(uint8_t *payload, uint16_t length) {
    // Calculate CRC for GPS payload.
    uint8_t crc = 0;

    for (uint16_t i = 0; i < length; i++) {
        //Serial.print(i);
        //Serial.print(F(" "));
        //Serial.print(payload[i], HEX);
        crc ^= payload[i];
        //Serial.print(" ");
        //Serial.println(crc, HEX);
    }

    return crc;
}



void configure_GPS_NMEA() {

    // Start message
    uint8_t start_message[4] = {GPS_SBYTE_1, GPS_SBYTE_2, 0, 9};

    // Update every 5 seconds
    // Payload for update frequency is of the form
    //                = {NEMA ID, GGA, GSA, GSV, GLL, RMC, VTG, ZDA, Attrib}
    uint8_t payload[9] = {0x08,     2,   2,   0,   0,   2,   2,  0,   0};

    // Calculate the crc of payload
    uint8_t crc = GPS_CRC(payload, 9);

    // Trailer for config
    uint8_t end_message[3] = {crc, GPS_END_1, GPS_END_2};

    // Bang it out to the GPS
    if (Serial1.availableForWrite() >= 16) {
        Serial1.write(start_message, 4);
        Serial1.write(payload, 9);
        Serial1.write(end_message, 3);

        /*
        Serial.print("Start: ");

        for (uint8_t i = 0; i < 4; i++) {
            Serial.print(start_message[i], HEX);
            Serial.print(F(" "));
        }

        Serial.println();
        Serial.print(F("Payload: "));

        for (uint8_t i = 0; i < 9; i++) {
            Serial.print(payload[i], HEX);
            Serial.print(F(" "));
        }

        Serial.println();
        Serial.print(F("End: "));

        for (uint8_t i = 0; i < 3; i++) {
            Serial.print(end_message[i], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
        */
    }
}



void check_battery() {
    // Check battery capacity and do appropriate things.

    if (sensors.voltage <= 3.2 || sensors.soc < 20) {
        // Turn off CO2 sensor as well.
        Heat_Enable = false;

    } else if (sensors.voltage > 3.4 && sensors.soc > 25) {
        Heat_Enable = true;
    }
}


void print_sensors() {

        if (sensors.voltage != 0) {
            Serial.print(F("Voltage: "));
            Serial.print(sensors.voltage);
            Serial.print(F("V "));
        }

        if (sensors.soc <= 100) {
            Serial.print(sensors.soc);
            Serial.print(F("%"));
        }

        Serial.println();

        if (sensors.sht31_h > 0) {  // check if 'is not a number'
            Serial.print(sensors.sht31_h);
        } else {
            Serial.print(F("XX"));
        }

        Serial.print(F("% "));

        if (sensors.sht31_t != 1000) {  // check if 'is not a number'
            Serial.print(sensors.sht31_t * 1.8 + 32);
            Serial.println(F("F "));
        } else {
            Serial.println(F("XX"));
        }

        Serial.print(F("F L:"));
        Serial.print(sensors.tsl.lux);
        Serial.print(F("/"));
        Serial.print(sensors.tsl.ir);
        Serial.print(F("/"));
        Serial.print(sensors.tsl.broadband);
        Serial.print(F("/"));
        Serial.println(sensors.uv);


        Serial.print(F("MX:"));
        Serial.print(sensors.mag.x);
        Serial.print(F("/"));
        Serial.print(sensors.mag.y);
        Serial.print(F("/"));
        Serial.println(sensors.mag.z);

        Serial.print(F("A:"));
        Serial.print(sensors.accel.x);
        Serial.print(F("/"));
        Serial.print(sensors.accel.y);
        Serial.print(F("/"));
        Serial.println(sensors.accel.z);

        Serial.print(F("G:"));
        Serial.print(sensors.gyro.x);
        Serial.print(F("/"));
        Serial.print(sensors.gyro.y);
        Serial.print(F("/"));
        Serial.println(sensors.gyro.z);

        Serial.print(sensors.ms5607_pressure);
        Serial.print(F("Pa "));
        Serial.print((sensors.ms5607_temp * 0.018) + 32);
        Serial.println(F("F "));


        Serial.print(F("DS18B20: "));
        Serial.print(sensors.DS18B20_temp_c[0] * 1.8 + 32);
        Serial.println(F("F "));

        Serial.print(sensors.gps.date);
        Serial.print(F(", "));
        Serial.print(sensors.gps.time);
        Serial.print(F(", "));

        Serial.print(sensors.gps.lat, 6);
        Serial.print(F(", "));
        Serial.print(sensors.gps.lng, 6);

        Serial.print(F(", "));
        Serial.print(sensors.gps.altitude, 6);
        Serial.print(F(" m"));
        Serial.print(F(", "));
        Serial.print(sensors.gps.speed);
        Serial.print(F("mps, "));
        Serial.print(sensors.gps.course);
        Serial.print(F("deg, "));

        Serial.print(sensors.gps.sats);
        Serial.print(F(", "));
        Serial.print(sensors.gps.hdop);
        Serial.println(F("m"));
        Serial.print(F("CRC ERROR: "));
        Serial.println(gps.failedChecksum());

        Serial.print(F("Up time: "));
        Serial.print(static_cast<unsigned long>(millis() / 1000L));
        Serial.println(F(" seconds"));
        Serial.println();
}


void loop() {



    readSensors();
    check_battery();


    #ifdef DEBUG
        print_sensors();


    #endif // DEBUG

    //delay(1000);


}
