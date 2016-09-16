/*
** Example Arduino sketch for SainSmart I2C LCD Screen 16x2
** based on https://bitbucket.org/celem/sainsmart-i2c-lcd/src/3adf8e0d2443/sainlcdtest.ino
** by
** Edward Comer
** LICENSE: GNU General Public License, version 3 (GPL-3.0)

** This example uses F Malpartida's NewLiquidCrystal library. Obtain from:
** https://bitbucket.org/fmalpartida/new-liquidcrystal

** Modified - Ian Brennan ianbren at hotmail.com 23-10-2012 to support Tutorial posted to Arduino.cc

** Written for and tested with Arduino 1.0
**
** NOTE: Tested on Arduino Uno whose I2C pins are A4==SDA, A5==SCL

*/



#include <Wire.h>/home/peanut/ownCloud-encfs/School/Robert/2016/Fall/CS 3560 Data Communications and Networks/Homework 1/
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

// Light sensor
Adafruit_SI1145 uv = Adafruit_SI1145();

// Create accelerometer object with arbitrary ID
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DS18B20(&oneWire);

// Create magnetometer
MAG3110 mag3110;

// Altimeter
MS5xxx ms5607(&Wire);

// Humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

#ifdef USE_LCD
    // Create LCD
    LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
#endif


bool SHT31_found = false;
bool ms5607_found = false;
bool uv_found = false;
bool mag3110_found = false;

// Nine degrees of freedom (accel, gyro and magnetometer)
bool triple_sensor_found = false;

void setup()
{
    pinMode(PIEZO_PIN, OUTPUT);
    pinMode(MAIN_HEATER_PIN, OUTPUT);

    Serial.begin(9600);
    Serial.println(F("Restarting HAB Controller"));
    #ifdef USE_LCD
        lcd.begin (20,4); //  <<----- My LCD was 16x2
    #endif

    DS18B20.begin();

    // Set to global desired resolution.
    DS18B20.setResolution(DS18B20_RESOLUTION);

    // Don't wait for conversion
    DS18B20.setWaitForConversion(false);

    // Send the command to get temperatures so that the conversion complete flag
    // can be used in the main loop
    DS18B20.requestTemperatures();

    mag3110.config();

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

    if (! uv.begin()) {
        Serial.println(F("Didn't find Si1145"));
        #ifdef USE_LCD
            lcd.print(F("Didn't find Si1145"));
            #endif
        /*
         * Setup no UV HERE
         */
    }

    if(!accel.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        /*
         * Set up accelerometer fail here!
         */
    }

    if(ms5607.connect()>0) {
        Serial.println("Error connecting to MS5607...");
        // Do something about fail here.

    } else {
        // Load calibration from ROM of sensor.
        ms5607.ReadProm();

        // We found it
        ms5607_found = true;
    }

    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_6_25HZ);
    accel.setCorrection(ADXL345_CORRECTION);

    // Start and check for humidity sensor
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println("Couldn't find SHT31");
        // Do something here about fail.
    } else {
        // We found the  humidity sensor.
        SHT31_found = true;
    }

    // "Normal" range (indoors) instead of "high" gain (outdoors) for visible and IR gain. Read-modify-write registers.
    uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
    uv.setRegister(SI1145_PARAM_ALSIRADCMISC, IRADC_value & 0b11011111);
    uv.setRegister(SI1145_PARAM_ALSVISADCMISC, uv.getRegister(SI1145_PARAM_ALSVISADCMISC) & 0b11011111);

    // First analog reading can be inaccurate due to ADC capacitor needing to be primed, so start it up.
    analogRead(UV_PIN);


    analogWrite(MAIN_HEATER_PIN, 255);
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


void loop() {
    uint16_t visible = uv.readVisible();
    uint16_t infrared = uv.readIR();
    uint16_t ultraViolet = uv.readUV();
    //uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
    float DS18B20_temp_f{0};

    if (SHT31_found == true) {
        float sht31_t = sht31.readTemperature();
        float sht31_h = sht31.readHumidity();
    }

    if (ms5607_found == true) {
        double ms5607_pressure{ms5607.GetPres()};
        double ms5607_temp{ms5607.GetTemp()};

        // Read the altitude sensor values into object.
        ms5607.Readout();
    }


    // Check if temp is available
    if (DS18B20.isConversionAvailable(0)) {
        DS18B20_temp_f = DS18B20.getTempFByIndex(0);
        DS18B20.requestTemperaturesByIndex(0); // Send the command to get temperatures
    }


    if (mag3110_found == true) {
        int mag3110_x = mag3110.read_x();
        int mag3110_y = mag3110.read_y();
        int mag3110_z = mag3110.read_z();
    }

    //if (visible == 0 || infrared == 0) {
    //uv.reset();
    //}

    // Get a new sensor event
    sensors_event_t event;
    accel.getEvent(&event);

    /*
    Serial.print(F("Vis: "));
    Serial.print(visible);
    Serial.print(F(" IR: "));
    Serial.print(infrared);
    Serial.print(F(" UV: "));
    Serial.print(ultraViolet);
    Serial.print(F(" "));
    Serial.print(DS18B20_temp_f);
    Serial.println(F("F"));
    */

    #ifdef USE_LCD
        lcd.clear();
        //lcd.setCursor(0, 0);

        if (! isnan(sht31_h)) {  // check if 'is not a number'
            lcd.print(sht31_h);
        } else {
            lcd.print(F("XX"));
        }

        lcd.print(F("% "));

        if (! isnan(sht31_t)) {  // check if 'is not a number'
            lcd.print(sht31_t * 1.8 + 32);
        } else {
            lcd.print(F("XX"));
        }

        lcd.print(F("F L:"));
        lcd.print(visible);
        //lcd.setCursor(0, 1);
        lcd.print(F("/"));
        lcd.print(infrared);
        //lcd.setCursor(0, 2);
        //lcd.print(F("/"));
        //lcd.print(ultraViolet);
        //lcd.setCursor(0,);
        //lcd.print(F("/"));
        //lcd.print(analogRead(UV_PIN) * 5);
        //lcd.print(F(" "));




        lcd.setCursor(0, 1);
        lcd.print(F("MX:"));
        lcd.print(mag3110.read_x());
        //lcd.setCursor(11, 1);
        lcd.print(F("/"));
        lcd.print(mag3110.read_y());
        //lcd.setCursor(11, 2);
        lcd.print(F("/"));
        lcd.print(mag3110.read_z());

        //lcd.setCursor(0, 2);


        lcd.setCursor(0, 2);
        lcd.print(F("A:"));
        lcd.print(event.acceleration.x);
        lcd.print(F("/"));
        lcd.print(event.acceleration.y);
        lcd.print(F("/"));
        lcd.print(event.acceleration.z);


        lcd.setCursor(0, 3);

        /*
        if (ms5607.Read_CRC4() == ms5607.Calc_CRC4()) {
            lcd.print(ms5607_pressure);
            lcd.print(F("Pa "));
            //lcd.print((ms5607.GetTemp() * 0.018) + 32);
            //lcd.print(F("F "));
        } else {
            lcd.print(F("CRC !="));
        }

        lcd.print(DS18B20_temp_f);
        lcd.print(F("F "));
        //uv.nop();
        */

        lcd.print(analogRead(CO2_PIN));
    #endif

    uv.force_convert();
    delay(500);


}
