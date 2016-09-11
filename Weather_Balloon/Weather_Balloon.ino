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
#include <Wire.h>
#include <LCD.h>
#include "LiquidCrystal_I2C.h"
#include "Adafruit_SI1145.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "MAG3110.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pin numbers for LCD display *AT THE DISPLAY END* not for the Arduino.
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

// One Wire data is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// UV sensor is on analog A0
#define UV_PIN  A0

// Piezo pin
#define PIEZO_PIN   3

// LCD stuff
#define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define COLS      20
#define ROWS       4
#define PAUSE    400

/*  DS18B20 resolution / conversion times
 *  9 =  93.75 ms
 * 10 = 187.5  ms
 * 11 = 375    ms
 * 12 = 750    ms
*/
#define DS18B20_RESOLUTION 10

// Calculated correction factor for ADXL345
const double ADXL345_CORRECTION  = 0.911265;

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

// Create LCD
LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);



void setup()
{
    pinMode(PIEZO_PIN, OUTPUT);

    Serial.begin(9600);

    lcd.begin (20,4); //  <<----- My LCD was 16x2
    DS18B20.begin();

    // Set to global desired resolution.
    DS18B20.setResolution(DS18B20_RESOLUTION);

    // Don't wait for conversion
    DS18B20.setWaitForConversion(false);

    DS18B20.requestTemperatures(); // Send the command to get temperatures

    mag3110.config();

    // Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    //lcd.setBacklight(255);
    lcd.backlight();

    lcd.home (); // go home
    //lcd.autoscroll();
    // lcd.print("SainSmartI2C16x2");
     //lcd.print("Super duper long thingamabobish!!!");

    if (! uv.begin()) {
        Serial.println(F("Didn't find Si1145"));
        lcd.print(F("Didn't find Si1145"));
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

    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_6_25HZ);
    accel.setCorrection(ADXL345_CORRECTION);

    // "Normal" range (indoors) instead of "high" gain (outdoors) for visible and IR gain. Read-modify-write registers.
    uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
    uv.setRegister(SI1145_PARAM_ALSIRADCMISC, IRADC_value & 0b11011111);
    uv.setRegister(SI1145_PARAM_ALSVISADCMISC, uv.getRegister(SI1145_PARAM_ALSVISADCMISC) & 0b11011111);

    // First analog reading can be inaccurate due to ADC capacitor needing to be primed, so start it up.
    analogRead(UV_PIN);
}

void beep_small_piezo(unsigned int freq, unsigned long millisecs) {
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
        digitalWrite(PIEZO_PIN, HIGH);
        delayMicroseconds(squareWave_us);
        digitalWrite(PIEZO_PIN, LOW);
        delayMicroseconds(squareWave_us);
    }
}


void loop() {
    unsigned long visible = uv.readVisible();
    unsigned long infrared = uv.readIR();
    unsigned long ultraViolet = uv.readUV();
    //uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
    float DS18B20_temp_f{0};



    // Check if temp is available
    if (DS18B20.isConversionAvailable(0)) {
        DS18B20_temp_f = DS18B20.getTempFByIndex(0);
        DS18B20.requestTemperaturesByIndex(0); // Send the command to get temperatures
    }



    //if (visible == 0 || infrared == 0) {
    //uv.reset();
    //}

    // Get a new sensor event
    sensors_event_t event;
    accel.getEvent(&event);

    lcd.clear();
    //lcd.setCursor(0, 0);

    lcd.print(F("L:"));
    lcd.print(visible);
    //lcd.setCursor(0, 1);
    lcd.print(F("/"));
    lcd.print(infrared);
    //lcd.setCursor(0, 2);
    lcd.print(F("/"));
    lcd.print(ultraViolet);
    //lcd.setCursor(0,);
    lcd.print(F("/"));
    lcd.print(analogRead(UV_PIN) * 5);
    //lcd.print(F(" "));


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
    lcd.print(DS18B20_temp_f);
    lcd.print(F("F"));
    //uv.nop();
    uv.force_convert();
    //delay(100);


}

