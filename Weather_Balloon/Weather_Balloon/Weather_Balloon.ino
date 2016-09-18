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


void beep_piezo(unsigned int, unsigned long, unsigned int);

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



/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
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
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}



void setup()
{
    pinMode(PIEZO_PIN, OUTPUT);
    pinMode(MAIN_HEATER_PIN, OUTPUT);
    beep_piezo(1000, 1000, PIEZO_PIN);

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
        /* Setup the sensor gain and integration time */
        configureSensor();
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


    analogWrite(MAIN_HEATER_PIN, 127);
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


void loop() {
    float DS18B20_temp_f, sht31_t, sht31_h, light, voltage, soc = 0;
    double ms5607_pressure, ms5607_temp = 0;
    int mag3110_x, mag3110_y, mag3110_z = 0;
    uint16_t ir, broadband = 0;

    if (SHT31_found == true) {
        sht31_t = sht31.readTemperature();
        sht31_h = sht31.readHumidity();
    }

    if (ms5607_found == true) {
        ms5607_pressure = ms5607.GetPres();
        ms5607_temp = ms5607.GetTemp();

        // Read the altitude sensor values into object.
        ms5607.Readout();
    }


    // Check if temp is available
    if (DS18B20.isConversionAvailable(0)) {
        DS18B20_temp_f = DS18B20.getTempFByIndex(0);
        DS18B20.requestTemperaturesByIndex(0); // Send the command to get temperatures
    }

    /*
    if (mag3110_found == true) {
        mag3110_x = mag3110.read_x();
        mag3110_y = mag3110.read_y();
        mag3110_z = mag3110.read_z();
    }
    */

    //if (visible == 0 || infrared == 0) {
    //uv.reset();
    //}

    voltage = fuel_gauge.getVoltage();
    soc = fuel_gauge.getSOC();

    // Get a new sensor event
    /*
    sensors_event_t event;
    accel.getEvent(&event);
    */

    tsl.getLuminosity(&broadband, &ir);
    light = tsl.calculateLux(broadband, ir);

    #ifdef DEBUG

        if (voltage != 0) {
            Serial.print(F("Voltage: "));
            Serial.print(voltage);
            Serial.print(F("V "));
            Serial.print(soc);
            Serial.println(F("%"));
        }

        if (! isnan(sht31_h)) {  // check if 'is not a number'
            Serial.print(sht31_h);
        } else {
            Serial.print(F("XX"));
        }

        Serial.print(F("% "));

        if (! isnan(sht31_t)) {  // check if 'is not a number'
            Serial.print(sht31_t * 1.8 + 32);
            Serial.print(F("F "));
        } else {
            Serial.print(F("XX"));
        }

        /*Serial.print(F("F L:"));
        Serial.print(visible);
        Serial.print(F("/"));
        Serial.print(infrared);
        Serial.print(F("/"));*/
        Serial.println(analogRead(UV_PIN));

        /*
        Serial.print(F("MX:"));
        Serial.print(mag3110_x);
        Serial.print(F("/"));
        Serial.print(mag3110_y);
        Serial.print(F("/"));
        Serial.println(mag3110_z);

        Serial.print(F("A:"));
        Serial.print(event.acceleration.x);
        Serial.print(F("/"));
        Serial.print(event.acceleration.y);
        Serial.print(F("/"));
        Serial.println(event.acceleration.z);
        */


        if (ms5607.Read_CRC4() == ms5607.Calc_CRC4()) {
            Serial.print(ms5607_pressure);
            Serial.print(F("Pa "));
            Serial.print((ms5607_temp * 0.018) + 32);
            Serial.println(F("F "));
        } else {
            Serial.print(F("CRC !="));
        }

        Serial.print(F("DS18B20: "));
        Serial.print(DS18B20_temp_f);
        Serial.println(F("F "));
        Serial.print(F("CO2: "));
        Serial.println(analogRead(CO2_PIN));

        if (light) {
            Serial.print(light);
            Serial.print(" lux. Broadband: ");
            Serial.print(broadband);
            Serial.print(F(" IR: "));
            Serial.println(ir);
        } else {
            /* If event.light = 0 lux the sensor is probably saturated
             * and no reliable data could be generated! */
            Serial.println("Sensor overload");
        }

        printGyro();  // Print "G: gx, gy, gz"
        printAccel(); // Print "A: ax, ay, az"
        printMag();   // Print "M: mx, my, mz"

        // Print the heading and orientation for fun!
        // Call print attitude. The LSM9DS1's magnetometer x and y
        // axes are opposite to the accelerometer, so my and mx are
        // substituted for each other.
        printAttitude(DOF.ax, DOF.ay, DOF.az, -DOF.my, -DOF.mx, DOF.mz);

        Serial.print(F("Up time: "));
        Serial.print(static_cast<unsigned long>(millis() / 1000L));
        Serial.println(F(" seconds"));
        Serial.println();
    #endif // DEBUG

    //delay(1000);


}
