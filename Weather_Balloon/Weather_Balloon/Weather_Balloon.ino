/* High Altitude Weather Balloon Controller
 * Controller to log sensor data to an SD card and allow location finding through
 * sound and lights.
 * Possible data transmission to a ground station through a radio transmitter.
 *
 * CS4360 Fall 2016
 * Licensed GPLv3
 * Robert Susmilch
*/

#define DEBUG 1

#include <stdint.h>
#include <Wire.h>
//#include <SPI.h>
//#define USE_LCD
#include <util/atomic.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>

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

// Definitions
void beep_piezo(unsigned int, unsigned long, unsigned int);
void configure_GPS_NMEA();
void configure_GPS_WAAS();
uint32_t find_sdcard_tail(const uint32_t, const uint32_t);


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


/* Remote Heaters
 * Arduino board
 * Daughter board (heater control, level shifting, sensor power cutoff) with battery
 * Humidity
 * UV
 * IR, Visible
 * GPS (though draws 90 ma at 3.3v...)
 * 3x Possibly Geiger
 */


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
    struct {
        // Initialize sensors if we found them or not
        bool LSM9DS1 = false;
        bool SHT31 = false;
        bool MS5607 = false;
        bool UV = false;
        bool DS18B20 = false;
        bool TSL2561 = false;
        bool GPS = false;

    } found;

    // Temperature sensors
    float DS18B20_temp_c[1] = {0};

    struct {
        // Humidity sensor
        float temp, humidity = 0;
    } sht31;

    // UV light output
    float uv = 0;


    struct {
        // Battery voltage and state-of-charge
        float voltage, soc = 0;
    } battery;

    struct {
        // Pressure and temp of pressure sensor
        double pressure, temp = 0;
    } ms5607;



    struct {
        // Infrared and broadband light readings
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


    struct {
        // Geiger counts for X, Y, Z axis.
        // When time to log data, copy data from count to axis, zero count.
        // Protects from losing counts while writing log (very slow.)
        // These are the values we will data log.
        uint16_t x, y, z = 0;

    } geiger;

    struct {
        double lat, lng = 0;
        double altitude = 0;
        double speed, course = 0;

        // This seems to be the value returned by the NMEA sentence * 100.
        // Eg, 3.2 * 100 * 320.
        // However, it is supposed to be a multiplication value [00.0,99.9]
        // and multiplied by the absolute accuracy of the receiver. The
        // Venus datasheet claims 2.5m accuracy, so 3.2*2.5 = 8 m (26.2 ft)
        // This seems high, but this value is indoors as well, so will need
        // to be tested outside with a clear sky view.
        uint16_t hdop = 0;
        uint16_t vdop = 0;

        uint8_t sats = 0;

        uint32_t date = 0;
        uint32_t time = 0;

    } gps;

    // Maximum value for heater control.
    uint8_t heater_max = 255;

    struct {
        // Which sensors are enabled?
        bool LSM9DS1 = false;
        bool SHT31 = false;
        bool MS5607 = false;
        bool UV = true;
        bool DS18B20 = false;
        bool TSL2561 = false;
        bool GPS = true;

    } enabled;

    uint16_t crc;

} data_struct;

typedef struct {
        // Geiger counts for X, Y, Z axis.
        // When time to log data, copy data from count to axis, zero count.
        // Protects from losing counts while writing log (very slow.)
        // These are the values we will data log.

        // These are in use counters.
        volatile uint16_t x_count, y_count, z_count = 0;
} geiger_temp;

// Create global sensor structure
data_struct dataLog;

geiger_temp geiger;

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

Sd2Card sdcard;

typedef struct {
    // Number of 512 byte blocks reported by sdcard.
    uint32_t blocks = 0;

    // Current block on the card.
    // *CANNOT* write to block zero, sd card configuration block.
    uint32_t current_block = 1;

    uint32_t write_errors, crc_errors = 0;

    // CRC byte
    int16_t crc;

    // Buffer to hold for reading, writing, finding tail.
    uint8_t buffer[512];

    // Buffer to hold data waiting to be written between logging calls.
    uint8_t hold_buffer[512];

    // This holds the number of "structures" that have been packed into a
    // 512 byte block.
    uint8_t num_packed = 0;

} sd_data;

sd_data sdcard_data;

typedef struct {
    uint8_t queue_loc = 0;

    uint16_t address = 0;


} eeprom_struct;

eeprom_struct eeprom;

/* Create GPS object
 * REQUIRES CUSTOM LARGER BUFFER FOR INCOMING UART
*/
TinyGPSPlus gps;

// Grab vertical dilution of precision.
TinyGPSCustom vdop(gps, "GPGSA", 17);


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
#define GPS_CONFIG_WAAS 0x37

#define GPS_ACK         0x83
#define GPS_NACK        0x84

// GPS precision is within 2.5 meters. For dilution of precision
#define GPS_PRECISION   2.5

#ifdef USE_LCD
    // Create LCD
    LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
#endif




// Start with heat off.
bool Heat_Enable = false;




template <typename T, size_t N>

size_t countof( T (&array)[N] )

{

    return N;

}

int16_t sd_calc_crc(const uint8_t* src, const uint8_t block_size) {
    int16_t crc, i, x = 0;

    // CRC16 code via Scott Dattalo www.dattalo.com
    for(crc = i = 0; i < block_size; i++) {
      x   = ((crc >> 8) ^ src[i]) & 0xff;
      x  ^= x >> 4;
      crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }

    return crc;
}


/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configure_TSL2561(void)
{
   if(!tsl.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));

    } else {
        dataLog.found.TSL2561 = true;

        dataLog.enabled.TSL2561 = true;

        /* Setup the sensor gain and integration time */
        /* You can also manually set the gain or enable auto-gain support */
        // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
        // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
        tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

        /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
        // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
        // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

    }


  /* Update these values depending on what you've set above! */
  /*Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");*/
}

ISR(PCINT1_vect) {
    /* Pin change interrupt for Geiger counters.
     * No falling or rising edge detection on these pins so we
     * need to track previous pin states.
     * When a pin has changed value we update the appropriate counter.
     * Geiger counter outputs a low pulse of 150 microseconds on particle detection.
     */

    // Previous pin value for port. Static to preserve between calls.
    static uint8_t previous_pins_j = 0;

    // Grab current pin state quickly. Low pulse from Geiger is only 150 microseconds.
    uint8_t current_state_j = PINJ;

    // If the current pin state is low, and previous it was high, we have
    // a falling edge.
    if ((current_state_j & (1<<PJ1)) == 0 && (previous_pins_j & (1<<PJ1)) > 0) {
      // Trigger count
      geiger.x_count++;
    }

    if ((current_state_j & (1<<PJ0)) == 0 && (previous_pins_j & (1<<PJ0)) > 0) {
      // Trigger count
      geiger.y_count++;
    }

    // Store for later.
    previous_pins_j = current_state_j;
}

ISR(PCINT2_vect) {
    /* Pin change interrupt for Geiger counters.
     * No falling or rising edge detection on these pins so we
     * need to track previous pin states.
     * When a pin has changed value we update the appropriate counter.
     * Geiger counter outputs a low pulse of 150 microseconds on particle detection.
     */

    // Previous pin value for port. Static to preserve between calls.
    static uint8_t previous_pins_k = 0;

    // Grab current pin state quickly. Low pulse from Geiger is only 150 microseconds.
    uint8_t current_state_k = PINK;

    // If the current pin state is low, and previous it was high, we have
    // a falling edge.
    if ((current_state_k & (1<<PK7)) == 0 && (previous_pins_k & (1<<PK7)) > 0) {
      // Trigger count
      geiger.z_count++;
    }

    // Store for later.
    previous_pins_k = current_state_k;
}

void configure_geiger() {
    // Enable pin interrupts for Geiger counter on

    // X axis
    // pin 64 (PJ1, PCINT10, TXD3), on PCIE1.
    // Arduino D14 (TX3)

    // Y-axis
    // pin 63 (PJ0, PCINT10, RXD3), on PCIE1.
    // Arduino D15 (RX3)

    // Z-axis
    // pin 63 (PK7, PCINT23, ADC15), on PCIE2.
    // Arduino A15 (ADC15)

    // Set as input for high impedance. However, this will allow
    // noise to trigger the pin if it is unconnected. The Geiger counter
    // is running on 3.3 volts, so a weak pull-up sends 5 volts through.
    // May not be healthy.
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    pinMode(A15, INPUT_PULLUP);

    // Clear interrupts
    cli();
    // Enable pin change interrupts for the needed port.
    PCICR |= (1<<PCIE1) | (1<<PCIE2);

    // Change the mask to allow interrupts ONLY for this pin.
    PCMSK1 |= (1<<PCINT10) | (1<<PCINT9);
    PCMSK2 |= (1<<PCINT23);

    // Set interrupts
    sei();

    // Now pin changes will call the ISR(PCINT1_vect) for X and Y.
    // and will call ISR(PCINT2_vect) for Z.

}

void configure_ds18b20() {
    // Configure Dallas temp dataLog.
    DS18B20.begin();

    if (DS18B20.getDeviceCount() > 0) {
        dataLog.found.DS18B20 = true;
        dataLog.enabled.DS18B20 = true;

        // Set to global desired resolution.
        DS18B20.setResolution(DS18B20_RESOLUTION);

        // Don't wait for conversion
        DS18B20.setWaitForConversion(false);

        // Send the command to get temperatures so that the conversion complete flag
        // can be used in the main loop
        DS18B20.requestTemperatures();
    }
}



void configure_ms5607() {
   if(ms5607.connect()>0) {
        Serial.println(F("Error connecting to MS5607..."));
        // Do something about fail here.

    } else {
        // Load calibration from ROM of sensor.
        ms5607.ReadProm();

        // We found it
        dataLog.found.MS5607 = true;
        dataLog.enabled.MS5607 = true;
    }
}


void configure_LSM9DS1() {
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
        dataLog.found.LSM9DS1 = false;

    } else {
        dataLog.found.LSM9DS1 = true;
        dataLog.enabled.LSM9DS1 = true;
        DOF.calibrate();
        DOF.calibrateMag();
    }

}

void configure_sht31() {

    // Start and check for humidity sensor
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println(F("Couldn't find SHT31"));
        dataLog.found.SHT31 = false;
        // Do something here about fail.
    } else {
        // We found the  humidity sensor.
        dataLog.found.SHT31 = true;
        dataLog.enabled.SHT31 = true;

        // Enable the heater, its cold up there.
        // Sensor performs best in 5-60 C temps.
        // However, that changes the relative humidity, there is no compensation.
        sht31.heater(true);
    }
}

void print_found() {
    // Print a list of all sensors found
    Serial.println(F("\nSensors found"));
    Serial.println(F("-------------"));
    Serial.print(F("DS18B20: "));
    Serial.println(dataLog.found.DS18B20);
    Serial.print(F(" MS5607: "));
    Serial.println(dataLog.found.MS5607);
    Serial.print(F("LSM9DS1: "));
    Serial.println(dataLog.found.LSM9DS1);
    Serial.print(F("  SHT31: "));
    Serial.println(dataLog.found.SHT31);
    Serial.print(F("TLS2561: "));
    Serial.println(dataLog.found.TSL2561);
    Serial.println();

    delay(1000);
}
void configure_sdcard() {

    if (sdcard.init(SPI_HALF_SPEED, SS_PIN)==0) {
        Serial.print(F("Something wrong initilizing SDCard...\nError: "));
        Serial.println(sdcard.errorCode(), HEX);
    } else {
        Serial.print(F("Card size: "));
        sdcard_data.blocks = sdcard.cardSize() - 1;
        Serial.print(sdcard_data.blocks);
        Serial.println(F(" blocks"));
    }

    uint32_t tail = find_sdcard_tail(1, sdcard_data.blocks);

    if (tail != 0) {
        sdcard_data.current_block = tail;
        Serial.print(F("\nFound tail of log at block: "));
        Serial.println(sdcard_data.current_block);
    }

}


void setup()
{
    pinMode(PIEZO_PIN, OUTPUT);
    pinMode(MAIN_HEATER_PIN, OUTPUT);
    beep_piezo(1000, 1000, PIEZO_PIN);

    Serial.begin(115200);
    Serial.println(F("Restarting HAB Controller"));

    Serial.print(F("Hardware Serial buffer size: "));
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    Serial.println(F("Configuring GPS..."));
    Serial1.begin(9600);

    configure_GPS_NMEA();
    configure_GPS_WAAS();

    Serial.println(F("Configuring SDCard..."));
    configure_sdcard();

    Serial.println(F("Configuring Geigers..."));
    configure_geiger();

    // Start and setup DS18B20s.
    Serial.println(F("Configuring DS18B20s..."));
    configure_ds18b20();

    // Pressure sensor
    Serial.println(F("Configuring MS5607..."));
    configure_ms5607();

    // Configure lux sensor
    Serial.println(F("Configuring TSL2561..."));
    configure_TSL2561();

    // Configure the accel, gyro and magnetometer sensor.
    Serial.println(F("Configuring LSM9DS1..."));
    configure_LSM9DS1();

    // Humidity sensor
    Serial.println(F("Configuring SHT31..."));
    configure_sht31();

    // Start the fuel gauge
    Serial.println(F("Configuring fuel gauge..."));
    fuel_gauge.begin();

    // Restart for more accurate reading.
    Serial.println(F("Quick-starting fuel gauge..."));
    fuel_gauge.quickStart();

    #ifdef DEBUG
        print_found();
    #endif // DEBUG

    // First analog reading can be inaccurate due to ADC capacitor needing to be primed, so start it up.
    analogRead(UV_PIN);

    // FOR TESTING BATTERY RUNTIME ONLY.
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
        dataLog.gps.lat = gps.location.lat();
        dataLog.gps.lng = gps.location.lng();
    } else {
        dataLog.gps.lat = 0;
        dataLog.gps.lng = 0;
    }

    if (gps.altitude.isValid()) {
        dataLog.gps.altitude = gps.altitude.meters();
    } else {
        dataLog.gps.altitude = 0;
    }

    if (gps.date.isValid()) {
        dataLog.gps.date = gps.date.value();
    } else {
        dataLog.gps.date = 0;
    }

    if (gps.time.isValid()) {
        dataLog.gps.time = gps.time.value();
    } else {
        dataLog.gps.time = 0;
    }

    if (gps.satellites.isValid()) {
        dataLog.gps.sats = static_cast<uint8_t>(gps.satellites.value());
    } else {
        dataLog.gps.sats = 0;
    }

    if (gps.hdop.isValid()) {
        dataLog.gps.hdop = static_cast<uint16_t>(gps.hdop.value() * GPS_PRECISION);
    } else {
        dataLog.gps.hdop = 0;
    }

    if (gps.course.isValid()) {
        dataLog.gps.course = gps.course.deg();
    } else {
        dataLog.gps.course = -1;
    }

    if (gps.speed.isValid()) {
        dataLog.gps.speed = gps.speed.mps();
    } else {
        dataLog.gps.speed = -1;
    }

    if (vdop.isValid()) {
        // Convert character buffer pointer to a double with null end pointer,
        // multiply by 100 to remove decimal, and then convert to integer.
        dataLog.gps.vdop = static_cast<uint16_t>(strtod(vdop.value(), NULL) * 100 * GPS_PRECISION);
        //Serial.println(vdop.value());
        //Serial.println(dataLog.gps.vdop);
    } else {
        dataLog.gps.vdop = 0;
    }
    //Serial.println(gps.failedChecksum());
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


void readSensors() {

    uint16_t ir, broadband = 0;

    read_GPS();

    // To read from the accelerometer, you must first call the
    // readAccel() function. When this exits, it'll update the
    // ax, ay, and az variables with the most current data.
    DOF.readAccel();

    dataLog.accel.x = DOF.calcAccel(DOF.ax);
    dataLog.accel.y = DOF.calcAccel(DOF.ay);
    dataLog.accel.z = DOF.calcAccel(DOF.az);

    // To read from the magnetometer, you must first call the
    // readMag() function. When this exits, it'll update the
    // mx, my, and mz variables with the most current data.
    DOF.readMag();

    dataLog.mag.x = DOF.calcMag(DOF.mx);
    dataLog.mag.y = DOF.calcMag(DOF.my);
    dataLog.mag.z = DOF.calcMag(DOF.mz);

    // To read from the gyroscope, you must first call the
    // readGyro() function. When this exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    DOF.readGyro();

    dataLog.gyro.x = DOF.calcGyro(DOF.gx);
    dataLog.gyro.y = DOF.calcGyro(DOF.gy);
    dataLog.gyro.z = DOF.calcGyro(DOF.gz);

    if (dataLog.found.SHT31 == true) {
        dataLog.sht31.temp = sht31.readTemperature();
        dataLog.sht31.humidity = sht31.readHumidity();
    }


    if (dataLog.found.MS5607 == true && ms5607.Read_CRC4() == ms5607.Calc_CRC4()) {
        dataLog.ms5607.pressure = ms5607.GetPres();
        dataLog.ms5607.temp = ms5607.GetTemp();

        // Read the altitude sensor values into object for next round?
        ms5607.Readout();

    } else {
        dataLog.ms5607.pressure = 0;
        dataLog.ms5607.temp = 1000;
    }

    // Check if temp is available
    if (dataLog.found.DS18B20 == true && DS18B20.isConversionAvailable(0)) {
        dataLog.DS18B20_temp_c[0] = DS18B20.getTempCByIndex(0);
        DS18B20.requestTemperaturesByIndex(0); // Send the command to get temperatures
    } else {
        dataLog.DS18B20_temp_c[0] = 1000;
    }

    dataLog.battery.voltage = fuel_gauge.getVoltage();
    dataLog.battery.soc = fuel_gauge.getSOC();

    if (dataLog.found.TSL2561 == true) {
        // Get a new sensor event
        //sensors_event_t event;
        //tsl.getEvent(&event);
        //Serial.print(event.light);
        tsl.getLuminosity(&broadband, &ir);
        dataLog.tsl.broadband = broadband;
        dataLog.tsl.ir = ir;
        dataLog.tsl.lux = tsl.calculateLux(broadband, ir);
    }

    dataLog.uv = analogRead(UV_PIN);

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

    // Update every 2 seconds
    // Payload for update frequency is of the form
    //                =          {NEMA ID, GGA, GSA, GSV, GLL, RMC, VTG, ZDA, Attrib}
    uint8_t payload[9] = {GPS_CONFIG_NMEA,   3,   10,   0,   0,   3,   3,  0,   0};

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

void configure_GPS_WAAS() {
    // Turn on WAAS

    // Start message
    uint8_t start_message[4] = {GPS_SBYTE_1, GPS_SBYTE_2, 0, 3};

    uint8_t payload[3] = {GPS_CONFIG_WAAS, 1, 0};

    // Calculate the crc of payload
    uint8_t crc = GPS_CRC(payload, 3);

    // Trailer for config
    uint8_t end_message[3] = {crc, GPS_END_1, GPS_END_2};

    // Bang it out to the GPS
    if (Serial1.availableForWrite() >= 16) {
        Serial1.write(start_message, 4);
        Serial1.write(payload, 3);
        Serial1.write(end_message, 3);


    }
}

void adjust_heaters() {
    // Adjust heaters checks the setpoint temperatures of various sensors
    // and changes the output appropriately to maintain setpoint.
}


void check_battery() {
    // Check battery capacity and do appropriate things.

    if (dataLog.battery.voltage <= 3.1 || dataLog.battery.soc < 10) {
        // Turn off CO2 sensor as well.
        Heat_Enable = false;


    } else if (dataLog.battery.voltage > 3.4 && dataLog.battery.soc > 25) {
        Heat_Enable = true;
    }
}

void heater_control() {

    if (Heat_Enable == false) {
        analogWrite(MAIN_HEATER_PIN, 0);
    }
}


void print_sensors() {

        if (dataLog.battery.voltage != 0) {
            Serial.print(F("Voltage: "));
            Serial.print(dataLog.battery.voltage);
            Serial.print(F(" V "));
        }

        if (dataLog.battery.soc <= 100) {
            Serial.print(dataLog.battery.soc);
            Serial.print(F("%"));
        }

        Serial.println();

        if (dataLog.sht31.humidity > 0) {  // check if 'is not a number'
            Serial.print(dataLog.sht31.humidity);
            Serial.print(F("% "));
        } else {
            Serial.print(F("XX"));
        }

        if (dataLog.sht31.temp != 1000) {  // check if 'is not a number'
            Serial.print(dataLog.sht31.temp * 1.8 + 32);
            Serial.println(F(" F "));
        } else {
            Serial.println(F("XX"));
        }

        Serial.print(F("L:"));
        Serial.print(dataLog.tsl.lux);
        Serial.print(F("/"));
        Serial.print(dataLog.tsl.ir);
        Serial.print(F("/"));
        Serial.print(dataLog.tsl.broadband);
        Serial.print(F("/"));
        Serial.println(dataLog.uv);


        Serial.print(F("M:"));
        Serial.print(dataLog.mag.x);
        Serial.print(F("/"));
        Serial.print(dataLog.mag.y);
        Serial.print(F("/"));
        Serial.println(dataLog.mag.z);

        Serial.print(F("A:"));
        Serial.print(dataLog.accel.x);
        Serial.print(F("/"));
        Serial.print(dataLog.accel.y);
        Serial.print(F("/"));
        Serial.println(dataLog.accel.z);

        Serial.print(F("G:"));
        Serial.print(dataLog.gyro.x);
        Serial.print(F("/"));
        Serial.print(dataLog.gyro.y);
        Serial.print(F("/"));
        Serial.println(dataLog.gyro.z);

        Serial.print(dataLog.ms5607.pressure);
        Serial.print(F(" Pa "));
        Serial.print((dataLog.ms5607.temp * 0.018) + 32);
        Serial.println(F(" F "));


        Serial.print(F("DS18B20: "));
        Serial.print(dataLog.DS18B20_temp_c[0] * 1.8 + 32);
        Serial.println(F(" F "));

        Serial.print(dataLog.gps.date);
        Serial.print(F(", "));
        Serial.print(dataLog.gps.time);
        Serial.print(F(", "));

        Serial.print(dataLog.gps.lat, 6);
        Serial.print(F(", "));
        Serial.print(dataLog.gps.lng, 6);

        Serial.print(F(", "));
        Serial.print(dataLog.gps.altitude, 6);
        Serial.print(F(" m, "));
        Serial.print(dataLog.gps.speed);
        Serial.print(F(" mps, "));
        Serial.print(dataLog.gps.course);
        Serial.print(F(" deg, "));

        Serial.print(dataLog.gps.sats);
        Serial.print(F(" sats, HDOP: "));
        Serial.print(static_cast<float>(dataLog.gps.hdop / 100), 1);
        Serial.print(F(" m, VDOP: "));
        Serial.print(static_cast<float>(dataLog.gps.vdop / 100), 1);
        Serial.println(F(" m"));
        Serial.print(F("CRC ERROR: "));
        Serial.println(gps.failedChecksum());

        Serial.print(F("Geiger: "));
        Serial.print(geiger.x_count);
        Serial.print(F("/"));
        Serial.print(geiger.y_count);
        Serial.print(F("/"));
        Serial.println(geiger.z_count);

        Serial.print(F("Up time: "));
        Serial.print(static_cast<unsigned long>(millis() / 1000L));
        Serial.println(F(" seconds\n"));

}

void power_control() {
    // Control power consumption with powering down sensors or limiting heater power.

}

void reset_sdcard() {
    // Toggle power to SD card here to reset?

}

uint8_t EEPROM_read(uint16_t uiAddress) {
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE)) {
    }

    /* Set up address register */
    EEAR = uiAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from Data Register */
    return EEDR;
}


void EEPROM_update(uint16_t uiAddress, uint8_t ucData) {
    // Write data to EEPROM unless the data is already the same at the address.

    uint8_t readEEPROM = EEPROM_read(uiAddress);

    if (readEEPROM == ucData) {
        // We already have that value written.
        return;
    }

    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE)) {
    }

    /* Set up address and Data Registers */
    EEAR = uiAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}


uint32_t find_sdcard_tail(const uint32_t first = 1, const uint32_t last = sdcard_data.blocks) {
    // On powerup look for the tail of the SD card log.
    // We use a binary search algorithm with a slight twist.

    bool not_found = true;
    bool left_empty = true;
    bool right_empty = true;

    // Current block, start at the beginning block (block 0 is out of bounds!)
    uint32_t current_search = first;

    // Get size of the data structure
    uint16_t data_size = sizeof(dataLog);

    // Can't have zero block. Holds SD card information. Would be a false short circuit.
    if (current_search == 0) {
        current_search = 1;
    }

    uint32_t left = current_search;
    uint32_t right = last;


    // Check left side is empty?
    // Read a partial block for data
    sdcard.readData(left, 0, data_size, sdcard_data.buffer);

    for (uint8_t i = 0; i < data_size; i++) {
        if (sdcard_data.buffer[i] != 0) {
            // It's not empty
            left_empty = false;

            // That's all we care about
            break;
        }
    }

    if (left_empty == true) {
        // It's empty, so return it.
        return left;
    }

    // Check right point for empty
    // Read a partial block for data
    sdcard.readData(right, 0, data_size, sdcard_data.buffer);

    for (uint8_t i = 0; i < data_size; i++) {
        //Serial.println(sdcard_data.buffer[i], HEX);

        if (sdcard_data.buffer[i] != 0) {

            // It's empty
            right_empty = false;

            // Abort, since we are looking for empty blocks
            break;
        }
    }

    if (right_empty == false) {
        // We can't get any more right, so return.
        return right;
    }


    while (not_found == true) {

        // Middle value
        current_search = (right + left) / 2;

        #ifdef DEBUG

            Serial.print(F("Left: "));
            Serial.print(left);
            Serial.print(F(" Current: "));
            Serial.print(current_search);
            Serial.print(F(" Right: "));
            Serial.println(right);

        #endif // DEBUG

        // Read a partial block for data
        sdcard.readData(current_search, 0, data_size, sdcard_data.buffer);

        for (uint8_t i = 0; i < data_size; i++) {
            if (sdcard_data.buffer[i] != 0) {
                // Block is not empty.
                not_found = true;

                // New left value is this current block
                left = current_search;

                left_empty = false;
                #ifdef DEBUG_MORE
                    sdcard_data.current_block++;
                    Serial.print(F("SDcard block "));
                    Serial.print(sdcard_data.current_block);
                    Serial.println(F(" NOT empty... skipping."));
                #endif // DEBUG_MORE
                break;
            }
        }


        if (left != current_search) {
            // Since left is not equal to current_search we assume it was empty
            right = current_search;
            right_empty = true;
        }

        if (left_empty == false && right_empty == true && right - left <= 1 ) {
            // Close enough
            not_found = false;
        }
    }

    return right;
}

void test_logging() {
    // Get size of the data structure
    static uint16_t data_size = sizeof(dataLog);

    Serial.print(F("\n\n\n*************************\nRead sensors from SDcard\n*************************\n\n\n"));
    for (uint8_t i = 0; i < 510/data_size; i++) {
        memcpy(&dataLog, &sdcard_data.buffer[i * data_size], data_size);

        print_sensors();
    }

    Serial.print(F("\n\n\n*************************\nEnd sensors from SDcard\n*************************\n\n\n"));

}


void log_data() {
    // Log the sensor data by packing it into a holding buffer.
    // When the buffer is full, we flush it to storage and reset
    // the holding buffer.

    uint16_t crc_before, crc_after, loops = 0;
    bool success, empty_block = false;

    // Get size of the data structure
    static uint16_t data_size = sizeof(dataLog);

    if (sdcard_data.num_packed == 0) {
        // Zero the SDcard buffer since we're starting fresh
        memset(sdcard_data.hold_buffer, 0, 512);
    }

    if (sdcard_data.num_packed * data_size < 510 - data_size) {
        // Check if we can squeeze another data packet into the buffer.

        // Make sure that optimizations don't fiddle with our ordering.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

            // Copy over Geiger counts for logging

            dataLog.geiger.x = geiger.x_count;
            dataLog.geiger.y = geiger.y_count;
            dataLog.geiger.z = geiger.z_count;

            // Zero interrupt counts
            geiger.x_count = 0 = geiger.y_count = geiger.z_count = 0;

        }

        // Copy the data log structure as a byte stream to the SD card buffer for later writing.
        memcpy(&sdcard_data.hold_buffer[sdcard_data.num_packed * data_size], &dataLog, data_size);

        // We packed another
        sdcard_data.num_packed++;

        #ifdef DEBUG
            Serial.print(F("Number of structures packed: "));
            Serial.println(sdcard_data.num_packed);
            Serial.print(F("SD card packing start address: "));
            Serial.println(sdcard_data.num_packed * data_size, DEC);
        #endif // DEBUG

    }

    if (sdcard_data.num_packed * data_size >= 510 - data_size) {
        // Test again to see if we're at the limit. If we are, we need to log it.
        // Otherwise we lose a log when we are called again.
        // We allow 2 bytes for the CRC16 at the end of the buffer.

        #ifdef DEBUG
            Serial.print(F("\nSensor data size: "));
            Serial.println(data_size);

            Serial.print(F("SD Block: "));
            Serial.print(sdcard_data.current_block);
            Serial.print(F("/"));
            Serial.println(sdcard_data.blocks);
        #endif // DEBUG

        //print_sensors();

        // We may want to try multiple data writes in case of a bad block.
        // Need to define how large a bad block flash memory can have.
        // In other words, if a write/erase block is 8MB, then multiple 512 byte
        // blocks will need to be skipped to get to "good" memory.
        // 8MB = 16384 pages of 512 bytes (we're writing to 512 byte blocks, so
        // so that's what we care about. Add 1 to get over any threshold.
        while(success == false && sdcard_data.current_block <= sdcard_data.blocks) {
            // Reset for loop
            success = true;

            // Zero the SDcard buffer
            memset(sdcard_data.buffer, 0, 512);

            //Serial.println(F("Reading block verify zero"));
            // Read a block of data to ensure we are writing to an empty block.
            sdcard.readBlock(sdcard_data.current_block, sdcard_data.buffer);

            empty_block = true;

            //Serial.println(F("Verify zero"));

            for (uint16_t i = 0; i < 512; i++) {
                // Iterate over the block, looking for non-zero values
                //Serial.println(sdcard_data.buffer[i], HEX);
                if (sdcard_data.buffer[i] != 0) {
                    // Not empty
                    empty_block = false;
                    success = false;

                    #ifdef DEBUG
                        sdcard_data.current_block++;
                        Serial.print(F("SDcard block "));
                        Serial.print(sdcard_data.current_block);
                        Serial.println(F(" NOT empty... skipping."));
                    #endif // DEBUG_MORE

                    // Break for next block.
                    break;
                }
            }

            if (empty_block == true) {
                // Copy the data log structure as a byte stream to the SD card buffer for later writing.
                // memcpy(&sdcard_data.buffer, &dataLog, data_size);

                //Serial.println(F("Calculating CRC..."));
                // Calculate the buffer's CRC16 for comparison after writing.
                crc_before = sd_calc_crc(sdcard_data.hold_buffer, data_size * (sdcard_data.num_packed + 1));

                // Tack on CRC to end of SDcard buffer
                memcpy(&sdcard_data.hold_buffer[511 - sizeof(crc_before)], &crc_before, sizeof(crc_before));

                // Write the buffer and check if returned successfully
                if (sdcard.writeBlock(sdcard_data.current_block, sdcard_data.hold_buffer) == 0) {
                    sdcard_data.write_errors++;
                    success = false;

                    // Had an error. Get error code and "status"
                    Serial.print(F("SDcard Write Error: "));
                    Serial.println(sdcard.errorCode(), HEX);
                    Serial.print(F("Status: "));
                    Serial.println(sdcard.errorData(), HEX);

                    // Reset the card in case it was a lock up error.
                    reset_sdcard();

                    // Try the next data block.
                    sdcard_data.current_block++;

                    loops++;

                    // Skip reading the data since we had a write error.
                    continue;
                }


                // Zero the buffer
                memset(sdcard_data.buffer, 0, 512);

                // Memset does not seem to zero data structures. However, we can copy over our freshly zeroed buffer no problems
                // to zero the sensor data.
                // memcpy(&dataLog, &sdcard_data.buffer, data_size);

                //Serial.println(F("\n\nAFTER ZEROING\n"));
                //print_sensors();

                // Read the data in for verification
                sdcard.readBlock(sdcard_data.current_block, sdcard_data.buffer);

                // Copy over the CRC we read
                memcpy(&crc_after, &sdcard_data.buffer[511 - sizeof(crc_before)], sizeof(crc_before));

                // Calculate the CRC after we read back the data.
                crc_after = sd_calc_crc(sdcard_data.buffer, data_size * (sdcard_data.num_packed + 1));

                if (crc_before != crc_after) {
                    // Test if the write and read were identical
                    // They were not!
                    sdcard_data.crc_errors++;

                    // We have a problem, crc mismatch.
                    Serial.println(F("\n************************\n*     CRC MISMATCH     *\n************************"));

                    success = false;

                    // Reset the card in case it was a lock up error.
                    reset_sdcard();

                    // Try the next data block.
                    sdcard_data.current_block++;

                    loops++;

                    // Skip to next loop since we had an error.
                    continue;

                } else {
                    // Success!
                    // Increase to next block.
                    sdcard_data.current_block++;
                }

                if (success == true) {
                    // We only write when the holding buffer is full, so
                    // reset the packing counter
                    sdcard_data.num_packed = 0;

                    #ifdef DEBUG
                        test_logging();
                    #endif // DEBUG
                }

                #ifdef DEBUG
                    Serial.print(F("****************\nCRC BEFORE: "));
                    Serial.println(crc_before, HEX);

                    Serial.print(F(" CRC AFTER: "));
                    Serial.println(crc_after, HEX);
                    Serial.print(F("CRC Errors: "));
                    Serial.print(sdcard_data.crc_errors);
                    Serial.print(F(" Write Errors: "));
                    Serial.println(sdcard_data.write_errors);
                    Serial.println(F("****************\n"));
                #endif // DEBUG
            }
        }
    }


        //print_sensors();
}


void loop() {


    // Read the sensors
    readSensors();

    // How's our battery?
    check_battery();

    // Control heat and power consumption
    heater_control();

    #ifdef DEBUG

        print_sensors();

    #endif // DEBUG

    log_data();
    //delay(1000);


}
