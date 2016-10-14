/* High Altitude Weather Balloon Controller
 * Controller to log sensor data to an SD card and allow location finding through
 * sound and lights.
 * Possible data transmission to a ground station through a radio transmitter.
 *
 * CS4360 Fall 2016
 * Licensed GPLv3
 * Robert Susmilch
*/

bool DEBUG = false;
bool DEBUG_MORE = false;
volatile uint16_t watchdog = 10;

#include <stdint.h>
#include <Wire.h>
//#include <SPI.h>
//#define USE_LCD
#include <util/atomic.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

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
#include "DS3231.h"
#include "PID_v1.h"
#include <EEPROM.h>
#include "EEPROM_anything.h"



// Definitions
void beep_piezo(unsigned long, unsigned long, unsigned int);
void configure_GPS_NMEA();
void configure_GPS_WAAS();
uint32_t find_sdcard_tail(const uint32_t, const uint32_t);
void flash_landing(uint16_t);

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

// One Wire data is plugged into pin 4 on the Arduino
#define ONE_WIRE_BUS 4

// UV sensor is on analog A0
#define UV_PIN  A0

// CO2 analog pin
#define CO2_PIN A3

// Piezo pin
#define PIEZO_PIN   A2

#define GREEN_LED     22
#define YELLOW_LED  23
#define RED_LED   24

// Remote Heaters
// Arduino board
// Daughter board (heater control, level shifting, sensor power cutoff) with battery
// Humidity
// UV
// IR, Visible
// GPS (though draws 90 ma at 3.3v...)
// 3x Possibly Geiger
//


// Resistive heater pin. Note that these are Arduino defined pin numbers.
#define MAIN_HEATER_PIN     45
#define GPS_HEATER_PIN      44
#define UV_HEATER_PIN       11
#define TSL2561_HEATER_PIN  46
#define CUSTOM_HEATER_PIN   12
#define SHT31_HEATER_PIN    5

#define GEIGER_PWR_PIN      39

// Resistive heater hystersis about setpoint. In whatever units temps
// are stored and processed (Celsius in this case.)
float hyster = 0.5;

// Number of degrees Celsius over ambient temps to prevent condensation.
float over_ambient = 2;

#define SAMPLE_TIME 1000

#ifdef USE_LCD
    // LCD stuff
    #define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
    #define COLS      20
    #define ROWS       4
    #define PAUSE    400
#endif

//  DS18B20 resolution / conversion times
//  9 =  93.75 ms
// 10 = 187.5  ms
// 11 = 375    ms
// 12 = 750    ms
//
#define DS18B20_RESOLUTION 11

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

    bool launched = false;
    bool alarm = false;
    // Temperature sensors
    // Box internal temp
    // External temp
    // Main
    // GPS
    // TSL2561
    // SHT31
    // UV
    // Custom daughter board (if any)
    struct {
        float main = -1000;
        float internal = -1000;
        float external = -1000;
        float gps = -1000;
        float tsl2561 = -1000;
        float sht31 = -1000;
        float uv = -1000;
        float ms5607 = -1000;
        float custom = -1000;
        float clock = -1000;

    } temp;

    struct {
        uint8_t main, gps, tsl2561, sht31, uv;
    } pwm;

    struct {
        // Humidity sensor
        float humidity = 0;
    } sht31;

    // UV light output
    float uv = 0;


    struct {
        // Battery voltage and state-of-charge
        float voltage, soc = 0;
    } battery;

    struct {
        // Pressure and temp of pressure sensor
        double pressure = 0;
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

    uint32_t unix_time;
    //uint16_t crc;

} data_struct;

// Create global sensor structure
data_struct dataLog;

struct {
        // Geiger counts for X, Y, Z axis.
        // When time to log data, copy data from count to axis, zero count.
        // Protects from losing counts while writing log (very slow.)
        // These are the values we will data log.

        // These are in use counters.
        volatile uint16_t x_count, y_count, z_count = 0;
} geiger;


// Eight DS18B20 sensor addresses, each address is 8 bytes long.
// The search order for this library is deterministic (same sensors
// will yield the same order.

// Box internal temp
// External temp
// Main
// GPS
// TSL2561
// SHT31
// UV
// Custom daughter board (if any)

struct {
    uint8_t INTERNAL_ADDR [8]   = {0x28, 0x70, 0x95, 0x22, 0x05, 0x00, 0x00, 0xA1};
    uint8_t TSL2561_ADDR [8]    = {0x28, 0x57, 0x8C, 0x22, 0x05, 0x00, 0x00, 0x1D};
    uint8_t UV_ADDR [8]         = {0x28, 0xFF, 0xB6, 0x50, 0x6A, 0x14, 0x03, 0xB9};
    uint8_t GPS_ADDR [8]        = {0x28, 0xFF, 0x4D, 0x7F, 0x6F, 0x14, 0x04, 0x33};
    uint8_t MAIN_ADDR [8]       = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t EXTERNAL_ADDR [8]   = {0x28, 0xFF, 0x8C, 0x54, 0x6F, 0x14, 0x04, 0x38};
    uint8_t CUSTOM_ADDR [8]     = {0x28, 0x1B, 0x46, 0x6E, 0x00, 0x00, 0x00, 0x4D};

    bool main = false;
    bool internal = false;
    bool external = false;
    bool gps = false;
    bool tsl2561 = false;
    bool uv = false;
    bool custom = false;
    uint8_t num_found = 0;

} temp_sensors;

struct {
    // Holds various configuration and memory settings.
    // User settings such as alarm thresholds, and weather the device has taken off are stored here.

    // Location alarms. These will be user settable through EEPROM in the future.
    // If under this pressure, we start the alarm for location retrieval. In Pascals
    float pressure_alarm = 82553;

    // If under this GPS altitude, we start the alarm for location retrieval. In meters.
    float altitude_alarm = 1800;

    // Did we take off from the ground already?
    bool launched = false;

    // Are we in alarm?
    bool alarm = false;

    bool decent = false;

    // Number of seconds above altitude threshold to "arm"
    uint16_t altitude_time = 120;

    // Number of minutes to assume balloon has launched or has landed.
    uint32_t launch_time = 120;
    uint32_t location_time = 240;

} configuration, default_config;



// Counter for above altitude to arm in-flight.
uint16_t count_at_altitude = 0;

// History of pressure and altitude to derive climbing or falling over time.
float pressure_hist[120];
float altitude_hist[120];

// Circular array queue head and tail indexes.
int16_t pressure_hist_head = 0;
int16_t pressure_hist_tail = -1;
int16_t altitude_hist_head = 0;
int16_t altitude_hist_tail = -1;


// Light sensor
//Adafruit_SI1145 uv = Adafruit_SI1145();

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

DS3231 clock;
RTCDateTime date_time;

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

// Create GPS object
// REQUIRES CUSTOM LARGER BUFFER FOR INCOMING UART

TinyGPSPlus gps;

// Grab vertical dilution of precision.
TinyGPSCustom vdop(gps, "GPGSA", 17);

struct {
    double temp;
    double pwm;
    double setpoint;

    // Set using the Ziegler-Nichols Method.
    double Kp = 176.47;
    double Ki = 7;
    double Kd = 1.75;
} gps_vPID, uv_vPID, tsl_vPID, sht_vPID, custom_vPID;

struct {
    double temp;
    double pwm;
    double setpoint;

    // Set using the Ziegler-Nichols Method.
    double Kp = 352.94; //600;
    double Ki = 30; //0;
    double Kd = 9; //0;
} main_vPID;

//PID_Vars main_vPID, gps_vPID, uv_vPID, tsl_vPID, sht_vPID, custom_vPID;

PID PID_main(&main_vPID.temp, &main_vPID.pwm, &main_vPID.setpoint, main_vPID.Kp, main_vPID.Ki, main_vPID.Kd, DIRECT);
PID PID_gps(&gps_vPID.temp, &gps_vPID.pwm, &gps_vPID.setpoint, gps_vPID.Kp, gps_vPID.Ki, gps_vPID.Kd, DIRECT);
PID PID_uv(&uv_vPID.temp, &uv_vPID.pwm, &uv_vPID.setpoint, uv_vPID.Kp, uv_vPID.Ki, uv_vPID.Kd, DIRECT);
PID PID_tsl(&tsl_vPID.temp, &tsl_vPID.pwm, &tsl_vPID.setpoint, tsl_vPID.Kp, tsl_vPID.Ki, tsl_vPID.Kd, DIRECT);
PID PID_sht(&sht_vPID.temp, &sht_vPID.pwm, &sht_vPID.setpoint, sht_vPID.Kp, sht_vPID.Ki, sht_vPID.Kd, DIRECT);
PID PID_custom(&custom_vPID.temp, &custom_vPID.pwm, &custom_vPID.setpoint, custom_vPID.Kp, custom_vPID.Ki, custom_vPID.Kd, DIRECT);

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

// Have we set the DS3231 RTC from GPS yet?
bool set_clock = false;

volatile bool reset_flag = false;

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
        // There was a problem detecting the ADXL345 ... check your connections
        Serial.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));

    } else {
        dataLog.found.TSL2561 = true;

        dataLog.enabled.TSL2561 = true;

        // Setup the sensor gain and integration time
        // You can also manually set the gain or enable auto-gain support
        // tsl.setGain(TSL2561_GAIN_1X);      // No gain ... use in bright light to avoid sensor saturation
        // tsl.setGain(TSL2561_GAIN_16X);     // 16x gain ... use in low light to boost sensitivity
        tsl.enableAutoRange(true);            // Auto-gain ... switches automatically between 1x and 16x

        // Changing the integration time gives you better sensor resolution (402ms = 16-bit data)
        //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      // fast but low resolution
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium resolution and speed
        // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  // 16-bit data but slowest conversions

    }

}

ISR(PCINT1_vect) {
    //Pin change interrupt for Geiger counters.
     //No falling or rising edge detection on these pins so we
     // need to track previous pin states.
     // When a pin has changed value we update the appropriate counter.
     // Geiger counter outputs a low pulse of 150 microseconds on particle detection.


    // Previous pin value for port. Static to preserve between calls.
    static uint8_t previous_pins_j = 0;

    // Grab current pin state quickly. Low pulse from Geiger is only 150 microseconds.
    uint8_t current_state_j = PINJ;

    // If the current pin state is low, and previous it was high, we have
    // a falling edge.
    if ((current_state_j & (1<<PJ1)) == 0 && (previous_pins_j & (1<<PJ1)) > 0) {
      // Trigger count
      geiger.y_count++;
    }

    if ((current_state_j & (1<<PJ0)) == 0 && (previous_pins_j & (1<<PJ0)) > 0) {
      // Trigger count
      geiger.x_count++;
    }

    // Store for later.
    previous_pins_j = current_state_j;
}

ISR(PCINT2_vect) {
    // Pin change interrupt for Geiger counters.
    //No falling or rising edge detection on these pins so we
    //need to track previous pin states.
    //When a pin has changed value we update the appropriate counter.
    //Geiger counter outputs a low pulse of 150 microseconds on particle detection.
    //

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

ISR(INT4_vect) {
    // Pin change interrupt for Geiger counters.
    //No falling or rising edge detection on these pins so we
    //need to track previous pin states.
    //When a pin has changed value we update the appropriate counter.
    //Geiger counter outputs a low pulse of 150 microseconds on particle detection.
    //

    // Previous pin value for port. Static to preserve between calls.
    static uint8_t previous_pins_e = 0;
    static bool button = false;
    static uint32_t button_time = 0;
    static uint8_t led_status;

    uint32_t current_time = millis();

    // Grab current pin state quickly. Low pulse from Geiger is only 150 microseconds.
    uint8_t current_state_e = PINE;

    // If the current pin state is high, and previous it was low, we have
    // a rising edge.
    if ((current_state_e & (1<<PE4)) >= 1 && (previous_pins_e & (1<<PE4)) == 0) {
        button = true;
        button_time = current_time;
        led_status = PINA & (1 << PA0) & (1 << PA1) & (1 << PA2);

        PORTA &= ~(1 << PA0) & ~(1 << PA1) & ~(1 << PA2);
    } else if ((current_state_e & (1 << PE4)) == 0 && ((previous_pins_e & (1 << PE4)) >= 1)) {
        PORTA |= (1 << PA0) | (1 << PA1) | (1 << PA2);

        if ((current_time - button_time) > 1000L && (current_time - button_time) < 5000L) {
            reset_flag = true;
            PORTA |= (1 << PA0) | (1 << PA1) | (1 << PA2);
        } else {
            reset_flag = false;
            PORTA |= (1 << PA0) | (1 << PA1) | (1 << PA2);
        }

    } else {
        PORTA |= (1 << PA0) | (1 << PA1) | (1 << PA2);
    }

    // Store for later.
    previous_pins_e = current_state_e;
}

ISR(WDT_vect) {
    // Run when the watchdog wakes from sleep.

}

void configure_button() {
    pinMode(2, INPUT_PULLUP);

    // Clear interrupts
    cli();

    EIMSK &= ~(1 << INT4);

    EICRB &= ~(1 << ISC41);
    EICRB |= (1 << ISC40);

    EIMSK |= (1 << INT4);


    // Set interrupts
    sei();

    // Now pin changes will call the ISR(PCINT0_vect) for button presses.
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

bool compare_buffers(uint8_t *first, uint8_t *second, uint16_t bytes) {
    // Compare two buffers for equality.

    // Return the default if we didn't short circuit on mismatch.

    for (uint16_t i = 0; i < bytes; i++) {
        if (first[i] != second[i]) {
            if (DEBUG_MORE == true) {
                Serial.print(F("Buffer mismatch at index: "));
                Serial.print(i);
                Serial.print(F(" with value 0x"));
                Serial.print(first[i], HEX);
                Serial.print(F(", 0x"));
                Serial.println(second[i]);
            }
            return false;
        }
    }
    return true;
}


void configure_ds18b20() {
    // Configure Dallas temp dataLog.

    uint8_t DS18B20_addr [8] = {0};

    DS18B20.begin();

    temp_sensors.num_found = DS18B20.getDeviceCount();

    if (temp_sensors.num_found > 0) {
        dataLog.found.DS18B20 = true;
        dataLog.enabled.DS18B20 = true;

        for (uint8_t i = 0; i < temp_sensors.num_found; i++) {
            DS18B20.getAddress(DS18B20_addr, i);

            if (DEBUG == true) {
                Serial.print(F(" DS18B20 device found: "));
                for (uint8_t j = 0; j < 8; j++) {
                    Serial.print(DS18B20_addr[j], HEX);
                    Serial.print(F(" "));
                }

                Serial.println();
            }

            // See if sensors match
            if (compare_buffers(DS18B20_addr, temp_sensors.MAIN_ADDR, 8) == true) {
                temp_sensors.main = true;

                if (DEBUG == true) {
                    Serial.println(F("Main Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.CUSTOM_ADDR, 8) == true) {
                temp_sensors.custom = true;

                if (DEBUG == true) {
                    Serial.println(F("Custom Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.GPS_ADDR, 8) == true) {
                temp_sensors.gps = true;

                if (DEBUG == true) {
                    Serial.println(F("GPS Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.TSL2561_ADDR, 8) == true) {
                temp_sensors.tsl2561 = true;

                if (DEBUG == true) {
                    Serial.println(F("TSL2561 Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.UV_ADDR, 8) == true) {
                temp_sensors.uv = true;

                if (DEBUG == true) {
                    Serial.println(F("UV Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.INTERNAL_ADDR, 8) == true) {
                temp_sensors.internal = true;

                if (DEBUG == true) {
                    Serial.println(F("Internal Temp Sensor Found"));
                }

            } else if (compare_buffers(DS18B20_addr, temp_sensors.EXTERNAL_ADDR, 8) == true) {
                temp_sensors.external = true;

                if (DEBUG == true) {
                    Serial.println(F("External Temp Sensor Found"));
                }

            }

        }
        // Set to global desired resolution.
        DS18B20.setResolution(DS18B20_RESOLUTION);

        // Don't wait for conversion
        DS18B20.setWaitForConversion(false);

        // Send the command to get temperatures so that the conversion complete flag
        // can be used in the main loop

        if (DEBUG == true) {
            Serial.println(F("Requesting DS18B20 all temps..."));
        }

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
        //sht31.heater(true);
    }
}

void print_found() {
    // Print a list of all sensors found
    Serial.println(F("\nSensors found"));
    Serial.println(F("-------------"));
    Serial.print(F("DS18B20: "));
    Serial.println(temp_sensors.num_found);
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
        if (configuration.launched == false) {
            digitalWrite(RED_LED, LOW);
        }
    } else {
        Serial.print(F("Card size: "));
        sdcard_data.blocks = sdcard.cardSize() - 1;
        Serial.print(sdcard_data.blocks);
        Serial.println(F(" blocks"));
    }

    uint32_t tail = find_sdcard_tail(1, sdcard_data.blocks);

    if (tail > 1) {
        sdcard_data.current_block = tail;
        Serial.print(F("\nFound tail of log at block: "));
        Serial.println(sdcard_data.current_block);
        if (configuration.launched == false) {
            digitalWrite(GREEN_LED, LOW);
        }
    } else {
        Serial.print(F("\nDid NOT find tail of log at block: "));
        Serial.println(sdcard_data.current_block);
        if (configuration.launched == false) {
            digitalWrite(RED_LED, LOW);
        }

    }

}

void configure_PID() {

    Serial.println(F("Configuring PID..."));
    //
    PID_main.SetMode(MANUAL);
    PID_gps.SetMode(MANUAL);
    PID_tsl.SetMode(MANUAL);
    PID_sht.SetMode(MANUAL);
    PID_custom.SetMode(MANUAL);
    PID_uv.SetMode(MANUAL);

    PID_main.SetSampleTime(SAMPLE_TIME);
    PID_gps.SetSampleTime(SAMPLE_TIME);
    PID_tsl.SetSampleTime(SAMPLE_TIME);
    PID_sht.SetSampleTime(SAMPLE_TIME);
    PID_custom.SetSampleTime(SAMPLE_TIME);
    PID_uv.SetSampleTime(SAMPLE_TIME);
}

void piezo_power(bool power) {
    if (power == true) {
        // PIEZO POWER
        DDRJ |= (1 << PJ7);
        PORTJ |= (1 << PJ7);
    } else {
        // PIEZO POWER
        DDRJ &= ~(1 << PJ7);
        PORTJ &= ~(1 << PJ7);
    }
}

void tsl2561_power(bool power) {
    if (power == true) {
        // TSL2561 as output
        DDRJ |= (1 << PJ5);
        // Turn on TSL2561
        PORTJ |= (1 << PJ5);
    } else {
        //Tristate
        DDRJ &= ~(1 << PJ5);
        PORTJ &= ~(1 << PJ5);
    }
}

void ms5607_power(bool power) {
    if (power == true) {
        // As output
        DDRJ |= (1 << PJ6);
        // Turn on
        PORTJ |= (1 << PJ6);
    } else {
        //Tristate
        DDRJ &= ~(1 << PJ6);
        PORTJ &= ~(1 << PJ6);
    }
}

void sht31_power(bool power) {
    if (power == true) {
        DDRJ |= (1 << PJ3);

        // SHT31 reset is inverse of power control. Shorting to ground resets sensor
        // but holds I2C hostage.
        PORTJ &= ~(1 << PJ3);
    } else {
        // Tri state
        DDRJ &= ~(1 << PJ3);
        PORTJ &= ~(1 << PJ3);
    }
}

void geiger_power(bool power) {

    if (power == true) {
        digitalWrite(GEIGER_PWR_PIN, HIGH);
    } else {
        // Tri state
        pinMode(GEIGER_PWR_PIN, INPUT);
    }
}

void sd_power(bool power) {
    if (power == true) {
        // SD Power
        DDRG |= (1 << PG1);
        PORTG |= (1 << PG1);
    } else {
        // Tristate
        DDRG &= ~(1 << PG1);
        PORTG &= ~(1 << PG1);
    }
}

void gps_power(bool power) {
    if (power == true) {
        // GPS
        DDRL |= (1 << PL7) ;
        PORTL |= (1 << PL7);
    } else {
        // Tristate
        DDRL &= ~(1 << PL7) ;
        PORTL &= ~(1 << PL7);
    }
}

void ds18b20_power(bool power) {
    if (power == true) {
        // DS18B20 Power
        DDRE |= (1 << PE2);
        PORTE |= (1 << PE2);
    } else {
        // Tri state
        DDRE &= ~(1 << PE2);
        PORTE &= ~(1 << PE2);
    }
}

void dof_power(bool power) {
    if (power == true) {
        // 9DOF
        DDRA |= (1 << PA7);
        PORTA |= (1 << PA7);
    } else {
        // Tristate
        DDRA &= ~(1 << PA7);
        PORTA &= ~(1 << PA7);
    }
}

void led_power(bool power) {
    if (power == true) {
        pinMode(RED_LED, OUTPUT);
        pinMode(YELLOW_LED, OUTPUT);
        pinMode(GREEN_LED, OUTPUT);
        digitalWrite(RED_LED, HIGH);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    } else {
        pinMode(RED_LED, INPUT);
        pinMode(YELLOW_LED, INPUT);
        pinMode(GREEN_LED, INPUT);
    }
}


void power_sensors(bool power) {

    geiger_power(power);
    ms5607_power(power);
    tsl2561_power(power);
    piezo_power(power);
    dof_power(power);
    sht31_power(power);
    sd_power(power);
    gps_power(power);
    ds18b20_power(power);
    led_power(power);

    if (power == true) {
        pinMode(MAIN_HEATER_PIN, OUTPUT);
        pinMode(UV_HEATER_PIN, OUTPUT);
        pinMode(TSL2561_HEATER_PIN, OUTPUT);
        pinMode(SHT31_HEATER_PIN, OUTPUT);
        pinMode(GEIGER_PWR_PIN, OUTPUT);
        pinMode(GPS_HEATER_PIN, OUTPUT);

    } else {
        pinMode(MAIN_HEATER_PIN, INPUT);
        pinMode(UV_HEATER_PIN, INPUT);
        pinMode(TSL2561_HEATER_PIN, INPUT);
        pinMode(SHT31_HEATER_PIN, INPUT);
        pinMode(GEIGER_PWR_PIN, INPUT);
        pinMode(GPS_HEATER_PIN, INPUT);


    }
}



void watchdog_sleep() {
  // Setup the WDT for sleep waking
  cli();

  // Clear the reset flag.
  MCUSR &= ~(1<<WDRF);

  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).

  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout prescaler value
  //WDTCSR = 1<<WDP0 | 1<<WDP3;                       // 8.0 seconds
  //WDTCSR = (1 << WDP2) | (1 << WDP1);               // 1 Second
  //WDTCSR = (1 << WDP2) | (1 << WDP1) | (1 << WDP0); // 2 seconds;
  // Include the interrupt and then reset.
  WDTCSR = (1 <<WDP3) | (1 << WDIE) | (1 << WDE);     // 4 seconds;

  sei();
}

void watchdog_reset_only() {
  // Setup the WDT for sleep waking
  cli();

  // Clear the reset flag.
  MCUSR &= ~(1<<WDRF);

  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).

  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout prescaler value
  //WDTCSR = 1<<WDP0 | 1<<WDP3;                       // 8.0 seconds
  //WDTCSR = (1 << WDP2) | (1 << WDP1);               // 1 Second
  //WDTCSR = (1 << WDP2) | (1 << WDP1) | (1 << WDP0); // 2 seconds;
  WDTCSR = (1 <<WDP3) | (1 << WDE);                   // 4 seconds;
  // Enable the WD interrupt (note no reset).
  //WDTCSR |= _BV(WDIE);

  sei();
}

void disable_peripherals () {
    // Here we've entered alarm mode, or assumed location mode.
    // Disable all peripherals to save power.

    power_adc_disable();
    power_spi_disable();
    power_twi_disable();
    power_usart0_disable();
    power_usart1_disable();
    power_usart2_disable();
    //power_usart3_disable();
}

void enter_sleep(void) {
  // EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption.
  wdt_reset();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //sleep_bod_disable();
  sei();
  // Now enter sleep mode.
  sleep_mode();

  // The program will continue from here after the WDT timeout
  sleep_disable(); // First thing to do is disable sleep.

  // Re-enable the peripherals.
  //power_all_enable();
}
void location_alarm() {
    disable_peripherals();
    watchdog_sleep();

    while(true) {
        enter_sleep();
        WDTCSR |= (1 << WDIE);

        flash_landing(10);
        piezo_power(true);
        //Serial.begin(115200);
        beep_piezo(1000L, 250L, PIEZO_PIN);
        beep_piezo(2000L, 250L, PIEZO_PIN);
        piezo_power(false);
    }
}

void count_down_led() {
    wdt_reset();
    led_power(true);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    wdt_reset();
    delay(3333);

    digitalWrite(GREEN_LED, HIGH);
    wdt_reset();
    delay(3333);

    digitalWrite(YELLOW_LED, HIGH);
    wdt_reset();
    delay(3333);
    digitalWrite(RED_LED, HIGH);

    wdt_reset();
    led_power(false);
}

void setup() {

    wdt_reset();
    watchdog_reset_only();

    configure_button();

    power_sensors(false);

    EEPROM.get(0, configuration);
/*
    Serial.begin(115200);

    Serial.print(F("Launched: "));
    Serial.print(configuration.launched);
    Serial.print(F(" Alarm: "));
    Serial.println(configuration.alarm);
*/
    if (configuration.alarm == true || configuration.launched == true) {
        count_down_led();
    }

    if (reset_flag == true) {
        wdt_reset();
        EEPROM.put(0, default_config);
        reset_flag = false;
        flash_landing(10);
        piezo_power(true);
        beep_piezo(1000, 250, PIEZO_PIN);
        delay(100);
        flash_landing(10);
        beep_piezo(2000, 250, PIEZO_PIN);
        piezo_power(false);
    }

    wdt_reset();
    EEPROM.get(0, configuration);
/*
    Serial.begin(115200);

    Serial.print(F("Launched: "));
    Serial.print(configuration.launched);
    Serial.print(F(" Alarm: "));
    Serial.println(configuration.alarm);

    delay(1000);
    */
    if (configuration.alarm == false) {
        Heat_Enable = true;

        configure_PID();

        // LEDs
        led_power(true);

        //delay(1000);

        //digitalWrite(RED_LED, HIGH);

        // Sensor power and heater pin modes.
        power_sensors(true);


        Serial.begin(115200);
        Serial.println(F("\n\rRestarting HAB Controller"));

        clock.begin();

        beep_piezo(1000, 250, PIEZO_PIN);



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

        wdt_reset();
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

        wdt_reset();

        if (DEBUG == true) {
            print_found();
        }

        //Heat_Enable = true;

        // First analog reading can be inaccurate due to ADC capacitor needing to be primed, so start it up.
        analogRead(UV_PIN);
    } else {
        watchdog_sleep();
        power_sensors(false);
        location_alarm();
    }
    // FOR TESTING BATTERY RUNTIME ONLY.
    /*
    analogWrite(MAIN_HEATER_PIN, 150);
    analogWrite(TSL2561_HEATER_PIN, 150);
    analogWrite(UV_HEATER_PIN, 150);
    analogWrite(SHT31_HEATER_PIN, 150);
    analogWrite(GPS_HEATER_PIN, 150);
    */

    if (watchdog == 10) {
        watchdog = 20;
    }
}

void flash_landing(uint16_t time) {
    // Flash location landing lights for time ms.
    DDRF |= (1 << PF1);
    PORTF |= (1 << PF1);
    delay(time);
    DDRF &= ~(1 << PF1);
    PORTF &= ~(1 << PF1);

}

void beep_piezo(unsigned long freq, unsigned long millisecs, unsigned int pin) {
    // Buzzer harmonics seem to be around 2000 Hz for max volume. 3500-4000 also work.
    // How long to go high or low for frequency
    unsigned long squareWave_us = (1000000L / (2 * freq));
    // How many loops to beep for?
    unsigned long duration_loops = (unsigned long)(freq * millisecs / 1000L);
/*
    Serial.print(F("Freq: "));
    Serial.print(freq);
    Serial.print(F(" Square: "));
    Serial.print(squareWave_us);
    Serial.print(F(" Duration: "));
    Serial.println(duration_loops);
*/
    for (unsigned long i=0; i < duration_loops; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(squareWave_us);
        digitalWrite(pin, LOW);
        delayMicroseconds(squareWave_us);
    }


}



void read_GPS() {
    // TODO: Add check if time chip has been set from GPS values.
     // Better accuracy from reliable GPS settings.
     //
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

        if (configuration.launched == false) {
            digitalWrite(YELLOW_LED, LOW);
        }
    } else {
        dataLog.gps.lat = 0;
        dataLog.gps.lng = 0;
        if (configuration.launched == false) {
            digitalWrite(YELLOW_LED, HIGH);
        }
    }

    if (gps.altitude.isValid() && gps.altitude.age() < 10000L) {
        dataLog.gps.altitude = gps.altitude.meters();
    } else {
        dataLog.gps.altitude = 0;
    }

    if (gps.date.isValid() && gps.date.age() < 60000L) {
        dataLog.gps.date = gps.date.value();
    } else {
        dataLog.gps.date = 0;
    }

    if (DEBUG == true) {
        Serial.print(F("Date age: "));
        Serial.print(gps.date.age());
    }

    if (gps.time.isValid() && gps.time.age() < 5000L) {
        dataLog.gps.time = gps.time.value();

        // Need to set the RTC. This will always set the clock when a valid GPS time is first found.
        // This protects against a set clock with backup battery having clock drift.
        // Also synchronizes the clocks.
        if (set_clock == false) {
            clock.setDateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
            set_clock = true;
        }

    } else {
        dataLog.gps.time = 0;
    }

    if (DEBUG == true) {
        Serial.print(F(" time: "));
        Serial.println(gps.time.age());
    }

    if (gps.satellites.isValid()) {
        dataLog.gps.sats = static_cast<uint8_t>(gps.satellites.value());
    } else {
        dataLog.gps.sats = 0;
    }

    if (gps.hdop.isValid()) {
        dataLog.gps.hdop = static_cast<uint16_t>(gps.hdop.value());// * GPS_PRECISION);
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
        dataLog.gps.vdop = static_cast<uint16_t>(strtod(vdop.value(), NULL));// * 100 * GPS_PRECISION);
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
    if (watchdog == 30) {
        watchdog = 40;
    }

    uint16_t ir, broadband = 0;

    // Get the time from the DS3231 RTC
    date_time = clock.getDateTime();

    dataLog.unix_time = date_time.unixtime;

    // The temperature registers are updated after every 64-second conversion.
    // If you want force temperature conversion use forceConversion()
    clock.forceConversion();
    dataLog.temp.main = clock.readTemperature();

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
        dataLog.temp.sht31 = sht31.readTemperature();
        dataLog.sht31.humidity = sht31.readHumidity();
    } else {
        dataLog.temp.sht31 = -1000;
        dataLog.sht31.humidity = -1000;
    }


    if (dataLog.found.MS5607 == true && ms5607.Read_CRC4() == ms5607.Calc_CRC4()) {
        dataLog.ms5607.pressure = ms5607.GetPres();
        dataLog.temp.ms5607 = ms5607.GetTemp();

        // Read the altitude sensor values into object for next round?
        ms5607.Readout();

    } else {
        dataLog.ms5607.pressure = 0;
        dataLog.temp.ms5607 = -1000;
    }

    // Check if temp is available
    if (dataLog.found.DS18B20 == true) {
        /*
        if (temp_sensors.main == true && DS18B20.isConversionAvailable(temp_sensors.MAIN_ADDR)) {
            dataLog.temp.main = DS18B20.getTempC(temp_sensors.MAIN_ADDR);
            if (DEBUG_MORE == true) {
                Serial.print(F("Main: "));
                Serial.println(dataLog.temp.main);
            }
        } else {
            dataLog.temp.main = -1000;
        }
        */

        if (temp_sensors.internal == true && DS18B20.isConversionAvailable(temp_sensors.INTERNAL_ADDR)) {
            dataLog.temp.internal = DS18B20.getTempC(temp_sensors.INTERNAL_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.INTERNAL_ADDR);
            if (DEBUG_MORE == true) {
                Serial.print(F("Internal: "));
                Serial.println(dataLog.temp.internal);
            }
        } else {
            dataLog.temp.internal = -1000;
        }

        if (temp_sensors.external == true && DS18B20.isConversionAvailable(temp_sensors.EXTERNAL_ADDR)) {
            dataLog.temp.external = DS18B20.getTempC(temp_sensors.EXTERNAL_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.EXTERNAL_ADDR);
            if (DEBUG_MORE == true) {
                Serial.print(F("External: "));
                Serial.println(dataLog.temp.external);
            }
        } else {
            dataLog.temp.external = -1000;
        }

        if (temp_sensors.gps == true && DS18B20.isConversionAvailable(temp_sensors.GPS_ADDR)) {
            dataLog.temp.gps = DS18B20.getTempC(temp_sensors.GPS_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.GPS_ADDR);
        } else {
            dataLog.temp.gps = -1000;
        }

        if (temp_sensors.tsl2561 == true && DS18B20.isConversionAvailable(temp_sensors.TSL2561_ADDR)) {
            dataLog.temp.tsl2561 = DS18B20.getTempC(temp_sensors.TSL2561_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.TSL2561_ADDR);
        } else {
            dataLog.temp.tsl2561 = -1000;
        }

        if (temp_sensors.uv == true && DS18B20.isConversionAvailable(temp_sensors.UV_ADDR)) {
            dataLog.temp.uv = DS18B20.getTempC(temp_sensors.UV_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.UV_ADDR);
        } else {
            dataLog.temp.uv = -1000;
        }

        if (temp_sensors.custom == true && DS18B20.isConversionAvailable(temp_sensors.CUSTOM_ADDR)) {
            dataLog.temp.custom = DS18B20.getTempC(temp_sensors.CUSTOM_ADDR);
            //DS18B20.requestTemperaturesByAddress(temp_sensors.CUSTOM_ADDR);
        } else {
            dataLog.temp.custom = -1000;
        }

        // Send the command to get temperatures for next time.
        DS18B20.requestTemperatures();

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

    if (watchdog == 40) {
        watchdog = 50;
    }
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
    if (watchdog == 140) {
        watchdog = 150;
    }
    // Check battery capacity and do appropriate things.

    if (dataLog.battery.voltage <= 3.2 || dataLog.battery.soc < 10) {
        // Turn off CO2 sensor as well.
        Heat_Enable = false;


    } else if (dataLog.battery.voltage > 3.4 && dataLog.battery.soc > 10) {
        Heat_Enable = true;
    }

    if (watchdog == 150) {
        watchdog = 160;
    }
}

void heater_control() {
    if (watchdog == 160) {
        watchdog = 170;
    }
    // Define the setpoint for whatever the external temperature is,
    // plus some for dew point.
    //float setpoint = 40;

    float setpoint = -1000;

    if (dataLog.temp.external != -1000) {
        setpoint = dataLog.temp.external + 2;


    }

    // Check for minimum operating temps for electronic components.
    if (setpoint < -40 + over_ambient) {
        setpoint = -40 + over_ambient;
    }


    if (DEBUG == true) {
        Serial.print(F("Temp setpoint: "));
        Serial.print(setpoint * 1.8 + 32);
        Serial.print(F("F Hystersis: "));
        Serial.print(hyster * 1.8);
        Serial.println(F("F"));
    }

    if (Heat_Enable == false) {
        // Turn off heaters
        PID_main.SetMode(MANUAL);
        PID_gps.SetMode(MANUAL);
        PID_tsl.SetMode(MANUAL);
        PID_sht.SetMode(MANUAL);
        PID_custom.SetMode(MANUAL);
        PID_uv.SetMode(MANUAL);

        digitalWrite(MAIN_HEATER_PIN, LOW);
        digitalWrite(CUSTOM_HEATER_PIN, LOW);
        digitalWrite(GPS_HEATER_PIN, LOW);
        digitalWrite(TSL2561_HEATER_PIN, LOW);
        digitalWrite(UV_HEATER_PIN, LOW);
        digitalWrite(SHT31_HEATER_PIN, LOW);



    } else {
        PID_main.SetMode(AUTOMATIC);
        PID_gps.SetMode(AUTOMATIC);
        PID_tsl.SetMode(AUTOMATIC);
        PID_sht.SetMode(AUTOMATIC);
        PID_custom.SetMode(AUTOMATIC);
        PID_uv.SetMode(AUTOMATIC);
        // Check setpoints

        // Check if sensor was found, and valid reading.

        if (temp_sensors.gps == true && dataLog.temp.gps != -1000) {
            // Found. Update setpoint
            gps_vPID.setpoint = setpoint;
            // Save temp to PID
            gps_vPID.temp = dataLog.temp.gps;
            // Compute PID
            PID_gps.Compute();
            // Set output.
            analogWrite(GPS_HEATER_PIN, gps_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_gps.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(GPS_HEATER_PIN, 0);
        }


         // Check if sensor was found, and valid reading.
        if (temp_sensors.uv == true && dataLog.temp.uv != -1000) {
            // Found. Update setpoint
            uv_vPID.setpoint = setpoint;
            // Save temp to PID
            uv_vPID.temp = dataLog.temp.uv;
            // Compute PID
            PID_uv.Compute();
            // Set output.
            analogWrite(UV_HEATER_PIN, uv_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_uv.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(UV_HEATER_PIN, 0);
        }

        // Check if sensor was found, and valid reading.
        if (temp_sensors.tsl2561 == true && dataLog.temp.tsl2561 != -1000) {
            // Found. Update setpoint
            tsl_vPID.setpoint = setpoint;
            // Save temp to PID
            tsl_vPID.temp = dataLog.temp.tsl2561;
            // Compute PID
            PID_tsl.Compute();
            // Set output.
            analogWrite(TSL2561_HEATER_PIN, tsl_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_tsl.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(TSL2561_HEATER_PIN, 0);
        }

        // Check if sensor was found, and valid reading.
        if (dataLog.temp.main != -1000) {
            // Found. Update setpoint
            main_vPID.setpoint = setpoint;
            // Save temp to PID
            main_vPID.temp = dataLog.temp.main;
            // Compute PID
            PID_main.Compute();
            // Set output.
            analogWrite(MAIN_HEATER_PIN, main_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_main.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(MAIN_HEATER_PIN, 0);
        }

        // Check if sensor was found, and valid reading.
        if (temp_sensors.custom == true && dataLog.temp.custom != -1000) {
            // Found. Update setpoint
            custom_vPID.setpoint = setpoint;
            // Save temp to PID
            custom_vPID.temp = dataLog.temp.custom;
            // Compute PID
            PID_custom.Compute();
            // Set output.
            analogWrite(CUSTOM_HEATER_PIN, custom_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_custom.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(CUSTOM_HEATER_PIN, 0);
        }

        // Check if sensor was found, and valid reading.
        if (dataLog.found.SHT31 == true && dataLog.temp.sht31 != -1000) {
            // Found. Update setpoint
            sht_vPID.setpoint = setpoint;
            // Save temp to PID
            sht_vPID.temp = dataLog.temp.sht31;
            // Compute PID
            PID_sht.Compute();
            // Set output.
            analogWrite(SHT31_HEATER_PIN, sht_vPID.pwm);
        } else {
            // No sensor found, or faulty reading.
            PID_sht.SetMode(MANUAL);
            // Turn off heater.
            analogWrite(SHT31_HEATER_PIN, 0);
        }

        /*

        if (dataLog.temp.custom < (setpoint - hyster) && dataLog.temp.custom != -1000) {
            analogWrite(CUSTOM_HEATER_PIN, dataLog.heater_max);
        } else if (dataLog.temp.custom > (setpoint + hyster)) {
            digitalWrite(CUSTOM_HEATER_PIN, LOW);
        }

        if (dataLog.temp.main < (setpoint - hyster) && dataLog.temp.main != -1000) {
            digitalWrite(MAIN_HEATER_PIN, HIGH);
        } else if (dataLog.temp.main > (setpoint + hyster)) {
            digitalWrite(MAIN_HEATER_PIN, LOW);
        }

        if (dataLog.temp.uv < (setpoint - hyster) && dataLog.temp.uv != -1000) {
            digitalWrite(UV_HEATER_PIN, HIGH);
        } else if (dataLog.temp.uv > (setpoint + hyster)) {
            digitalWrite(UV_HEATER_PIN, LOW);
        }

        if (dataLog.temp.tsl2561 < (setpoint - hyster) && dataLog.temp.tsl2561 != -1000) {
            digitalWrite(TSL2561_HEATER_PIN, HIGH);
        } else if (dataLog.temp.tsl2561 > (setpoint + hyster)) {
            digitalWrite(TSL2561_HEATER_PIN, LOW);
        }

        if (dataLog.temp.gps < (setpoint - hyster) && dataLog.temp.gps != -1000) {
            digitalWrite(GPS_HEATER_PIN, HIGH);
        } else if (dataLog.temp.gps > (setpoint + hyster)) {
            digitalWrite(GPS_HEATER_PIN, LOW);
        }

        if (dataLog.temp.sht31 < (setpoint - hyster) && dataLog.temp.sht31 != -1000) {
            digitalWrite(SHT31_HEATER_PIN, HIGH);
        } else if (dataLog.temp.sht31 > (setpoint + hyster)) {
            digitalWrite(SHT31_HEATER_PIN, LOW);
        }
        */

        if (DEBUG == true) {
            Serial.print(F("PWM -- Main: "));
            Serial.print(main_vPID.pwm);
            Serial.print(F(" GPS: "));
            Serial.print(gps_vPID.pwm);
            Serial.print(F(" SHT: "));
            Serial.print(sht_vPID.pwm);
            Serial.print(F(" TSL: "));
            Serial.print(tsl_vPID.pwm);
            Serial.print(F(" UV: "));
            Serial.print(uv_vPID.pwm);
            Serial.print(F(" CUSTOM: "));
            Serial.println(custom_vPID.pwm);
        }
    }
    //Serial.println(dataLog.temp.main);
}


void print_sensors() {

        Serial.print(date_time.year);   Serial.print("-");
        Serial.print(date_time.month);  Serial.print("-");
        Serial.print(date_time.day);    Serial.print(" ");
        Serial.print(date_time.hour);   Serial.print(":");
        Serial.print(date_time.minute); Serial.print(":");
        Serial.print(date_time.second); Serial.print("  ");
        Serial.println(date_time.unixtime);

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

        if (dataLog.temp.sht31 != 1000) {  // check if 'is not a number'
            Serial.print(dataLog.temp.sht31 * 1.8 + 32);
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
        Serial.print((dataLog.temp.ms5607 * 0.018) + 32);
        Serial.println(F(" F "));


        Serial.print(F("DS18B20\n\rMain: "));
        Serial.print(dataLog.temp.main * 1.8 + 32);
        Serial.print(F(", In: "));
        Serial.print(dataLog.temp.internal * 1.8 + 32);
        Serial.print(F(", Ex: "));
        Serial.print(dataLog.temp.external * 1.8 + 32);
        Serial.print(F(", GPS: "));
        Serial.print(dataLog.temp.gps * 1.8 + 32);
        Serial.print(F(", TSL: "));
        Serial.print(dataLog.temp.tsl2561 * 1.8 + 32);
        Serial.print(F(", UV: "));
        Serial.print(dataLog.temp.uv * 1.8 + 32);
        Serial.print(F(", CUST: "));
        Serial.print(dataLog.temp.custom * 1.8 + 32);
        Serial.println(F(", "));

        Serial.print(F("Clock temp: "));
        Serial.println(dataLog.temp.clock * 1.8 + 32);

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
        Serial.print(dataLog.geiger.x);
        Serial.print(F("/"));
        Serial.print(dataLog.geiger.y);
        Serial.print(F("/"));
        Serial.println(dataLog.geiger.z);

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
    // Wait for completion of previous write
    while(EECR & (1<<EEPE)) {
    }

    // Set up address register
    EEAR = uiAddress;
    // Start eeprom read by writing EERE
    EECR |= (1<<EERE);
    // Return data from Data Register
    return EEDR;
}


void EEPROM_update(uint16_t uiAddress, uint8_t ucData) {
    // Write data to EEPROM unless the data is already the same at the address.

    uint8_t readEEPROM = EEPROM_read(uiAddress);

    if (readEEPROM == ucData) {
        // We already have that value written.
        return;
    }

    // Wait for completion of previous write
    while(EECR & (1<<EEPE)) {
    }

    // Set up address and Data Registers
    EEAR = uiAddress;
    EEDR = ucData;
    // Write logical one to EEMPE
    EECR |= (1<<EEMPE);
    // Start eeprom write by setting EEPE
    EECR |= (1<<EEPE);
}

void read_configuration() {
    // Read the configuration data from EEPROM.

    // How big is our buffer?
    uint16_t buffer_size = sizeof(configuration);
    uint8_t buffer[buffer_size];
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

        if (DEBUG == true) {

            Serial.print(F("Left: "));
            Serial.print(left);
            Serial.print(F(" Current: "));
            Serial.print(current_search);
            Serial.print(F(" Right: "));
            Serial.println(right);

        }

        // Read a partial block for data
        sdcard.readData(current_search, 0, data_size, sdcard_data.buffer);

        for (uint8_t i = 0; i < data_size; i++) {
            if (sdcard_data.buffer[i] != 0) {
                // Block is not empty.
                not_found = true;

                // New left value is this current block
                left = current_search;

                left_empty = false;
                if (DEBUG_MORE == true) {
                    sdcard_data.current_block++;
                    Serial.print(F("SDcard block "));
                    Serial.print(sdcard_data.current_block);
                    Serial.println(F(" NOT empty... skipping."));
                }
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

    Serial.print(F("\n\n\n\r*************************\n\rRead sensors from SDcard\n\r*************************\n\n\n\r"));
    for (uint8_t i = 0; i < 510/data_size; i++) {
        memcpy(&dataLog, &sdcard_data.buffer[i * data_size], data_size);

        print_sensors();
    }

    Serial.print(F("\n\n\n\r*************************\n\rEnd sensors from SDcard\n\r*************************\n\n\n\r"));

}


void log_data() {

    if (watchdog == 120) {
        watchdog = 130;
    }

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

        // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
        _MemoryBarrier();

        // Stop all interrupts. Prevents the Geiger interrupt from increasing our variables while we copy and zero them for logging.
        cli();

        // Copy over Geiger counts for logging

        dataLog.geiger.x = geiger.x_count;
        dataLog.geiger.y = geiger.y_count;
        dataLog.geiger.z = geiger.z_count;

        // Zero interrupt counts
        geiger.x_count = 0;
        geiger.y_count = 0;
        geiger.z_count = 0;

        // Allow interrupts to continue.
        sei();
        _MemoryBarrier();

        // Copy the data log structure as a byte stream to the SD card buffer for later writing.
        memcpy(&sdcard_data.hold_buffer[sdcard_data.num_packed * data_size], &dataLog, data_size);

        // We packed another
        sdcard_data.num_packed++;

        if (DEBUG == true) {
            Serial.print(F("Number of structures packed: "));
            Serial.println(sdcard_data.num_packed);
            Serial.print(F("SD card packing start address: "));
            Serial.println(sdcard_data.num_packed * data_size, DEC);
        }

    }

    if (sdcard_data.num_packed * data_size >= 510 - data_size) {
        // Test again to see if we're at the limit. If we are, we need to log it.
        // Otherwise we lose a log when we are called again.
        // We allow 2 bytes for the CRC16 at the end of the buffer.

        if (DEBUG == true) {
            Serial.print(F("\nSensor data size: "));
            Serial.println(data_size);

            Serial.print(F("SD Block: "));
            Serial.print(sdcard_data.current_block);
            Serial.print(F("/"));
            Serial.println(sdcard_data.blocks);
        }

        //print_sensors();

        // We may want to try multiple data writes in case of a bad block.
        // Need to define how large a bad block flash memory can have.
        // In other words, if a write/erase block is 8MB, then multiple 512 byte
        // blocks will need to be skipped to get to "good" memory.
        // 8MB = 16384 pages of 512 bytes (we're writing to 512 byte blocks, so
        // so that's what we care about. Add 1 to get over any threshold.

        // Additionally, one may, or may not, want to use a watchdog timer reset
        // within the code to find an empty block. Thjs may also depend on whether
        // the controller is feeding radio telemetry. One would not want to get stuck
        // looking for an open location. This may be a moot point with a properly zeroed
        // card, but worth considering.
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

                    if (DEBUG == true) {
                        sdcard_data.current_block++;
                        Serial.print(F("SDcard block "));
                        Serial.print(sdcard_data.current_block);
                        Serial.println(F(" NOT empty... skipping."));
                    }

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

                // Copy over the CRC we read...
                // This may not do anything since we set the CRC variable in the next line (over writing it)
                memcpy(&crc_after, &sdcard_data.buffer[511 - sizeof(crc_before)], sizeof(crc_before));

                // Calculate the CRC after we read back the data.
                crc_after = sd_calc_crc(sdcard_data.buffer, data_size * (sdcard_data.num_packed + 1));

                if (crc_before != crc_after) {
                    // Test if the write and read were identical
                    // They were not!
                    sdcard_data.crc_errors++;

                    // We have a problem, crc mismatch.
                    Serial.println(F("\n\r************************\n\r*     CRC MISMATCH     *\n\r************************"));

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

                    if (DEBUG_MORE == true) {
                        test_logging();
                    }
                }

                if (DEBUG == true) {
                    Serial.print(F("****************\n\rCRC BEFORE: "));
                    Serial.println(crc_before, HEX);

                    Serial.print(F(" CRC AFTER: "));
                    Serial.println(crc_after, HEX);
                    Serial.print(F("CRC Errors: "));
                    Serial.print(sdcard_data.crc_errors);
                    Serial.print(F(" Write Errors: "));
                    Serial.println(sdcard_data.write_errors);
                    Serial.println(F("****************\n"));
                }
            }
        }
    }

    if (watchdog == 130) {
        watchdog = 140;
    }
        //print_sensors();

}


void read_sdcard() {
    // Reads data off the SD card and spits it out to the serial console as a comma separated value stream.

    // Get size of the data structure
    static uint16_t data_size = sizeof(dataLog);

    // Save our position.
    uint32_t saved_current_block = sdcard_data.current_block;

    bool empty_block, crc_match;

    uint16_t crc_read, crc_calculated;

    // Start at the beginning
    //sdcard_data.current_block = 1;

    // Print column headers
    Serial.println(F("***** DATA START *****"));
    Serial.println(F("unix_time,gps_date,gps_time,latitude,longitude,altitude,speed,course,sats,hdop,vdop,lux,broadband,ir,uv,pressure,humidity,accel_x,accel_y,accel_z, mag_x,mag_y,mag_z,gyro_x,gyro_y,gyro_z,temp_main,temp_internal,temp_external,temp_gps,temp_tsl2561,temp_sht31,temp_uv,temp_ms5607,temp_custom,temp_clock,geiger_x,geiger_y,geiger_z,pwm_main,pwm_gps,pwm_tsl2561,pwm_sht31,pwm_uv,max_pwm,voltage,state_of_charge,launched,alarm,found_lsm9ds1,found_sht31,found_ms5607,found_uv,found_ds18b20,found_tsl2561,found_gps,crc_match"));

    for (uint32_t block = 1; block <= sdcard_data.blocks; block++) {
        // Reset the watchdog, we're going to be here a while.
        wdt_reset();

        // Zero the buffer
        memset(sdcard_data.buffer, 0, 512);

        sdcard.readBlock(block, sdcard_data.buffer);

        empty_block = true;

        //Serial.println(F("Verify zero"));

        for (uint16_t i = 0; i < 512; i++) {
            // Iterate over the block, looking for non-zero values

            //Serial.println(sdcard_data.buffer[i], HEX);
            if (sdcard_data.buffer[i] != 0) {
                // Not empty
                empty_block = false;
                //success = false;

                // Break for next block.
                break;
            }
        }

        if (empty_block == false) {
            // Skip all empty blocks. (Only process blocks with any bytes in them.

            // Copy over the CRC we read...
            memcpy(&crc_read, &sdcard_data.buffer[511 - sizeof(crc_read)], sizeof(crc_read));

            // Calculate the CRC after we read back the data.
            crc_calculated = sd_calc_crc(sdcard_data.buffer, data_size * (sdcard_data.num_packed + 1));


            if (crc_read == crc_calculated) {
                crc_match = true;
            } else {
                crc_match = false;
            }

            for (uint8_t i = 0; i < 510/data_size; i++) {
                memcpy(&dataLog, &sdcard_data.buffer[i * data_size], data_size);

                Serial.print(dataLog.unix_time);
                Serial.print(F(","));
                Serial.print(dataLog.gps.date);
                Serial.print(F(","));
                Serial.print(dataLog.gps.time);
                Serial.print(F(","));
                Serial.print(dataLog.gps.lat);
                Serial.print(F(","));
                Serial.print(dataLog.gps.lng);
                Serial.print(F(","));
                Serial.print(dataLog.gps.altitude);
                Serial.print(F(","));
                Serial.print(dataLog.gps.speed);
                Serial.print(F(","));
                Serial.print(dataLog.gps.course);
                Serial.print(F(","));
                Serial.print(dataLog.gps.sats);
                Serial.print(F(","));
                Serial.print(dataLog.gps.hdop);
                Serial.print(F(","));
                Serial.print(dataLog.gps.vdop);
                Serial.print(F(","));
                Serial.print(dataLog.tsl.lux);
                Serial.print(F(","));
                Serial.print(dataLog.tsl.broadband);
                Serial.print(F(","));
                Serial.print(dataLog.tsl.ir);
                Serial.print(F(","));
                Serial.print(dataLog.uv);
                Serial.print(F(","));
                Serial.print(dataLog.ms5607.pressure);
                Serial.print(F(","));
                Serial.print(dataLog.sht31.humidity);
                Serial.print(F(","));
                Serial.print(dataLog.accel.x);
                Serial.print(F(","));
                Serial.print(dataLog.accel.y);
                Serial.print(F(","));
                Serial.print(dataLog.accel.z);
                Serial.print(F(","));
                Serial.print(dataLog.mag.x);
                Serial.print(F(","));
                Serial.print(dataLog.mag.y);
                Serial.print(F(","));
                Serial.print(dataLog.mag.z);
                Serial.print(F(","));
                Serial.print(dataLog.gyro.x);
                Serial.print(F(","));
                Serial.print(dataLog.gyro.y);
                Serial.print(F(","));
                Serial.print(dataLog.gyro.z);
                Serial.print(F(","));
                Serial.print(dataLog.temp.main);
                Serial.print(F(","));
                Serial.print(dataLog.temp.internal);
                Serial.print(F(","));
                Serial.print(dataLog.temp.external);
                Serial.print(F(","));
                Serial.print(dataLog.temp.gps);
                Serial.print(F(","));
                Serial.print(dataLog.temp.tsl2561);
                Serial.print(F(","));
                Serial.print(dataLog.temp.sht31);
                Serial.print(F(","));
                Serial.print(dataLog.temp.uv);
                Serial.print(F(","));
                Serial.print(dataLog.temp.ms5607);
                Serial.print(F(","));
                Serial.print(dataLog.temp.custom);
                Serial.print(F(","));
                Serial.print(dataLog.temp.clock);
                Serial.print(F(","));
                Serial.print(dataLog.geiger.x);
                Serial.print(F(","));
                Serial.print(dataLog.geiger.y);
                Serial.print(F(","));
                Serial.print(dataLog.geiger.z);
                Serial.print(F(","));
                Serial.print(dataLog.pwm.main);
                Serial.print(F(","));
                Serial.print(dataLog.pwm.gps);
                Serial.print(F(","));
                Serial.print(dataLog.pwm.tsl2561);
                Serial.print(F(","));
                Serial.print(dataLog.pwm.sht31);
                Serial.print(F(","));
                Serial.print(dataLog.pwm.uv);
                Serial.print(F(","));
                Serial.print(dataLog.heater_max);
                Serial.print(F(","));
                Serial.print(dataLog.battery.voltage);
                Serial.print(F(","));
                Serial.print(dataLog.battery.soc);
                Serial.print(F(","));
                Serial.print(dataLog.launched);
                Serial.print(F(","));
                Serial.print(dataLog.alarm);
                Serial.print(F(","));
                Serial.print(dataLog.found.LSM9DS1);
                Serial.print(F(","));
                Serial.print(dataLog.found.SHT31);
                Serial.print(F(","));
                Serial.print(dataLog.found.MS5607);
                Serial.print(F(","));
                Serial.print(dataLog.found.UV);
                Serial.print(F(","));
                Serial.print(dataLog.found.DS18B20);
                Serial.print(F(","));
                Serial.print(dataLog.found.TSL2561);
                Serial.print(F(","));
                Serial.print(dataLog.found.GPS);
                Serial.print(F(","));
                Serial.println(crc_match);
            }
        }
    }

    // Replace the saved block value.
    //sdcard_data.current_block = saved_current_block;
}

void queue_altitude(float altitude) {
    if (watchdog == 40) {
        watchdog = 50;
    }

    // Static flag for first pass to allow head = tail.
    static bool first_pass = true;

    bool done = false;

    // Size of array
    uint8_t queue_size = sizeof(altitude_hist)/sizeof(*altitude_hist);
    //Serial.print(F("Incoming altitude: "));
    //Serial.println(altitude);

    //if (altitude != 0) {
        // Only add if not the initial bad value of 0 pressure.
        // Move tail
        altitude_hist_tail++;

        // Over queue size
        if (altitude_hist_tail >= queue_size) {
            //Serial.println(F("Tail over queue"));
            altitude_hist_tail = 0;
        }

        // Store value
        altitude_hist[altitude_hist_tail] = altitude;

        // Do we need to move the head?
        if (altitude_hist_tail == altitude_hist_head && first_pass != true) {
            //Serial.println(F("Head = Tail"));
            altitude_hist_head = altitude_hist_tail + 1;
        }

        // Is head over the queue size?
        if (altitude_hist_head >= queue_size) {
            //Serial.println(F("Head over queue"));
            altitude_hist_head = 0;
        }
/*
        Serial.print(F("Head: "));
        Serial.print(altitude_hist_head);
        Serial.print(F(" Tail: "));
        Serial.println(altitude_hist_tail);

        uint8_t index = altitude_hist_head;
        Serial.print(F("Altitude: "));

        while (done == false) {
                if (index >= queue_size) {
                    index = 0;
                }
                if (index == altitude_hist_tail) {
                    done = true;
                }

                Serial.print(altitude_hist[index]);
                Serial.print(F(" "));

                index++;
        }
*/
        /*
        for (uint8_t i = 0; i < queue_size; i++) {
                            Serial.print(altitude_hist[i]);
                Serial.print(F(" "));
        }
        */
        //Serial.println();

        // Allow head to move now.
        first_pass = false;
    //}

    if (watchdog == 50) {
        watchdog = 60;
    }
}

void queue_pressure(float pressure) {

    if (watchdog == 60) {
        watchdog = 70;
    }
    // Static flag for first pass to allow head = tail.
    static bool first_pass = true;

    bool done = false;

    // Size of array
    uint8_t queue_size = sizeof(pressure_hist)/sizeof(*pressure_hist);
    //Serial.print(F("Incoming pressure: "));
    //Serial.println(pressure);

    //if (pressure != 0) {
        // Only add if not the initial bad value of 0 pressure.
        // Move tail
        pressure_hist_tail++;

        // Over queue size
        if (pressure_hist_tail >= queue_size) {
            //Serial.println(F("Tail over queue"));
            pressure_hist_tail = 0;
        }

        // Store value
        pressure_hist[pressure_hist_tail] = pressure;

        // Do we need to move the head?
        if (pressure_hist_tail == pressure_hist_head && first_pass != true) {
            //Serial.println(F("Head = Tail"));
            pressure_hist_head = pressure_hist_tail + 1;
        }

        // Is head over the queue size?
        if (pressure_hist_head >= queue_size) {
            //Serial.println(F("Head over queue"));
            pressure_hist_head = 0;
        }
/*
        Serial.print(F("Head: "));
        Serial.print(pressure_hist_head);
        Serial.print(F(" Tail: "));
        Serial.println(pressure_hist_tail);

        uint8_t index = pressure_hist_head;
        Serial.print(F("pressure: "));

        while (done == false) {
                if (index >= queue_size) {
                    index = 0;
                }
                if (index == pressure_hist_tail) {
                    done = true;
                }

                Serial.print(pressure_hist[index]);
                Serial.print(F(" "));

                index++;
        }
*/
        /*
        for (uint8_t i = 0; i < queue_size; i++) {
                            Serial.print(pressure_hist[i]);
                Serial.print(F(" "));
        }
        */
        //Serial.println();

        // Allow head to move now.
        first_pass = false;
    //}

    if (watchdog == 70) {
        watchdog = 80;
    }
}

void check_launch() {

    if (watchdog == 80) {
        watchdog = 90;
    }

    uint16_t pressure_count = 0;
    uint16_t alt_count = 0;

    if (configuration.launched == false) {
        for (uint16_t i = 0; i < sizeof(pressure_hist)/sizeof(*pressure_hist); i++) {
            if (pressure_hist[i] != 0 && pressure_hist[i] < configuration.pressure_alarm) {
                pressure_count++;
            }
        }

        for (uint16_t i = 0; i < sizeof(altitude_hist)/sizeof(*altitude_hist); i++) {
            if (altitude_hist[i] != 0 && altitude_hist[i] > configuration.altitude_alarm) {
                alt_count++;
            }
        }

        if (pressure_count > 90 || alt_count > 90) {
            // At least 2 intervals with 90 positive readings were spent above the threshold.
            configuration.launched = true;
        } else if (millis() / 1000L > 7200L) {
            // We've been awake for 120 minutes. Assume we've launched then.
            configuration.launched = true;
        }
    }

    if (DEBUG == true) {
        Serial.print(F("Pressure count: "));
        Serial.print(pressure_count);
        Serial.print(F(" Alt Count: "));
        Serial.println(alt_count);
    }

    if (watchdog == 90) {
        watchdog = 100;
    }
}

void check_decent() {
    static uint8_t counter = 0;

    if (configuration.launched == true && pressure_hist[pressure_hist_tail] < configuration.pressure_alarm && pressure_hist[pressure_hist_head] < configuration.pressure_alarm) {
            counter++;
    }

    if (configuration.launched == true && altitude_hist[altitude_hist_tail] < configuration.altitude_alarm && altitude_hist[altitude_hist_head] < configuration.altitude_alarm) {
            counter++;
    }

    if (configuration.launched == true && counter > 30) {
        // At least 30 seconds were spent below the threshold.
        configuration.decent = true;
    } else if (configuration.launched == true) { //} && millis() / 1000L > 14400L) {
        // We've been awake for 4 hours. Assume we've landed then.
        configuration.decent = true;
    }
}

void check_landing() {
    if (watchdog == 100) {
        watchdog = 110;
    }

    if (configuration.launched == true) {
        bool pres_threshold = true;
        bool alt_threshold = true;

        for (uint16_t i = 0; i < sizeof(pressure_hist)/sizeof(*pressure_hist); i++) {
            if (pressure_hist[i] < configuration.pressure_alarm || pressure_hist[i] == 0) {
                pres_threshold = false;
            }

        }

        for (uint16_t i = 0; i < sizeof(altitude_hist)/sizeof(*altitude_hist); i++) {
            if (altitude_hist[i] > configuration.altitude_alarm || altitude_hist[i] == 0) {
                alt_threshold = false;
            }

        }

        if (pres_threshold == true || alt_threshold == true) {
            configuration.alarm = true;
        }
    } else if (millis() / 1000L > 14400L) {
        // We've been awake for 240 minutes. Assume we've landed then.
        configuration.alarm = true;
    }

    if (watchdog == 110) {
        watchdog = 120;
    }
}

void configure_write() {

}

void check_alarms() {

    if (watchdog == 170) {
        watchdog = 180;
    }

    if (configuration.launched == true) {
        EEPROM.put(0, configuration);

        if (DEBUG == true) {
            Serial.println(F("***** LAUNCHED *****"));
            //delay(1000);
        }

        piezo_power(true);
        flash_landing(10);
        beep_piezo(1000, 250, PIEZO_PIN);
        flash_landing(250);
        beep_piezo(2000, 500, PIEZO_PIN);
        piezo_power(false);
        led_power(false);

    }

    if (configuration.alarm == true) {
        EEPROM.put(0, configuration);
        delay(100);
        watchdog_sleep();
        power_sensors(false);
        location_alarm();
    }

    if (watchdog == 180) {
        watchdog = 190;
    }
}

void print_serial_help() {
    // Prints the serial command list

    Serial.println(F("\n\n\n\rCommand help list\n\rCommands NOT case sensitive\n\r===========================\n"));
    Serial.println(F("help                  This command list.\n\r"));
    Serial.println(F("help sdcard           Help with SDCard commands."));
    Serial.println(F("help altitude         Help with altitude threshold commands."));
    Serial.println(F("help pressure         Help with pressure threshold commands."));
    Serial.println(F("help launch timer     Help with launch timer commands"));
    Serial.println(F("help location timer   Help with location timer commands\n\r"));
    Serial.println(F("read sdcard           Output the SDCard data as a CSV to serial console."));
    Serial.println(F("                      Output needs to be saved from your serial program"));
    Serial.println(F("                      or through console redirection."));
    Serial.println(F("==========================="));
}

void set_alt_threshold(float altitude) {
    if (altitude > 0 && altitude <= 100000) {
        configuration.altitude_alarm = altitude;
        EEPROM.put(0, configuration);
        Serial.println(F("Note that a low altitude will disable the altitude arming since\n\rthe GPS signal will never trigger the threshold."));
        Serial.print(F("Altitude threshold set to: "));
        Serial.print(altitude);
        Serial.print(F(" meters..."));
    } else {
        Serial.println(F("Invalid altitude.\n\rAcceptable range is 0-100000 m."));
    }
}

void set_pressure_threshold(float pressure) {

}

void set_launch_timeout(uint32_t timeout) {
}

void set_location_timeout(uint32_t timeout) {

}

void read_serial() {
    if (watchdog == 190) {
        watchdog = 200;
    }
    static String incoming = "";
    String original = "";
    char c;
    uint8_t loops = 0;

    while (Serial.available() && loops < 35) {

        c = Serial.read();
        incoming += c;

        // Echo to the screen.
        Serial.print(c);
        Serial.print(F(" 0x"));
        Serial.println(c, HEX);
        //Serial.print(F(" "));
        //Serial.println(incoming.length());

        if (incoming.length() > 34) {
            // Max length of string to prevent eating all memory.
            // Adds 2 characters for a 32 byte command plus a new line and carriage return.

            incoming = "";

            //Serial.println(F("\n\r***    to long, please retry. Max limit is 32 characters. ***"));
        }

        loops++;
    }

    if (c == "\n") {
        incoming.trim();

        if (incoming.length() > 0) {
            // The string has length, let's see what it says.
            original = incoming;
            // Convert to lower case.
            incoming.toLowerCase();
            if (incoming == "read sdcard") {
                read_sdcard();
            } else if (incoming == "help" || "?" || "??" || "/?") {
                print_serial_help();
            } else {
                Serial.print(F("*** Command unknown: "));
                Serial.print(original);
                Serial.println(F(" ***"));
                print_serial_help();
            }
        }
    }


    if (watchdog == 200) {
        watchdog = 210;
    }

}

//int8_t direction = 10;
void pet_watchdog() {
    // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
    _MemoryBarrier();
    if (watchdog == 210) {
        wdt_reset();
        watchdog = 20;
        _MemoryBarrier();
    } else {
        // Wait for reset.
        while(true) {}
    }
}

void loop() {
    //static float test_altitude = 400;
    if (watchdog == 20) {
        watchdog = 30;
    }

    // Temp override.
    read_sdcard();

    if (reset_flag == true) {
        EEPROM.put(0, default_config);
        reset_flag = false;
        flash_landing(10);
        beep_piezo(1000, 250, PIEZO_PIN);
        //delay(100);
        flash_landing(10);
        beep_piezo(2000, 250, PIEZO_PIN);
    }

    if (DEBUG == true) {
        Serial.println(F("Reading sensors..."));
    }
    // Read the sensors
    readSensors();

    /*
    dataLog.gps.altitude = test_altitude;
    test_altitude += direction;

    if (test_altitude > 3000) {
        direction = -10;
    }
*/
    queue_altitude(dataLog.gps.altitude);
    queue_pressure(dataLog.ms5607.pressure);

    check_launch();
    check_landing();

    dataLog.launched = configuration.launched;
    dataLog.alarm = configuration.alarm;

    dataLog.pwm.main = main_vPID.pwm;
    dataLog.pwm.gps = gps_vPID.pwm;
    dataLog.pwm.sht31 = sht_vPID.pwm;
    dataLog.pwm.tsl2561 = tsl_vPID.pwm;
    dataLog.pwm.uv = uv_vPID.pwm;

    log_data();


    // How's our battery?
    check_battery();

    // Control heat and power consumption
    heater_control();

    if (DEBUG == true) {

        print_sensors();

    }
    if (DEBUG == true) {
        Serial.println(F("Logging data..."));
    }

    check_alarms();

    read_serial();

    if (watchdog == 200) {
        watchdog = 210;
    }
    pet_watchdog();
    //delay(100);
    //flash_landing(10);

}
