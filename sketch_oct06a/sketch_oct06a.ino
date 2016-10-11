#include <Wire.h>
#include "SparkFunMAX17043.h"
#include <MS5xxx.h>
#include "SparkFunLSM9DS1.h"
#include "Sd2Card.h"
#include "TinyGPS++.h"


// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

MAX17043 fuel_gauge;
// Altimeter
MS5xxx ms5607(&Wire);
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
    // Box internal temp
    // External temp
    // Main
    // GPS
    // TSL2561
    // SHT31
    // UV
    // Custom daughter board (if any)
    struct {
        float main = 0;
        float internal = 0;
        float external = 0;
        float gps = 0;
        float tsl2561 = 0;
        float sht31 = 0;
        float uv = 0;
        float ms5607 = 0;
        float custom = 0;

    } temp;

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

    //uint16_t crc;

} data_struct;

// Create global sensor structure
data_struct dataLog;

TinyGPSPlus gps;

// Grab vertical dilution of precision.
TinyGPSCustom vdop(gps, "GPGSA", 17);

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

void setup() {


  // MS5607, SHT31, TSL2561
    DDRJ |= (1 << PJ6) | (1 << PJ3) | (1 << PJ5);
    PORTJ |= (1 << PJ6) | (1 << PJ5);
    PORTJ &= ~(1 << PJ3);

  // 9DOF
    DDRA |= (1 << PA7);
    PORTA |= (1 << PA7);

  // 9DOF
    DDRG |= (1 << PG1);
    PORTG|= (1 << PG1);

  // GPS
    DDRL |= (1 << PL7) ;
    PORTL |= (1 << PL7);
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

      // Start the fuel gauge
    Serial.println(F("Configuring fuel gauge..."));
    fuel_gauge.begin();

    // Restart for more accurate reading.
    Serial.println(F("Quick-starting fuel gauge..."));
    fuel_gauge.quickStart();


    pinMode(45, OUTPUT);
    //analogWrite(45, 175);
    pinMode(22, OUTPUT);
    digitalWrite(22,LOW);
        pinMode(23, OUTPUT);
    digitalWrite(23,LOW);
        pinMode(24, OUTPUT);
    digitalWrite(24,LOW);
    
    
       if(ms5607.connect()>0) {
        Serial.println(F("Error connecting to MS5607..."));
        // Do something about fail here.

    } else {
        // Load calibration from ROM of sensor.
        ms5607.ReadProm();

    }

        DOF.settings.device.commInterface = IMU_MODE_I2C;
    DOF.settings.device.mAddress = LSM9DS1_M;
    DOF.settings.device.agAddress = LSM9DS1_AG;

    // The above lines will only take effect AFTER calling
    // DOF.begin(), which verifies communication with the DOF
    // and turns it on.
    if (!DOF.begin()) {
        Serial.println(F("Failed to communicate with LSM9DS1."));
        Serial.println(F("Double-check wiring."));


    } else {
 
        DOF.calibrate();
        DOF.calibrateMag();
    }

    if (sdcard.init(SPI_HALF_SPEED, SS_PIN)==0) {
        Serial.print(F("Something wrong initilizing SDCard...\nError: "));
        Serial.println(sdcard.errorCode(), HEX);
    } else {
        Serial.print(F("Card size: "));
        Serial.print(sdcard.cardSize() - 1);
        Serial.println(F(" blocks"));
    }

    uint32_t tail = find_sdcard_tail(1, sdcard.cardSize() - 1);

    if (tail != 0) {
        Serial.print(F("\nFound tail of log at block: "));
        Serial.println(tail);
    }

}

void loop() {
  // put your main code here, to run repeatedly:
          // Read the altitude sensor values into object for next round?
        ms5607.Readout();
    Serial.print(fuel_gauge.getVoltage());
    Serial.print(" Volts. ");
    Serial.print(fuel_gauge.getSOC());
    Serial.print("% ");

    Serial.print(ms5607.GetPres());

        DOF.readAccel();
  Serial.print(" ");
  Serial.print(DOF.ax);
  Serial.print(" ");
  Serial.print(DOF.ay);
  Serial.print(" ");
  Serial.println(DOF.az);

      char read_char;
    //Serial.print(available);
    //Serial.print(F(" "));
    while(Serial1.available()) {
        read_char = Serial1.read();
        //gps.encode(Serial1.read());
        gps.encode(read_char);
        Serial.print(read_char);
    }

}
