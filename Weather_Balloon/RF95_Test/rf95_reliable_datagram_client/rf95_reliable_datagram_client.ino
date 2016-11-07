/*
Robert Susmilch
robert@susmilch.com

Example of poor performing LoRa radio sketch.

This is part two of two. This code (the client) feeds the server node
GPS coordinate data for logging out to serial console for range testing.

This config works,
driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512)

This config does not
driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096)

In addition, some time the receiving radio would not receive the packet
(eg, the data structure AND raw char buffer would be blank.)
Upon a reset, several data packets would be received, then go blank again.
Adafruit ties a pin to the radio reset line, Rocket Scream does not. Only recourse is the watch dog
timer. The watchdog timer is not out on the internet for the SAMD, however, Adafruit has an
undocumented addition to their SleepyDog that works!

Server code is better documented.
*/

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#include <RTCZero.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "TinyGPS++.h"
//#include "Adafruit_FONA.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define TIMEOUT 6000L

#define VBATPIN A5

#define EARTH_RADIUS 6371008.8

// Singleton instance of the radio driver
//RH_RF95 driver(8, 3);
RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// 31.25 Hz, CR 4/8, 1024 chips
PROGMEM static const RH_RF95::ModemConfig modem_config[] = {0x48, 0xa4, 0x00};

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
#define Serial SerialUSB

/* Create an rtc object */
RTCZero rtc;

struct {
        double lat, lng = 0;
        //double altitude = 0;
        //double speed, course = 0;

        //uint16_t hdop = 0;
        //uint16_t vdop = 0;

        //uint8_t sats = 0;

        //uint32_t date = 0;
        uint32_t time = 0;
        uint16_t battery_adc;
} gps_data, gps_old;

TinyGPSPlus gps;

bool set_clock = false;

struct {
  uint8_t seconds, minutes, hours = 0;
  uint8_t day = 15;
  uint8_t month = 6;
  uint8_t year = 15;
} current_time, old_time;

float measuredvbat;

// For the Adafruit shield, these are the default.
#define TFT_DC 10
#define TFT_CS 9

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

bool gps_enabled = true;

void setup()
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

    // Set the time
  rtc.setHours(current_time.hours);
  rtc.setMinutes(current_time.minutes);
  rtc.setSeconds(current_time.seconds);

  // Set the date
  rtc.setDay(current_time.day);
  rtc.setMonth(current_time.month);
  rtc.setYear(current_time.year);


  Serial.begin(9600);
  Serial1.begin(9600);

  //while (!Serial) ; // Wait for serial port to be available
  //while (!Serial1) ;
  Serial.println("Starting Radio...");


  manager.init();
  manager.setTimeout(TIMEOUT);
  manager.setRetries(1);
  driver.setTxPower(23, false);

  //driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
  //driver.setModemConfig(RH_RF95::Bw125Cr45Sf128);

  driver.setCADTimeout(TIMEOUT);
  driver.setFrequency(915);

    //Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:


  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);
  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:

  Serial.println("Starting TFT...");

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);

}

// Dont put this on the stack:
uint8_t buf[sizeof(gps_data)];

void print2digits(int number) {
  if (number < 10) {
    tft.print("0"); // print a 0 before if the number is < than 10
    //Serial.print("0");
  }
  tft.print(number);
  //Serial.print(number);
}

float toRadian(float degrees) {
    return (degrees / 180) * M_PI;
}

float distance(float from_lat, float from_lng, float to_lat, float to_lng) {
    double rec_lat = toRadian(from_lat);
    //float rec_lng = toRadian(gps_data.lng);

    double tran_lat = toRadian(to_lat);
    //float tran_lng = toRadian(tracker_data.lng);

    double diff_lat = toRadian(from_lat - to_lat);
    double diff_lng = toRadian(from_lng - to_lng);

    double a = sin(diff_lat / 2) * sin(diff_lat / 2) + cos(rec_lat) * cos(tran_lat) * sin(diff_lng / 2) * sin(diff_lng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;

}


void loop()
{
  uint8_t len = sizeof(buf);
  uint8_t current_rssi = 0;
  static uint8_t old_rssi;
  static float old_voltage;
  static uint32_t old_crc = 0;
  static uint16_t rssi_x, rssi_y, voltage_x, voltage_y, crc_x, crc_y;

  char read_char;

  uint32_t current_time = millis();

  while (gps_enabled == true && millis() - current_time < 2000L) {
    while(Serial1.available()) {
      read_char = Serial1.read();
      //gps.encode(Serial1.read());
      gps.encode(read_char);
      //SerialUSB.print(read_char);
    }
  }

  if (gps.location.isValid() && gps.location.age() < 10000L) {
        gps_data.lat = gps.location.lat();
        gps_data.lng = gps.location.lng();

    }
/*
    if (gps.date.isValid() && gps.date.age() < 60000L) {
        gps_data.date = gps.date.value();
    } else {
        gps_data.date = 0;
    }


   if (gps.altitude.isValid() && gps.altitude.age() < 10000L) {
        gps_data.altitude = gps.altitude.meters();
    } else {
        gps_data.altitude = 0;
    }

    if (gps.course.isValid()) {
        gps_data.course = gps.course.deg();
    } else {
        gps_data.course = -1;
    }

    if (gps.speed.isValid()) {
        gps_data.speed = gps.speed.mps();
    } else {
        gps_data.speed = -1;
    }
*/
    if (gps.time.isValid() && gps.time.age() < 5000L) {
        gps_data.time = gps.time.value();

        // Need to set the RTC. This will always set the clock when a valid GPS time is first found.
        // This protects against a set clock with backup battery having clock drift.
        // Also synchronizes the clocks.

        // RTC clocks lose sync due to interrupt disabling for TFT... disappointing.
        if (set_clock == false) {
            rtc.setDay(gps.date.day());
            rtc.setMonth(gps.date.month());
            rtc.setYear(gps.date.year());
            rtc.setMinutes(gps.time.minute());
            rtc.setHours(gps.time.hour());
            rtc.setSeconds(gps.time.second());
            set_clock = true;
        }
    }

  gps_data.battery_adc = analogRead(VBATPIN);

  memcpy(&buf, &gps_data, sizeof(gps_data));

  uint32_t timing = millis();
    Serial.println(distance(47.4924817,-94.8873933, gps_data.lat, gps_data.lng));

  if (distance(47.4924817,-94.8873933, gps_data.lat, gps_data.lng) > 50) {
    // Display feedback that we're transmitting.
    digitalWrite(13, HIGH);

      // Don't wait for acknowledgment, just send it and hope for the best.
      if (manager.sendtoWait(buf, sizeof(gps_data), RH_BROADCAST_ADDRESS)) {
        uint32_t time2 = millis();
        current_rssi = driver.lastRssi();
        Serial.print("Time to send and receive was: ");
        Serial.print(time2 - timing);
        Serial.println("ms.");

      } else {
        Serial.println(F("Timed out"));
      }
    // Off LED.
    digitalWrite(13, LOW);

  }


  measuredvbat = gps_data.battery_adc * 0.0064453125;

  // Protect against radio interrupts hanging the code due to TFT lock up.
  noInterrupts();

  tft.setCursor(0, 0);
  tft.print(gps_data.lat, 6);
  tft.print(", ");
  tft.println(gps_data.lng, 6);
  tft.println();
  tft.print("RSSI: ");

  tft.print(current_rssi, DEC);
  tft.println(F("   "));
  tft.print("Batt: ");

  tft.print(measuredvbat, 2);
  tft.println("v    ");
  tft.print("GPS CRC: ");

  tft.println(gps.failedChecksum(), DEC);
  interrupts();

}

