// rf95_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <math.h>
#include "Adafruit_SleepyDog.h"

#include <RTCZero.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "TinyGPS++.h"
#include <avr/interrupt.h>


//#include "Adafruit_FONA.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define TIMEOUT 4000L

#define VBATPIN A7

#define FONA_RX 0
#define FONA_TX 1

// Hard reset. Toggle low for 100 ms to reset.
#define FONA_RST 12

// On/Off key, pulse low for 2 seconds to turn on or off
#define FONA_KEY 11

// Power status
#define FONA_PS  6

// Ring indicator. Pulsed low for 100 ms when call is received.
// Can also use setSMSInterrupt to transistion to when SMS is received.
#define FONA_RI  10

#define TFT_RESET A3

#define EARTH_RADIUS 6371008.8

bool DEBUG = false;

// this is a large buffer for replies
char replybuffer[255];

//HardwareSerial *fonaSerial = &Serial1;

// Singleton instance of the radio driver
RH_RF95 driver(8, 3);
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// 31.25 Hz, CR 4/8, 1024 chips
PROGMEM static const RH_RF95::ModemConfig modem_config[] = {0x48, 0xa4, 0x00};

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);


// Use this for FONA 800 and 808s
//Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

/* Create an rtc object */
RTCZero rtc;

struct {
        float lat = -1;
        float lng = -1;
        uint16_t altitude = -1;
        uint16_t speed = -1;
        uint16_t course = -1;

        // This seems to be the value returned by the NMEA sentence * 100.
        // Eg, 3.2 * 100 * 320.
        // However, it is supposed to be a multiplication value [00.0,99.9]
        // and multiplied by the absolute accuracy of the receiver. The
        // Venus datasheet claims 2.5m accuracy, so 3.2*2.5 = 8 m (26.2 ft)
        // This seems high, but this value is indoors as well, so will need
        // to be tested outside with a clear sky view.
        //uint16_t hdop = 0;
        //uint16_t vdop = 0;

        //uint8_t sats = 0;

        //uint32_t date = 0;
        uint8_t hour = 0;
        uint8_t minute = 0;
        uint8_t second = 0;
        uint16_t battery_adc; //, battery_percent;
} tracker_data, tracker_old;

struct {
        float lat = -1;
        float lng = -1;
        float altitude = -1;
        float speed = -1;
        float course = -1;

        // This seems to be the value returned by the NMEA sentence * 100.
        // Eg, 3.2 * 100 * 320.
        // However, it is supposed to be a multiplication value [00.0,99.9]
        // and multiplied by the absolute accuracy of the receiver. The
        // Venus datasheet claims 2.5m accuracy, so 3.2*2.5 = 8 m (26.2 ft)
        // This seems high, but this value is indoors as well, so will need
        // to be tested outside with a clear sky view.
        //uint16_t hdop = 0;
        //uint16_t vdop = 0;

        //uint8_t sats = 0;

        //uint32_t date = 0;
        uint8_t hour = 0;
        uint8_t minute = 0;
        uint8_t second = 0;
        uint16_t battery_adc, battery_percent;
} gps_data, gps_old;

typedef struct {
    uint16_t x, y;
} cords;

static cords t_time, r_cord, bearing_cord, t_course, r_course, dist_cord, rssi_cord, r_batt, t_batt, crc_cord;

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
#define TFT_DC A2
#define TFT_CS A1

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

bool gps_enabled = true;
volatile bool sms_received = false;

void SMS_interrupt() {
    sms_received = true;
}

uint32_t FreeRam() {
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    return stackTop - heapTop;
}

void save_screen(cords &save) {
    save.x = tft.getCursorX();
    save.y = tft.getCursorY();
    Serial.print(save.x);
    Serial.println(save.y);
}

void restore_screen(const cords &save) {
    tft.setCursor(save.x, save.y);

}

void draw_static() {
  tft.setCursor(0, 0);
  //tft.setTextSize(1);
  tft.print(F("T: "));
  save_screen(t_time);

  tft.println();
  tft.println();
  tft.print(F("R: "));
  save_screen(r_cord);
  tft.println();
  tft.println();

  //tft.setTextSize(1);
  tft.print(F("Bearing:  "));
  save_screen(bearing_cord);
  tft.println();
  tft.print(F("T Course: "));
  save_screen(t_course);
  tft.println();

  tft.print(F("R Course: "));
  save_screen(r_course);
  tft.println();

  //Serial.println(F("Distance..."));
  tft.print(F("Distance: "));
  save_screen(dist_cord);
  //tft.setTextSize(2);
  //float dist = distance();
    tft.println();
  tft.print("RSSI: ");
    save_screen(rssi_cord);
  tft.println();
  tft.print(F("Rec Batt: "));
    save_screen(r_batt);
  tft.println();

  tft.print(F("Tr Batt: "));
    save_screen(t_batt);
    tft.println();
  tft.print(F("GPS CRC: "));
  save_screen(crc_cord);

}

void setup() {

  // Disable it quickly in case it is like the ATMEGA328ps which reset the timeout period to
  // obscenely low values on power on!
  Watchdog.disable();

  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);

  uint32_t wdt_ms = Watchdog.enable();

  rtc.begin();

    // Set the time
  rtc.setHours(current_time.hours);
  rtc.setMinutes(current_time.minutes);
  rtc.setSeconds(current_time.seconds);

  // Set the date
  rtc.setDay(current_time.day);
  rtc.setMonth(current_time.month);
  rtc.setYear(current_time.year);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(TFT_RESET, OUTPUT);
  digitalWrite(TFT_RESET, HIGH);
  delay(5);
  digitalWrite(TFT_RESET, LOW);
  delay(20);
  digitalWrite(TFT_RESET, HIGH);
  delay(150);
  //pinMode(FONA_KEY, OUTPUT);
  //digitalWrite(FONA_KEY, LOW);
  //pinMode(FONA_RI, INPUT_PULLUP);

  pinMode(VBATPIN, INPUT);

  //Serial.begin(9600);
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.print(F("Watchdog timer: "));
  Serial.print(wdt_ms);
  Serial.println();

  //while (!Serial) ; // Wait for serial port to be available
  //while (!Serial1) ;
  Serial.println("Starting Radio...");
  //Serial1.println("Serial1");
  //Serial.println("Serial");

  manager.init();
  manager.setTimeout(TIMEOUT);
  manager.setRetries(0);
  driver.setTxPower(23, false);
  //driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
  //driver.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  //driver.setCADTimeout(TIMEOUT);
  driver.setFrequency(434);

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
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    //tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

    draw_static();
    Watchdog.reset();

}

//uint8_t data[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[sizeof(tracker_data)];

void print2digits(uint8_t number) {
  if (number < 10) {
    tft.print(0); // print a 0 before if the number is < than 10
    //Serial.print(0);
  }
  tft.print(number, DEC);
  //Serial.print(number, DEC);
}

void read_gps() {
    char read_char;
    while(Serial1.available()) {
        read_char = Serial1.read();
        //gps.encode(Serial1.read());
        gps.encode(read_char);
        //SerialUSB.print(read_char);
    }
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

uint16_t bearing(float from_lat, float from_lng, float to_lat, float to_lng) {
    double rec_lat = toRadian(from_lat);
    double rec_lng = toRadian(from_lng);

    double tran_lat = toRadian(to_lat);
    double tran_lng = toRadian(to_lng);

    double y = sin(tran_lng - rec_lng) * cos(tran_lat);
    double x = cos(rec_lat) * sin(tran_lat) - sin(rec_lat) * cos(tran_lat)*cos(tran_lng - rec_lng);

    return static_cast<uint16_t>((atan2(y, x) / M_PI * 360) + 360) % 360;
}


void loop()
{
  uint8_t len = sizeof(buf);
  static uint8_t current_rssi = 0;
  static uint8_t old_rssi;
  static float old_voltage;
  static uint16_t old_trans_batt;
  static uint32_t old_crc = 0;
  static uint16_t course_bearing;

  //static uint16_t rssi_x, rssi_y, voltage_x, voltage_y, crc_x, crc_y, trans_volt_x, trans_volt_y;


  //Serial.print(available);
  //Serial.print(F(" "));
  uint32_t current_time = millis();

  //Serial.println(FreeRam());

  Serial.println(F("Reading GPS..."));
  while (gps_enabled == true && millis() - current_time < 2000L) {
    read_gps();

  }

/*
    if (manager.waitAvailableTimeout(3000L)) {
      // Should be a message for us now
      if (driver.recv(buf, &len)) {
        current_rssi = driver.lastRssi();

        SerialUSB.println(buf[0], HEX);
        if (buf[0] == 0x55) {
          digitalWrite(13, HIGH);
          delay(50);
          digitalWrite(13, LOW);
        }
    }
  }*/

  if (gps.location.isValid() && gps.location.age() < 10000L) {
        //digitalWrite(13, HIGH);
        gps_data.lat = gps.location.lat();
        gps_data.lng = gps.location.lng();
        //digitalWrite(13, HIGH);

        //manager.sendTo(&buf, sizeof(gps_data), RH_BROADCAST_ADDRESS);
        //driver.send(buf, sizeof(buf));
        //driver.waitPacketSent(10000L);
        //digitalWrite(13, LOW);

    }

/*
    if (gps.date.isValid() && gps.date.age() < 60000L) {
        gps_data.date = gps.date.value();
    } else {
        gps_data.date = 0;
    }
*/

   if (gps.altitude.isValid() && gps.altitude.age() < 10000L) {
        gps_data.altitude = gps.altitude.meters();
    }

    if (gps.course.isValid()) {
        gps_data.course = gps.course.deg();
    }

    if (gps.speed.isValid()) {
        gps_data.speed = gps.speed.mps();
    }

    if (gps.time.isValid() && gps.time.age() < 5000L) {

            gps_data.minute = gps.time.minute();
            gps_data.second = gps.time.second();


        // Need to set the RTC. This will always set the clock when a valid GPS time is first found.
        // This protects against a set clock with backup battery having clock drift.
        // Also synchronizes the clocks.

            rtc.setDay(gps.date.day());
            rtc.setMonth(gps.date.month());
            rtc.setYear(gps.date.year());
            rtc.setMinutes(gps.time.minute());
            int8_t hours = gps.time.hour() - 5;
            if (hours < 0) {
                hours = hours + 24;
            }
            gps_data.hour = hours;
            rtc.setHours(hours);
            rtc.setSeconds(gps.time.second());
            set_clock = true;

    }



  /*Serial.println(set_clock);
  Serial.print(gps.time.hour());
  Serial.print(F(":"));
  Serial.print(gps.time.minute());
  Serial.print(F(":"));
  Serial.println(gps.time.second());
*/

  gps_data.battery_adc = analogRead(VBATPIN);




  //pinMode(VBATPIN, OUTPUT);
  //digitalWrite(VBATPIN, HIGH);
  //measuredvbat = 1000;
  //measuredvbat *= 2;    // we divided by 2, so multiply back
  //measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  //measuredvbat /= 1024; // convert to voltage
  measuredvbat = gps_data.battery_adc * 0.0064453125;

  Watchdog.reset();

  Serial.println(F("Checking radio..."));

  if (manager.recvfromAck(buf, &len)) {
    // Should be a message for us now
    driver.setModeRx();
    digitalWrite(13, HIGH);

    current_rssi = driver.lastRssi();
    memcpy(&tracker_data, &buf, sizeof(tracker_data));
  }
  delay(50);
  digitalWrite(13, LOW);


  //Serial.print("VBat: " ); Serial.println(measuredvbat);
  //measuredvbat *= 0.006445313;
/*
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(tracker_old.lat, 6);
  tft.print(", ");
  tft.println(tracker_old.lng, 6);
  //tft.print("RSSI: ");
  tft.setCursor(rssi_x, rssi_y);
  tft.print(old_rssi, DEC);
  tft.setCursor(voltage_x, voltage_y);
  tft.print(old_voltage, 3);
  tft.println("v");
  //tft.setCursor(crc_x, crc_y);
  //tft.println(old_crc, DEC);
    //tft.print(F("Tr Batt: "));
  tft.setCursor(trans_volt_x, trans_volt_y);
  tft.print(tracker_old.battery_percent);
  tft.println(F("%"));
*/
    Serial.println(F("Displaying..."));
  Watchdog.reset();

  uint32_t timing = millis();

  noInterrupts();

  restore_screen(t_time);
  print2digits(tracker_data.hour);
  tft.print(F(":"));
  print2digits(tracker_data.minute);
  tft.print(F(":"));
  print2digits(tracker_data.second);
  tft.println();

  tft.print(tracker_data.lat, 7);
  tft.print(",");
  tft.println(tracker_data.lng, 7);

  restore_screen(r_cord);
  tft.print(gps_data.lat, 7);
  tft.print(",");
  tft.println(gps_data.lng, 7);

  print2digits(gps_data.hour);
  tft.print(F(":"));
  print2digits(gps_data.minute);
  tft.print(F(":"));
  print2digits(gps_data.second);
  tft.print(F("   "));


  //tft.setTextSize(1);
  restore_screen(bearing_cord);
  //tft.print(static_cast<int>(bearing()), DEC);
  tft.print(static_cast<int>(gps.courseTo(gps_data.lat, gps_data.lng, tracker_data.lat, tracker_data.lng)), DEC);
  tft.println(F(" deg    "));

  restore_screen(t_course);
  tft.print(tracker_data.course);
  tft.println(F(" deg    "));

  restore_screen(r_course);
  tft.print(static_cast<uint16_t>(gps.courseTo(gps_old.lat, gps_old.lng, gps_data.lat, gps_data.lng)), DEC);
  tft.println(F(" deg    "));

  //Serial.println(F("Distance..."));
  restore_screen(dist_cord);
  //tft.setTextSize(2);
  //float dist = distance();
  float dist = 3.28 * gps.distanceBetween(gps_data.lat, gps_data.lng, tracker_data.lat, tracker_data.lng);
  if (dist >= 5280) {
    tft.print(dist/5280, 1);

    tft.println(F(" miles    "));
  } else {
    tft.print(dist, 1);
    tft.println(F(" ft       "));
  }

  restore_screen(rssi_cord);
  //rssi_x = tft.getCursorX();
  //rssi_y = tft.getCursorY();

  //tft.setTextSize(1);
  //Serial.println(F("Batt..."));
  tft.print(current_rssi, DEC);
  tft.println(F("  "));

  restore_screen(r_batt);
  //voltage_x = tft.getCursorX();
  //voltage_y = tft.getCursorY();
  //tft.setTextSize(2);
  tft.print(measuredvbat, 2);
  tft.println("v ");

  restore_screen(t_batt);
  //trans_volt_x = tft.getCursorX();
  //trans_volt_y = tft.getCursorY();
  //tft.setTextSize(2);
  //tft.print(tracker_data.battery_percent);
  //tft.print(F("% "));
  tft.print(static_cast<float>(tracker_data.battery_adc)/1000, 2);
  tft.println(F("v "));

  restore_screen(crc_cord);
  tft.print(gps.failedChecksum());

  interrupts();
  uint32_t second = millis();
  Serial.println(second - timing);
  gps_old = gps_data;

  /*
  tft.print("GPS CRC: ");
  crc_x = tft.getCursorX();
  crc_y = tft.getCursorY();
  tft.println(gps.failedChecksum(), DEC);*/
  //if (current_rssi != old_rssi) {
  //  old_rssi = current_rssi;
  //}

  //old_voltage = measuredvbat;
  //  tracker_old = tracker_data;


  //old_crc = gps.failedChecksum();
 /*
  memcpy(&buf, &gps_data, sizeof(gps_data));
  //manager.sendtoWait(buf, sizeof(buf), SERVER_ADDRESS);
  driver.send(buf, sizeof(buf));
  driver.waitPacketSent(2000);


  if (manager.recvfromAckTimeout(buf, &len, 100)) {
      digitalWrite(13, HIGH);
      delay(100);
      tft.setCursor(0, 0);
      tft.setTextColor(ILI9341_BLACK);
      print2digits(old_time.hours);
      tft.print(":");
      //Serial.print(":");
      print2digits(old_time.minutes);
      tft.print(":");
      //Serial.print(":");
      print2digits(old_time.seconds);

      //tft.println();
      //tft.print("RSSI: ");
      tft.setCursor(rssi_x, rssi_y);
      tft.print(old_rssi, DEC);

      tft.setCursor(voltage_x, voltage_y);
      tft.print(old_voltage);
      tft.println("v");

      memcpy(&current_time, &buf, sizeof(current_time));

      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(0, 0);
      print2digits(current_time.hours);
      tft.print(":");
      //Serial.print(":");
      print2digits(current_time.minutes);
      tft.print(":");
      //Serial.print(":");
      print2digits(current_time.seconds);

      tft.println();
      tft.print("RSSI: ");
      rssi_x = tft.getCursorX();
      rssi_y = tft.getCursorY();

      uint8_t current_rssi = driver.lastRssi();
      tft.print(current_rssi, DEC);
      tft.print("   Batt: ");
      voltage_x = tft.getCursorX();
      voltage_y = tft.getCursorY();
      tft.print(measuredvbat);
      tft.println(" v");
      old_rssi = current_rssi;
      old_voltage = measuredvbat;

      memcpy(&old_time, &current_time, sizeof(old_time));
      digitalWrite(13, LOW);
    }*/
}

