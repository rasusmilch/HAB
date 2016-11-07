// TODO: Proper WDT.

#include <Arduino.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <avr/interrupt.h>

#include "Adafruit_SleepyDog.h"
#include <RTCZero.h>
//#include "Adafruit_GFX.h"
//#include "Adafruit_ILI9341.h"
#include "TinyGPS++.h"
#include "Adafruit_FONA.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define TIMEOUT 4000L

#define VBATPIN 9

#define FONA_RX 0
#define FONA_TX 1

// Hard reset. Toggle low for 100 ms to reset.
#define FONA_RST 12

// On/Off key, pulse low for 2 seconds to turn on or off
#define FONA_KEY 11

// Power status
#define FONA_PS  6

// Ring indicator. Pulsed low for 100 ms when call is received.
// Can also use setSMSInterrupt to transition to when SMS is received.
#define FONA_RI  10

bool DEBUG = true;

// We need to supply a current location.
bool current_location_flag = false;

// Number to reply to
char reply_number[255];

// this is a large buffer for replies
char replybuffer[255];
char sms_from[255];
char sms_buff[255];
//char char_sms_temp_str[255];
uint8_t reply_slot;

String sms_temp_str = "";


const char current_location[] = "CURRENT LOCATION";
const char repeat_location[] = "REPEAT LOCATION";
const char stop_location[] = "STOP LOCATION";


uint8_t fona_type;

//HardwareSerial *fonaSerial = &Serial1;

// Singleton instance of the radio driver
RH_RF95 driver(8, 3);
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// 31.25 Hz, CR 4/8, 1024 chips
PROGMEM static const RH_RF95::ModemConfig modem_config[] = {0x48, 0xa4, 0x00};

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);


// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

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
#define TFT_CS 6

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

bool gps_enabled = true;
volatile bool sms_received = true;

void SMS_interrupt() {
    sms_received = true;
}

bool power_fona(bool power) {

    for (uint8_t i = 0; i < 10; i++) {
        // Test for desired power for ten loops
        if (digitalRead(FONA_PS) != power) {
            // Pull low for 2 seconds
            digitalWrite(FONA_KEY, LOW);
            delay(2100);
            // Place into high
            //pinMode(FONA_KEY, INPUT);
            digitalWrite(FONA_KEY, HIGH);

        } else {
            // In the mode we desire, exit
            //Serial.println("Exiting FONA power");
            return true;
        }
        delay(100);
    }
    // Couldn't power cycle.
    return false;

}

void setup() {
    sms_temp_str.reserve(185);
  // Disable it quickly in case it is like the ATMEGA328ps which reset the timeout period to
  // obscenely low values on power on!
  Watchdog.disable();

  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);
  rtc.begin();
    // Set the time
  rtc.setHours(current_time.hours);
  rtc.setMinutes(current_time.minutes);
  rtc.setSeconds(current_time.seconds);

  // Set the date
  rtc.setDay(current_time.day);
  rtc.setMonth(current_time.month);
  rtc.setYear(current_time.year);

  //pinMode(VBATPIN, INPUT);

  Serial.begin(9600);

  //Serial.begin(9600);

  Serial1.begin(9600);
  //while (!Serial) ; // Wait for serial port to be available
  uint32_t wdt_ms = Watchdog.enable();
  Serial.print(F("Watchdog timer: "));
  Serial.print(wdt_ms);
  Serial.println();


  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);

  power_fona(true);

  pinMode(FONA_RI, INPUT_PULLUP);
  //while (!Serial1) ;
  if (DEBUG == true) {
    Serial.println("Starting Radio...");
  }
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
  if (DEBUG == true) {
    Serial.println("Starting fona...");
  }

  if (!fona.begin(Serial1)) {
    if (DEBUG == true) {
        Serial.println(F("Couldn't find FONA"));
    }
  } else {
        if (DEBUG == true) {
            Serial.println(F("FONA started"));
        }
        fona_type = fona.type();
        fona.enableNetworkTimeSync(true);
        fona.enableRTC(true);
        fona.enableCharge(true);
        fona.enableGPRS(false);

        if (!fona.enableGPS(true)) {
           if (DEBUG == true) {
            Serial.println(F("Failed to turn on"));
           }
        } else {
            if (DEBUG == true) {
              Serial.println(F("GPS Enabled"));
            }
          //fona.enableGPSNMEA(true);
        }
        // Set interrupt through RI pin to true when SMS is received.
        fona.setSMSInterrupt(true);
        attachInterrupt(digitalPinToInterrupt(FONA_RI), SMS_interrupt, FALLING);
  }

 // Serial.println("Starting TFT...");
/*
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);
    //tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);*/

    Watchdog.reset();
}

//uint8_t data[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[sizeof(gps_data)];
/*
void print2digits(int number) {
  if (number < 10) {
    tft.print("0"); // print a 0 before if the number is < than 10
    //Serial.print("0");
  }
  tft.print(number);
  //Serial.print(number);
}
*/
void read_gps() {
    char read_char;

    while(fona.available()) {

        read_char = fona.read();
        //gps.encode(Serial1.read());
        gps.encode(read_char);
        //Serial.print(read_char);
    }
    //digitalWrite(13, LOW);
}

void check_sms() {
    if (sms_received == true) {
               // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        int8_t smsn;

        if ( (fona_type == FONA3G_A) || (fona_type == FONA3G_E) ) {
          smsn = 0; // zero indexed
          smsnum--;
        } else {
          smsn = 1;  // 1 indexed
        }

        for ( ; smsn <= smsnum; smsn++) {

          Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);


          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
            Serial.println(F("Failed!"));
            break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            Serial.println(F("[empty slot]"));
            smsnum++;
            continue;
          }

          Serial.print(F("***** SMS #")); Serial.print(smsn);
            // Retrieve SMS sender address/phone number.
          if (! fona.getSMSSender(smsn, sms_from, 250)) {
            Serial.println("Failed!");
            continue;
          }
          Serial.print(F("FROM: ")); Serial.println(sms_from);

          Serial.print(" bytes from strlen. ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
          Serial.println(replybuffer);
          Serial.println(F("*****"));

          if (strstr(replybuffer, current_location) != NULL) {
            Serial.println(F("Found current location command!"));
            current_location_flag = true;
            // Save for delete after processing...
            reply_slot = smsn;
            strcpy(reply_number, sms_from);

            //fona.deleteSMS(smsn);
          } else {
            // No commands, lets remove to not block incoming messages due to full SMS buffer.
            Serial.print(F("Deleting SMS "));
            Serial.println(smsn);
            fona.deleteSMS(smsn);
          }


        }
      }

    sms_received = false;
    Watchdog.reset();
}

void check_commands() {

    if (current_location_flag == true) {
        Serial.println(F("Current location flag!"));
        current_location_sms();
    }

}

void current_location_sms() {
    float latitude, longitude;
    float course, altitude, speed;


    fona.getGPS(&gps_data.lat, &gps_data.lng, &speed, &course, &altitude);
    gps_data.altitude = static_cast<uint16_t>(altitude);
    gps_data.course = static_cast<uint16_t>(course);
    gps_data.speed = static_cast<uint16_t>(speed);
    // Get battery stats
    fona.getBattVoltage(&gps_data.battery_adc);
    //fona.getBattPercent(&gps_data.battery_percent);

    Serial.print(F("GPS: "));
    Serial.print(gps_data.lat, 9);
    Serial.print(F(", "));
    Serial.print(gps_data.lng, 9);
    Serial.print(F(" Alt: "));
    Serial.print(gps_data.altitude * 3.28);
    Serial.print(F(" ft "));
    Serial.print(F(" Course: "));
    Serial.print(gps_data.course);
    Serial.print(F(" deg "));
    Serial.print(gps_data.speed * 2.23694);
    Serial.println(F(" mph"));

    // Note that we ADD to the string, this allows sending other messages with the location data.
    // Such as a low battery warning with location. Since we're already sending an SMS this should be relatively free,
    // unless turning on GPRS causes a huge battery drop.
    sms_temp_str = sms_temp_str + "http://maps.google.com/?q=";                // 26

    sms_temp_str = sms_temp_str + String(gps_data.lat, 9);              // 14
    sms_temp_str = sms_temp_str + ",";                                  // 1
    sms_temp_str = sms_temp_str + String(gps_data.lng, 9);              // 14
    sms_temp_str = sms_temp_str + "\n\r";                               // 2
    sms_temp_str = sms_temp_str + "Alt: ";                              // 5
    sms_temp_str = sms_temp_str + String(gps_data.altitude * 3.28, 1);  // 7
    sms_temp_str = sms_temp_str + " ft. Course: ";                      // 13
    sms_temp_str = sms_temp_str + String(gps_data.course);              // 3
    sms_temp_str = sms_temp_str + " deg at ";                           // 8
    sms_temp_str = sms_temp_str + String(gps_data.speed * 2.23694, 1);  // 5
    sms_temp_str = sms_temp_str + " mph.";                              // 5
    //Serial.println(sms_temp_str);
    //Serial.println(char_sms_temp_str);

    //uint8_t n = sprintf(sms_buff, "GPS: %.6f, %.6f Alt: %d ft. Course: %d deg at %.1f mph.", static_cast<float>(gps_data.lat), static_cast<float>(gps_data.lng), static_cast<uint32_t>(gps_data.altitude * 3.28), gps_data.course, static_cast<float>(gps_data.speed * 2.23694));


    Watchdog.reset();
    // Check for network, then GPRS
  Serial.println(F("Checking for Cell network..."));
  if (fona.getNetworkStatus() == 1) {
    Serial.println(F("Resetting GPRS..."));
    //Serial.println(F("Disabling GPRS"));
    fona.enableGPRS(false);

    // turn GPRS on
    if (!fona.enableGPRS(true))
        Serial.println(F("Failed to turn GPRS on"));

    // network & GPRS? Great! Print out the GSM location to compare
    boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);

    if (gsmloc_success) {
        //sms_temp_str = sms_temp_str + "\n\rGSMLoc: ";
        sms_temp_str = sms_temp_str + "\n\rGSM\n\rhttp://maps.google.com/?q=";  // 33
        sms_temp_str = sms_temp_str + String(latitude, 9);                      // 14
        sms_temp_str = sms_temp_str + ",";                                      // 1
        sms_temp_str = sms_temp_str + String(longitude, 9);                     // 14

      Serial.print("GSMLoc: ");
      Serial.print(latitude, 9);
      Serial.print(", ");
      Serial.println(longitude, 9);
    } else {
      Serial.println("GSM location failed...");
    }
  }

  fona.enableGPRS(false);
    sms_temp_str = sms_temp_str + "\n\rBatt: ";
    sms_temp_str = sms_temp_str + String(static_cast<float>(gps_data.battery_adc * 0.0064453125), 2);
    sms_temp_str = sms_temp_str + "v";
    sms_temp_str.toCharArray(sms_buff, sizeof(sms_buff));
    Serial.println(F("SMS Char Buffer: "));
    Serial.println(sms_buff);
    // Send SMS!
    Watchdog.reset();
    if(fona.sendSMS(reply_number, sms_buff)) {
        Serial.println(F("Location SMS sent!"));
        fona.deleteSMS(reply_slot);
        current_location_flag = false;
    } else {
        Serial.println(F("Location SMS failed..."));
    }

    // Clear the string for next use... I hope it avoids fragmentation.
    sms_temp_str = "";

    Watchdog.reset();
}

void loop()
{
  uint8_t len = sizeof(buf);
  static uint8_t current_rssi = 0;
  static uint8_t old_rssi;
  static float old_voltage;
  static uint32_t old_crc = 0;
  static uint16_t rssi_x, rssi_y, voltage_x, voltage_y, crc_x, crc_y;


  //Serial.print(available);
  //Serial.print(F(" "));
  uint32_t current_time = millis();

  //while (gps_enabled == true && millis() - current_time < 2000L) {
    //read_gps();

  //}

/*
    if (manager.waitAvailableTimeout(3000L)) {
      // Should be a message for us now
      if (driver.recv(buf, &len)) {
        current_rssi = driver.lastRssi();

        Serial.println(buf[0], HEX);
        if (buf[0] == 0x55) {
          digitalWrite(13, HIGH);
          delay(50);
          digitalWrite(13, LOW);
        }
    }
  }*/
/*
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
    */
/*
    if (gps.date.isValid() && gps.date.age() < 60000L) {
        gps_data.date = gps.date.value();
    } else {
        gps_data.date = 0;
    }
*/
/*
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

    if (gps.time.isValid() && gps.time.age() < 5000L) {
        gps_data.time = gps.time.value();

        // Need to set the RTC. This will always set the clock when a valid GPS time is first found.
        // This protects against a set clock with backup battery having clock drift.
        // Also synchronizes the clocks.
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
*/

    // read the time

// read the time
    //char buffer[23];

    //fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!

    // Disable GPS
    //fona.enableGPSNMEA(false);
    // Read what's left.
    //read_gps();
    float course, altitude, speed;

    fona.getGPS(&gps_data.lat, &gps_data.lng, &speed, &course, &altitude);
    gps_data.altitude = static_cast<uint16_t>(altitude);
    gps_data.course = static_cast<uint16_t>(course);
    gps_data.speed = static_cast<uint16_t>(speed);
    // Get battery stats
    fona.getBattVoltage(&gps_data.battery_adc);
    //fona.getBattPercent(&gps_data.battery_percent);

    Serial.print(gps_data.lat);
    Serial.print(F(", "));
    Serial.println(gps_data.lng);
    Serial.println(sizeof(gps_data));
    //fona.enableGPSNMEA(true);

  //gps_data.battery_adc = analogRead(VBATPIN);


    //char gpsdata[120];
    //fona.getGPS(0, gpsdata, 120);
    //Serial.println(gpsdata);

    //String gps_string = String(gpsdata);
    //Serial.println(gps_string);
    //uint8_t commaIndex = gps_string.indexOf(',');
    //uint8_t secondIndex = gps_string.indexOf(',', commaIndex + 1);
    //uint8_t gps_mode = gps_string.substring(0, commaIndex)

  //pinMode(VBATPIN, OUTPUT);
  //digitalWrite(VBATPIN, HIGH);
  //measuredvbat = 1000;
  //measuredvbat *= 2;    // we divided by 2, so multiply back
  //measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  //measuredvbat /= 1024; // convert to voltage

  memcpy(&buf, &gps_data, sizeof(gps_data));
  //digitalWrite(13, LOW);
  Watchdog.reset();

  uint32_t timing = millis();

  digitalWrite(13, HIGH);
  if (manager.sendtoWait(buf, sizeof(gps_data), RH_BROADCAST_ADDRESS)) {
    uint32_t time2 = millis();
    current_rssi = driver.lastRssi();
    if (DEBUG == true) {
        Serial.print("Time to send and receive was: ");
        Serial.print(time2 - timing);
        Serial.println("ms.");
    }
    // Got an ack.
    Watchdog.reset();
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(50);
  } else {
    Serial.println(F("Time out on send..."));
  }
  digitalWrite(13, LOW);
  Watchdog.reset();

  check_sms();
  check_commands();

  //measuredvbat = gps_data.battery_adc * 0.0064453125;

  //Serial.print("VBat: " ); Serial.println(measuredvbat);
  //measuredvbat *= 0.006445313;
/*
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.print(gps_old.lat, 6);
  tft.print(", ");
  tft.println(gps_old.lng, 6);
  //tft.print("RSSI: ");
  tft.setCursor(rssi_x, rssi_y);
  tft.print(old_rssi, DEC);
  tft.setCursor(voltage_x, voltage_y);
  tft.print(old_voltage, 3);
  tft.println("v");
  tft.setCursor(crc_x, crc_y);
  tft.println(old_crc, DEC);

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.print(gps_data.lat, 6);
  tft.print(", ");
  tft.println(gps_data.lng, 6);
  tft.println();
  tft.print("RSSI: ");
  rssi_x = tft.getCursorX();
  rssi_y = tft.getCursorY();

  tft.println(current_rssi, DEC);
  tft.print("Batt: ");
  voltage_x = tft.getCursorX();
  voltage_y = tft.getCursorY();
  tft.print(measuredvbat, 3);
  tft.println("v");
  tft.print("GPS CRC: ");
  crc_x = tft.getCursorX();
  crc_y = tft.getCursorY();
  tft.println(gps.failedChecksum(), DEC);
  old_rssi = current_rssi;
  old_voltage = measuredvbat;
  gps_old.lat = gps_data.lat;
  gps_old.lng = gps_data.lng;
  old_crc = gps.failedChecksum();*/
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

