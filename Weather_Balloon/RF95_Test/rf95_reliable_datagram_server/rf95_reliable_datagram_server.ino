/*
Robert Susmilch
robert@susmilch.com

Example of poor performing LoRa radio sketch.

This is part one of two. This code (the server) accepts data from the "client" and feeds it out onto the
serial port for logging where packets are heard from as a range test. The client feeds this node
GPS coordinate data for logging.

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

*/

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "Adafruit_SleepyDog.h"

#include <RTCZero.h>

#define TIMEOUT 6000L

//#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  //#define Serial SERIAL_PORT_USBVIRTUAL
//#endif



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

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
//RH_RF95 driver(RFM95_CS, RFM95_INT);
RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
#define Serial SerialUSB
void reset_modem() {
  Serial.println(F("Resetting modem..."));
  //digitalWrite(4, LOW); // Adafruit Radio reset pin, not on RocketScream

  // Always set manager first, regardless of what RadioHead documentation states.
  // Manager will init driver, and docs say to set up driver first before initing driver.
  // However, placing driver config before manager init hangs radio.
  // Also, in manager implementation, it ignores the driver settings and specifies it's own default!!!
  manager.init();
  manager.setTimeout(TIMEOUT);
  manager.setRetries(1);
  driver.setTxPower(23, false);

  // ***********************************************
  // This one has nothing get through.
  // ***********************************************
  //driver.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

  // This one works the majority of the time.
  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);

  // This one works flawlessly, but no range (from what I read.)
  //driver.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  driver.setCADTimeout(TIMEOUT);

  driver.setFrequency(915);
}

void setup()
{
    // Disable it quickly in case it is like the ATMEGA328ps which reset the timeout period to
    // obscenely low values on power on!
    Watchdog.disable();

  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);


  /*
  rtc.begin(); // initialize RTC

  // Set the time
  rtc.setHours(current_time.hours);
  rtc.setMinutes(current_time.minutes);
  rtc.setSeconds(current_time.seconds);

  // Set the date
  rtc.setDay(current_time.day);
  rtc.setMonth(current_time.month);
  rtc.setYear(current_time.year);
  */

  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  uint32_t wdt_ms = Watchdog.enable();
  /*Serial.print(F("Watchdog timer: "));
  Serial.print(wdt_ms);
  Serial.println();
  Serial.println(F("Starting server.."));
  Serial.print(F("Hardware Serial buffer size: "));
  Serial.println(SERIAL_BUFFER_SIZE);
*/
    reset_modem();
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

    Watchdog.reset();
}

//uint8_t time_size = sizeof(current_time);
uint8_t data[sizeof(gps_data)];


// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t ack[1] = {0x55};
uint8_t len = sizeof(buf);

void loop() {
  uint8_t rssi;
  uint8_t bad_receive = 0;

    while(true) {
      if (manager.recvfromAck(buf, &len)) {
        // Should be a message for us now

          // After a call to the manager for receiving, it defaults to setting the radio to IDLE!
          // SERIOUSLY! That should be a call made by the programmer (and EXPLICTLY mentioned in the
          // docs to control your idle modes...)

          // We immediately set the mode back to receive.
          driver.setModeRx();

          // Display LED and beep the buzzer
          digitalWrite(13, HIGH);

          // Save signal strength.
          rssi = driver.lastRssi();

          // Copy buffer into data structure.
          memcpy(&gps_data, &buf, sizeof(gps_data));

          // Print it out for logging.
          Serial.print(static_cast<uint32_t>(gps_data.time), DEC);
          Serial.print(",");
          Serial.print(static_cast<float>(gps_data.battery_adc) * 0.0064453125, 2);
          Serial.print(",");
          Serial.print(gps_data.lat, 7);
          Serial.print(",");
          Serial.print(gps_data.lng, 7);
          Serial.print(",");
          Serial.println(rssi);
          /*
         Serial.println(len);
        for (uint16_t i = 0; i < len; i++) {
            Serial.print(buf[i]);
        }
        Serial.println();*/

        if (gps_data.battery_adc == 0) {
                // Should always get a battery measurement, even if no GPS.
                // We got a blank (why?)
                // If we get multiple from a hung radio the only recourse is resetting the controller
                // (eg, no radio reset pin wired.)
                bad_receive++;

            if (bad_receive > 5) {
                // Not that it matters with WDT.
                bad_receive = 0;


                Serial.println(F("Battery ADC == 0."));

                // Would be nice to just reset the modem. Might need to solder a wire to it.
                reset_modem();

                // Wait for watchdog timeout to reset the entire controller
                while(true) {
                    // Do nothing. Waiting for the end.
                }
            }
        }

        // Zero buffers and data so that we can detect not receiving a transmission.
        memset(buf, 0, len);
        gps_data.time = 0;
        gps_data.battery_adc = 0;
        gps_data.lat = 0;
        gps_data.lng = 0;
        rssi = 0;

        digitalWrite(13, LOW);


      }

      Watchdog.reset();

    }

}

