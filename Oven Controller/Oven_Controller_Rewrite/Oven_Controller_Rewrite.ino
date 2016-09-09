
#include "Arduino.h"
#define __AVR_ATmega328P__
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <TM1638.h>
//#include <avr/wdt.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include "LedControl.h"
#include "TimeLib.h"

// BM# = Bare metal #
// PB# = Port B #
// PC# = Port C #
// PD# = Port D #

// Define a TM1638 display/keypad module
#define TM1638_DATA     5       // PD5. BM11
#define TM1638_STRB     6       // PD6. BM12
#define TM1638_CLK      7       // PD7. BM13

#define BI_COLOR        true

// Display utilizing MAX7219. Not I2C. Partially SPI, where the MAX7221 *IS* fully SPI compatible.
/*
#define MAX_DISP_DATA   8        // PB0. BM14
#define MAX_DISP_CS     9        // PB1. BM15
#define MAX_DISP_CLK    10       // PB2. BM16
*/

// Attempt to use the same clock and data lines.
#define MAX_DISP_DATA   5        // PB0. BM14
#define MAX_DISP_CS     9        // PB1. BM15
#define MAX_DISP_CLK    7       // PB2. BM16


// Creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXCS   11              // Port B 3. Bare metal pin 17. MOSI.
#define MAXDO   12              // Port B 4. Bare metal pin 18. MISO.
#define MAXCLK  13              // Port B 5. Bare metal 19. SCK.

// Pin to toggle the external watchdog.
#define WATCHDOG_PIN    14      // A0. Low-level PC0. Bare metal 23.

// Control pins
#define BUZZER_PIN        15    // PC1. A1. BM24
#define FAN_PIN           16    // PC2. A2. BM25
#define BAKE_ELEMENT_PIN  17    // PC3. A3. BM26
#define BROIL_ELEMENT_PIN 18    // PC2. A4. BM27

// Thermistor ADC values
// 54C = 235
// 60C = 200
// 70C = 150
// 80C = 110
// adc = 1504.85 - 318.56 * ln(temp)
#define THERMISTOR_PIN      A5  // PC5. BM28

#define DEBOUNCE_TIME     250
#define BUTTON_TEMP_DIFF    10

// Time needed to read the MAX31855 thermocouple chip
#define THERMO_READ_TIME    150

#define MESSAGE_TIME        1000
#define CHIRP_TIME          1000

// How many milliseconds between MAX31855 readings.
const unsigned long read_temp_milli = 1000L;

// Max time allowed between readings
const long MAX_READING_TIMEOUT = 30000L;

// Start watchdog pin as high to signal that processor is up and running. Then pulse LOW for
// this long in MICROseconds. We've shortened it a bit from the required 1000 microseconds due to the
// in digitalWrite. Approx 56 cycles for digitalWrite a pin.
#define WATCHDOG_LOW     996

// Oven modes
#define STANDBY     0
#define TIMER       1
#define BAKE        2
#define BROIL       3
#define SETTINGS    4
#define CLOCK       5


// LEDs on panel are reversed. Eg, left most LED is LSB.
const byte timer_led        = B00000001;
const byte bake_led         = B00000010;
const byte broil_led        = B00000100;
const byte warm_led         = B00001000;
const byte fan_led          = B00010000;
const byte preheat_led      = B00100000;
const byte bake_element_led = B01000000;
const byte broil_element_led= B10000000;

const byte up_key     = B10000000;
const byte down_key   = B01000000;
const byte enter_key  = B00100000;
const byte menu_key   = B00010000;
const byte clock_key  = B00001000;
const byte broil_key  = B00000100;
const byte bake_key   = B00000010;
const byte timer_key  = B00000001;

// Brightness of panels, from 0 - 7.
const byte BRIGHTNESS = 5;

// Number of samples per thermistor reading to average out noise.
const byte NUM_SAMPLES = 10;

volatile unsigned long loop_time = 0;
volatile unsigned long max_loop_time = 0;
volatile unsigned long min_loop_time = 0;

// Maximum number of times thermocouple errors out from Maxim chip. Prevents spurious noise and other issues.
const byte thermocouple_error_max = 4;

// Thermocouple errors during read (broken wire, ground, etc)
volatile unsigned int thermocouple_errors  = 0;

// Dot blink milliseconds for timer
const int dot_blink   = 500;

// Alarm delay between tones and led flashings
const int alarm_delay = 250;

// Length of beep
const int alarm_beep = 1000;

// Length of preheat beep
const int preheat_beep = 500L;

// Delay between attention beeps when preheat done.
const int preheat_beep_delay = 10000L;

// Watchdog variable
volatile int watchdog = 0;

// Time last beep was since power on.
volatile unsigned long beep_previous_time = 0;
volatile unsigned long flash_leds_previous_time = 0;

// Temperature scale. 0 is Celsius, 1 is Farhenheit
bool temp_scale = 1;

// When to turn on oven is warm warning LED.
unsigned int warm_temp = 140;

// Menu timeout in seconds
unsigned long timeout = 10;

// Clock variables during setting
int clock_hours = 0;
int clock_minutes = 0;
int clock_month = 0;
int clock_day = 0;
int clock_year = 0;

bool clock_is_set = false;


#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif


TM1638 module(TM1638_DATA, TM1638_CLK, TM1638_STRB);
TM1638* panel = &module;

// Global panel led settings since there is no way to fetch them from the unit.
word panel_leds = 0;

// Offset for thermocouple.
int offset = 0;

// Hysteresis for temperature, eg bounds above and below setpoint to turn elements on and off
// Fah
long hysteresis = 1L;

// Which direction are we approaching the hysteresis points from? 1 is above, 0 is below the setpoint.
bool hysteresis_dir = 1;

// During preheat subtract this temperature (like hysteresis) to turn off broil element early to avoid overshooting setpoint due to thermal coasting.
int preheat_broil_setpoint = 60;

// Number of heating cycles
byte heating_cycle = 0;

// Every N cycles use the broil element instead of bake element to hopefully even out heating of food.
byte broil_cycles = 3;

// Oven mode
byte oven_mode = STANDBY;

// Temp oven mode during temp setting
byte oven_mode_temp = STANDBY;

// Key used for canceling in setup routines.
byte CANCEL_KEY = 0;

// Setpoint for oven temperature
// Fahrenheit
long setpoint = 0;

// Time MAX31855 was last read.
unsigned long last_read_temperature = 0L;

// Threashold for fan turn on temperature in Celsius. Based on internal MAX 31855 temp probe.
const unsigned int fan_temp = 50;
const int fan_temp_therm_hi = 258;
const int fan_temp_therm_lo = 292;

// Shutdown oven if controller temp reaches this temp (most electrolytic capacitors and chips are rated to 85 C for industrial grade
const unsigned int fan_alarm = 70;
const int fan_alarm_therm_hi = 129;
const int fan_alarm_therm_lo = 175;

// Shutdown the oven if internal temp exceeds 370 C (approx 700 F)
const unsigned int shutdown_temp = 370;

// Timer set to zero
volatile unsigned long timer_seconds = 0;

// Timer if no time is set
volatile unsigned long mand_timer = 0;

// mandatory timer if no timer is set to turn off oven.
const unsigned long DEFAULT_MAN_TIMER = 20 * 60 * 1000L;

// Are we using a mandatory timer?
bool man_timer_enable = false;

// Time since a button was last pressed.
volatile unsigned long button_time = 0;

// Last button pressed for comparious for debouncing.
byte last_button = 0;

// Byte value for which menu we are currently in.
// 0 - None
// 1 - Timer
// 2 - Bake
// 3 - Broil
// 4 - Settings
byte menu = STANDBY;

// Used to control certain sub menus with multiple parts, such as the timer setting with hour and minutes
// Eg.
// 0 - Not in use
// 1 - Setting hours
// 2 - Setting minutes
byte sub_menu = 0;

// Use temporary setpoint until a valid temp is entered and CONFIRMED by user.
long temp_setpoint = 0;

// Any advantage to combining bools into a single bit flag array with masking?
// Update. AVR (or Arduinos) use a byte to store bools, so memory crunch could benefit from
// Bit packing with masking, at the expense of CPU time.
// Oven is idle
bool bake = false;
bool broil = false;
bool self_clean = false;

// Timer off
bool timer = false;

// Bake and broil elements are off.
bool bake_element = LOW;
bool broil_element = LOW;

// Buzzer is off
bool buzzer = LOW;

// For alarms, as panel leds all lit?
bool panel_leds_alarm = LOW;

// Are we in alarm mode?
bool alarm_active = false;

// Fan status
bool fan = LOW;

// Flags for determining if we are currently setting a function
bool bake_setting = false;
bool broil_setting = false;
bool timer_setting = false;

// Array to find the median of X readings together for smoothing and jitter control
#define TEMP_ARRAY_SIZE 9
double temperature[TEMP_ARRAY_SIZE] = {0};

// Internal temperature
double internal_array[TEMP_ARRAY_SIZE] = {0};

// Array to hold thermistor readings for filter.
// Init thermistor array to larger value to prevent alarm on boot
unsigned int thermistor_array[TEMP_ARRAY_SIZE] = {1023};
unsigned int thermistor = 0;

// Single internal, most likely median filter.
double internal = 0;

// Average oven temp
double avg_temp = 0;
double avg_temp_c = 0;

const byte thermocouple_readings = 1;

// Preheat of oven (time until oven reaches temp)
bool preheat = false;
bool preheat_alarm = false;

unsigned long preheat_time;
unsigned long preheat_first_time;

// Auto-off timeout after preheat alarm goes off without user interaction.
// Milliseconds.
// 10 minutes * 60 seconds * 1000 ms.
unsigned long preheat_timeout = 10 * 60 * 1000L;

// Previous millisecond time
// Unsigned long is from 0 to 2^32 - 1.
// millis() function overflows back to zero. Beware. Number of milliseconds since power-on.
volatile unsigned long previous_time = 0;

// Time of previous alarm beep/flash
volatile unsigned long previous_alarm_time = 0;

unsigned long counter = 0;

// Used for timer function. Hold previous time when function was called.
volatile unsigned long previous_timer = 0;

// Derivative direction. Do we have an increasing (1) or decreasing (0) slope for temperature.
bool temp_direction = 1;

// Previous temp for slope check.
int previous_temp = 0;

// Time for blicking dot on timer
unsigned long temp_timer = 0;

// Temp hours and minutes in timer setup routine.
unsigned long temp_hours = 0;
unsigned long temp_minutes = 0;

// Whether a dot is blinking (false = off = 0)
bool dot = false;

// Which decimal dots are lit?
byte display_dots = B00000101;

bool do_chirp = false;
bool blocking_message = false;
unsigned long last_message = 0;

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Example creating a thermocouple instance with hardware SPI (Uno/Mega only)
// on a given CS pin.
//#define MAXCS   10
//Adafruit_MAX31855 thermocouple(MAXCS);


// MAX7219 LED Display driver
LedControl led=LedControl(MAX_DISP_DATA, MAX_DISP_CLK, MAX_DISP_CS, 1);

void display_MAX7219_str(const char message[8], byte display = 0) {
    // Print out a string to the led auxillary display with address of display (0, 1, 2, ..., 8)
    int i = 0;

    for (i = 0; i < 8; i++) {
        led.setChar(display, i, message[7 - i], false);
    }

}



void shutdown_oven() {
    // Shutdown the oven!
    // Reset cooking status
    bake = false;
    broil = false;

    // Cleaning is off
    self_clean = false;

    // Timer as well
    //timer = false;

    // Not cooking so no preheat
    preheat = false;
    preheat_alarm = false;

    // Set elements off
    bake_element = LOW;
    broil_element = LOW;

    oven_mode = STANDBY;

    // Turn on fan in case of residual heat buildup and lockup
    fan = HIGH;

    buzzer = LOW;

    // Make sure that registers are flushed and optimizations don't fiddle with our pins.
    _MemoryBarrier();

    // Update pins.
    digitalWrite(BAKE_ELEMENT_PIN, bake_element);
    digitalWrite(BROIL_ELEMENT_PIN, broil_element);
    digitalWrite(FAN_PIN, fan);
    digitalWrite(BUZZER_PIN, buzzer);

    // Make sure that registers are flushed and optimizations don't fiddle with our pins.
    _MemoryBarrier();

}



void test_dog(int expected, int new_value, const __FlashStringHelper* message) {
    // Test the watchdog variable.

    // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
    _MemoryBarrier();

    if (watchdog == expected) {
        watchdog = new_value;

        // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
        _MemoryBarrier();
    } else {
        // Don't know what is going on, so attempt to safetly shutdown and wait for reset.
        shutdown_oven();

        Serial.println(F("Watchdog failed, waiting for the end..."));
        Serial.println(message);
        Serial.println(watchdog);


        while(true) {
            // Something is wrong. Wait for reset.
        }
    }


}



void setup() {
    test_dog(0, 50, F("Setup begin"));
    int i = 0;
  //wdt_reset();
  //wdt_enable(WDTO_8S);
  // Disable watchdog timer
  //wdt_disable();

  // Setup pins
  pinMode(BAKE_ELEMENT_PIN, OUTPUT);
  pinMode(BROIL_ELEMENT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(WATCHDOG_PIN, OUTPUT);

  digitalWrite(BAKE_ELEMENT_PIN, LOW);
  digitalWrite(BROIL_ELEMENT_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Start watchdog pin HIGH.
  digitalWrite(WATCHDOG_PIN, HIGH);

  // Start fan immediately, in case of being reset in a dangorous high temp situation.
  // If internal temps dictate, fan will turn off in main loop.
  fan = HIGH;
  digitalWrite(FAN_PIN, HIGH);


  pinMode(MAX_DISP_CLK, OUTPUT);
  pinMode(MAX_DISP_CS, OUTPUT);
  pinMode(MAX_DISP_DATA, OUTPUT);

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  led.shutdown(0,false);

  /* Set the brightness to twice the value, since range is 0 - 15. */
  if (BRIGHTNESS > 7) {
      // Over maximum, so simply limit it to max.
    led.setIntensity(0, 15);
  } else {
      led.setIntensity(0, 2 * BRIGHTNESS);
  }

  /* and clear the display */
  led.clearDisplay(0);

  // LED panel display
  // On and brightness level
  panel->setupDisplay(true, BRIGHTNESS);
  panel->clearDisplay();

    // Display intro
    display_MAX7219_str("Oven    ");
    panel->setDisplayToString("Control ");
    delay(2000);

    display_MAX7219_str("v0.9    ");
    panel->setDisplayToString("8-2016  ");
    delay(2000);

    led.clearDisplay(0);
    panel->setDisplayToString("Testing ");
    display_MAX7219_str("Testing ");
    delay(2000);



  // Test the display and buzzer.
  panel->setDisplayToDecNumber(88888888, 0xFF, false);
  panel->setLEDs(0xFF);
  for (i=0; i < 8; i++) {
    led.setDigit(0, i, 8, true);
  }

  digitalWrite(BUZZER_PIN, HIGH);
  delay(2000);

  // Prime the analog capacitor by reading now.
  analogRead(THERMISTOR_PIN);

  // Read current value into array to prevent boot alarm
  unsigned int therm_prime = analogRead(THERMISTOR_PIN);
  for (unsigned int i{0}; i < TEMP_ARRAY_SIZE; i++) {
    thermistor_array[i] = therm_prime;
  }

  thermistor = therm_prime;

  // Clear display
  panel->clearDisplay();
  panel->setLEDs(0x00);
  led.clearDisplay(0);

  // Buzzer
  digitalWrite(BUZZER_PIN, LOW);
  //digitalWrite(FAN_PIN, LOW);

  Serial.begin(9600);

  Serial.println(F("Oven controller"));
  // wait for MAX chip to stabilize
  delay(500);

  //setTime(12, 42, 0, 8, 21, 2016);

  // Enable watchdog timer for 8 seconds until debugged more throughly.
  //wdt_enable(WDTO_8S);

  test_dog(50, 100, F("Setup, end"));


}



int read_thermistor() {
    // Read thermistor value and average number of samples together.
    byte i;
    int average = 0;

    for (i = 0; i < NUM_SAMPLES; i++) {
        average += analogRead(THERMISTOR_PIN);
        //delay(1);
    }
    /*
    Serial.print(F("Thermistor reading: "));
    Serial.println(average / NUM_SAMPLES);
    */
    return (average / NUM_SAMPLES);
}



void flash_leds(int loops, int delay_time) {
// Flash panel LEDS for loop times with delay between on and off.
  for(int i=0; i < loops; i++) {
    panel->setLEDs(0xFF);
    delay(delay_time);
    panel->setLEDs(0x00);
    delay(delay_time);
  }

  // Restore leds to panel
  panel->setLEDs(panel_leds);

}



unsigned long time_diff(unsigned long time1, unsigned long time2 = millis()) {
  // Given past time value in time1 and possible current time value in time2, find millisecond difference
  // taking into account possible overflow and wrapping of millisecond function.

  // Is the current time greater then the previous time?
    if (time2 > time1) {
      // Yes, all is normal
      return (unsigned long)(time2 - time1);

    } else if (time1 > time2) {
      // No, we had a millis overflow.
      // How much time has elapsed? Take max value, an unsigned long can hold a 2^32 - 1 value, so find difference
      // between max radix (2^32, since adding one would overflow to 0 and we'd lose one millisecond in the following add)
      // and previous time, now add current time.
      return (unsigned long)(4294967296L - time1 + time2);

    } else {
      // Neither greater or less then, must be equal.
      return (unsigned long)0;
    }

}



void sound_buzzer(unsigned long buzzer_length, unsigned long buzzer_delay) {
    // Flash LEDs and beep the buzzer
    // Buzzer_length is time to sound buzzer and time between LED flashing.
    // Buzzer_delay is time between beeps.

    // Time since last beep
    unsigned long difference = time_diff(beep_previous_time);


    // Need to flash the LEDs?
    if (time_diff(flash_leds_previous_time) > buzzer_length) {

        // We need to flash the LEDs.
      if (panel_leds_alarm == HIGH) {
        // Already on, turn off.
        panel_leds = 0x00;
        panel_leds_alarm = LOW;
        //Serial.println("LOW");

      } else {
        // Turn all LEDs on
        panel_leds = 0xFF;
        panel_leds_alarm = HIGH;
        //Serial.println("HIGH");
      }

      // Reset flash time
      flash_leds_previous_time = millis();

    }

    // Is it time to produce a short beep?
    if (difference < (unsigned long)buzzer_length) {
        // Yes
        buzzer = HIGH;
        //Serial.println("HIGH");

    } else {
        // Buzzer has sounded.
        //Serial.println("LOW");
        buzzer = LOW;
    }

    if (difference > (unsigned long)buzzer_delay) {
        // Time to reset the beeper timer
        beep_previous_time = millis();
    }

    /*Serial.print("Difference: ");
    Serial.print(difference);
    Serial.print(" ");
    Serial.println(panel_leds_alarm);*/

    // Write the state to the LEDs and buzzer
    digitalWrite(BUZZER_PIN, buzzer);
    panel->setLEDs(panel_leds);


}



void oven_cook_control() {
  // Control of baking element. May be useful to combine broil and bake into one function
  // with a call to which element (or elements for self cleaning.)
//Serial.println(F("Bake control"));
  // Check watchdog.
  test_dog(234, 300, F("Oven_cook_control, begin"));

  // Are we baking or broiling?
  if (bake == true || broil == true) {

    // Check if temperature is climbing since previous check
    if ((int)avg_temp > previous_temp) {
        // Temp is increasing. Are we at an inflection point (previously decreasing temp, now increasing)
        if (temp_direction == 0) {
            // We were decreasing, now update derivative direction
            temp_direction = 1;

            // Output data
            Serial.print(F("Min temp: "));
            Serial.println(previous_temp);
        }

    } else if ((int)avg_temp < previous_temp) {
        // Are we decreasing?
        if (temp_direction == 1) {
            // We WERE increasing, so update derivative direction
            temp_direction = 0;

            // Print
            Serial.print(F("Max temp: "));
            Serial.println(previous_temp);
        }
    }

    // Store current temp for later.
    previous_temp = avg_temp;

    // Preheating and less than preheat broil setpoint
    if (preheat == true && avg_temp < (setpoint - preheat_broil_setpoint)) {
      broil_element = HIGH;

      if (broil == false && bake == true) {
          // Only turn on bake element if oven is in bake mode. Broiling should only turn on broil element.
        bake_element = HIGH;
      }

    } else {
        // Over setpoint to allow to coast based on functioning element (bake or broil
        if (bake == true) {
            // Baking, so turn off broil
            broil_element = LOW;
        } else if (broil == true) {
            // Broil, so turn off bake
            bake_element = LOW;
        }
        //Serial.println(F("Over preheat setpoint: Broil off"));
    }

    /*if (avg_temp > (setpoint + hysteresis)) {
        hysteresis_dir = 1;

    } else if (avg_temp < (setpoint - hysteresis)) {
        hysteresis_dir = 0;
    }*/

    // Check if over temp. Due to thermal mass we turn the elements off below the setpoint to allow thermal
    // coasting over the setpoint.
    if (((int)avg_temp > (int)(setpoint + hysteresis)) && (bake_element == HIGH || broil_element == HIGH)) {
      // Yes, over high range, turn off element
      bake_element = LOW;
      broil_element = LOW;
      //Serial.print(avg_temp);
      //Serial.println(F(" Over setpoint: Elements LOW"));

      if (preheat == true) {
        // Done preheating

        // No longer preheating
        preheat = false;

        // Flag alarm
        preheat_alarm = true;

        if (man_timer_enable == true) {
            // We need to reset the mandatory timer.
            timer_seconds = DEFAULT_MAN_TIMER;
        }

        // Reset heating cycles
        heating_cycle = 0;

        // Set times.
        preheat_time = preheat_first_time = beep_previous_time = flash_leds_previous_time = millis();

        panel->clearDisplay();
        panel->setDisplayToString("Preheat ");

      }

    } else if (((int)avg_temp < (int)(setpoint - hysteresis)) && (bake_element == LOW && broil_element == LOW)) {
        // Are we baking? If so are we under temp?
        // Yes, so turn on element

        if (bake == true) {
            // Increase heating cycles
            heating_cycle++;

            // Need to turn on broil element in addition to bake?
            if (heating_cycle >= broil_cycles && broil == false) {
                // Yes
                // Reset heating_cycle.
                heating_cycle = 0;

                // Broil on
                broil_element = HIGH;
                //Serial.print(avg_temp);
                //Serial.println(F(" Under BAKE setpoint: Broil HIGH"));

            } else {
                // No, normal bake or broil
                bake_element = HIGH;
                //Serial.print(avg_temp);
                //Serial.println(F(" Under BAKE setpoint: Bake HIGH"));
            }

        } else if (broil == true) {
            // Not baking, broiling. Turn on.
            broil_element = HIGH;
        }
    }
  }

  if (bake != true && broil != true) {
      // No baking or self-cleaning, make sure element is off.
      bake_element = LOW;
      broil_element = LOW;
  }



  if (preheat_alarm == true) {

    // Has the user not interacted with preheat timer soon enough before timeout?
    if (time_diff(preheat_first_time) > preheat_timeout) {
      // No user, shutdown.
        shutdown_oven();
      bake_element = LOW;
      broil_element = LOW;

      bake = false;
      broil = false;
      preheat_alarm = false;
      preheat = false;
      buzzer = LOW;
      digitalWrite(BUZZER_PIN, buzzer);

    } else {
        // Preheat timeout was not exceeded, so beep and flash and make a fuss.
        sound_buzzer(preheat_beep, preheat_beep_delay);


      //preheat_time = millis();

      //panel->clearDisplay();
    }

  }

  //Serial.print(bake_element);
  //Serial.println(broil_element);

  // Write the element state to the pins
  digitalWrite(BAKE_ELEMENT_PIN, bake_element);
  digitalWrite(BROIL_ELEMENT_PIN, broil_element);

    // Check watchdog. Skip to 456 because we've removed obsolete broil function and combined into one cooking function.
  test_dog(300, 456, F("Oven cook control, end"));

}



void average_temp(double temp) {
  // Update and average temperature array.
  // Return averaged array

  // Initial average temp. Set it to current temperature, then add the rest of the previous values below
  avg_temp_c = temp;

  // Loop through last 7 values of FIFO buffer
  /*
  for (int i=0; i<7; i++) {

    // Move temps down one
    temperature[i] = temperature[i + 1];
    // Update average
    avg_temp_c += temperature[i];
  }

  // Insert current temp to end of the list.
  temperature[7] = temp;
  avg_temp_c /= 8;

  */

  // Are we in Farhenheit scale?
  //if (temp_scale == 1) {
    // Yes, convert for display and use.
    avg_temp = avg_temp_c * 1.8 + 32;
  //}
}

void process_temps(double oven, double board, unsigned int thermistor_raw) {
    // Process raw temp values.

    double sorted_oven[TEMP_ARRAY_SIZE] = {0};
    double sorted_board[TEMP_ARRAY_SIZE] = {0};
    unsigned int sorted_thermistor[TEMP_ARRAY_SIZE] = {0};

    // Make room to nsert new value at end of array
    for (int i = 0; i < TEMP_ARRAY_SIZE - 1; i++) {

        // Move temps down one
        temperature[i] = temperature[i + 1];
        internal_array[i] = internal_array[i + 1];
        thermistor_array[i] = thermistor_array[i + 1];
  }

    // Insert newest value
    temperature[TEMP_ARRAY_SIZE - 1] = oven;
    internal_array[TEMP_ARRAY_SIZE - 1] = board;
    thermistor_array[TEMP_ARRAY_SIZE - 1] = thermistor_raw;

    // Unsorted lists are copied
    for (unsigned int i = 0; i < TEMP_ARRAY_SIZE; i++) {
        sorted_oven[i] = temperature[i];
        sorted_board[i] = internal_array[i];
        sorted_thermistor[i] = thermistor_array[i];
    }

    // Sort lists
    sort_array(sorted_oven, TEMP_ARRAY_SIZE);
    sort_array(sorted_board, TEMP_ARRAY_SIZE);
    sort_array(sorted_thermistor, TEMP_ARRAY_SIZE);

    // Prepare median for their new home.
    avg_temp_c = median(sorted_oven, TEMP_ARRAY_SIZE);
    avg_temp = avg_temp_c * 1.8 + 32;
    internal = median(sorted_board, TEMP_ARRAY_SIZE);
    thermistor = median(sorted_thermistor, TEMP_ARRAY_SIZE);

    Serial.print(F("sorted_oven: "));

    for (unsigned int i = 0; i < TEMP_ARRAY_SIZE; i++) {
        Serial.print(sorted_oven[i]);
        Serial.print(F(" "));
    }
    Serial.println();

    Serial.print(F("sorted_board: "));

    for (unsigned int i = 0; i < TEMP_ARRAY_SIZE; i++) {
        Serial.print(sorted_board[i]);
        Serial.print(F(" "));
    }
    Serial.println();

    Serial.print(F("sorted_thermistor: "));

    for (unsigned int i = 0; i < TEMP_ARRAY_SIZE; i++) {
        Serial.print(sorted_thermistor[i]);
        Serial.print(F(" "));
    }
    Serial.println();

    Serial.print(F("Avg_temp_c: "));
    Serial.print(avg_temp_c);
    Serial.print(F(" internal: "));
    Serial.print(internal);
    Serial.print(F(" thermistor: "));
    Serial.println(thermistor);
}


void display_acc_temp() {
    // Display the setpoint and oven temperature on the accessory display.
    int hundreds, tens, ones, remainder = 0;

    // Use decimal points to differentiate between setpoint and internal temp.
    bool decimal = false;

    // Temp to display
    int temp_to_display = 0;

    if (alarm_active == false && blocking_message == false && preheat_alarm == false && bake == false && broil == false && internal < 38) {
        temp_to_display = setpoint;
    } else {
        temp_to_display = (internal * 1.8) + 32;
        decimal = true;
    }

    // Compute first temp
    ones = temp_to_display % 10;
    remainder = temp_to_display / 10;
    tens = remainder % 10;
    hundreds = remainder / 10;


    // The MAX72XX is in power-saving mode on startup,
    // we have to do a wakeup call

    led.shutdown(0,false);

    // Set the brightness to twice the value, since range is 0 - 15.
    if (BRIGHTNESS > 7) {
        // Over maximum, so simply limit it to max.
        led.setIntensity(0, 15);
    } else {
        led.setIntensity(0, 2 * BRIGHTNESS);
    }

    // and clear the display
    led.clearDisplay(0);

    // Write temp digits
    led.setDigit(0, 7, (byte)hundreds, decimal);
    led.setDigit(0, 6, (byte)tens, decimal);
    led.setDigit(0, 5, (byte)ones, decimal);

    // Blanks
    led.setChar(0, 4, ' ', false);
    led.setChar(0, 3, ' ', false);

    // Actual oven temperature
    ones = (int)avg_temp % 10;
    remainder = (int)avg_temp / 10;
    tens = remainder % 10;
    hundreds = remainder / 10;

    // Display oven temp.
    led.setDigit(0, 2, (byte)hundreds, false);
    led.setDigit(0, 1, (byte)tens, false);
    led.setDigit(0, 0, (byte)ones, false);
}



void display_temp(int temp) {
  // Display temperature with appropriate scale sign. Assumes correct temperature is passed.
  char s[8];

  // Is temp less then 1000?
  if (temp < 1000) {
    // Yes, pad the temp with leading zeros to right shift, including the "F"

    // Do we need a F?
    if (temp_scale == 1) {
      // Yes, need F
      sprintf(s, "    %3dF", temp);
    } else {
      // No, need C
      sprintf(s, "    %3dC", temp);
    }
  } else {
    // Do we need a F?
    if (temp_scale == 1) {
      // Yes, need F

      // No, need four decimal places so only shift 3 places.
      sprintf(s, "   %4dC", temp);
    } else {
      // No, need C

      // No, need four decimal places so only shift 3 places.
      sprintf(s, "   %4dC", temp);
    }
  }
  //panel->setDisplayToDecNumber(int(c * 1.8 + 32), 0, false);
  panel->setDisplayToString(s);
  //Serial.println("Printing display");

}



void display_panel() {
  unsigned long current_time = millis();


  if ((timer == false && preheat_alarm == false) && (bake == true || broil == true || self_clean == true)) {
    //counter = 0;
    // Display the temp
    //display_temp(avg_temp);

    light_panel_leds();

  }

  if (timer == true) {

    light_panel_leds();

  }

  if (preheat_alarm == false && timer == false && bake == false && broil == false && self_clean == false && alarm_active == false && menu == STANDBY) {

    // We're idle... display clock.
    if (blocking_message == false) {
        if (clock_is_set == true) {
            clock();
        } else {
            panel->setDisplayToString("Standby ");
        }
    }
    //panel->setDisplayToDecNumber(thermistor, 0);
    light_panel_leds();

    /*if ((unsigned long)(current_time - previous_time) > 1000) {
      counter++;
      previous_time = current_time;

    }
    panel_leds = panel_leds | counter;
    panel->setDisplayToDecNumber(counter, 0, false);*/
  }

}



void light_panel_leds() {

  // Reset panel led status.
  panel_leds = 0;

  if (bake == true) {
    // Baking so or panel led for bake light
    #if BI_COLOR == true
        panel_leds = panel_leds | bake_led << 8;
    #else
        panel_leds = panel_leds | bake_led;
    #endif

  }

  if (broil == true) {
    // Broiling
    #if BI_COLOR == true
        panel_leds = panel_leds | broil_led << 8;
    #else
        panel_leds = panel_leds | broil_led;
    #endif
  }

  if (timer == true) {
    // Timing
    #if BI_COLOR == true
        panel_leds = panel_leds | timer_led << 8;
    #else
        panel_leds = panel_leds | timer_led;
    #endif
  }

  if (avg_temp >= warm_temp) {
    // Oven is warm (> 60 C or 140 F)
    #if BI_COLOR == true
        panel_leds = panel_leds | warm_led << 8;
    #else
        panel_leds = panel_leds | warm_led;
    #endif
  }

  if (fan == HIGH) {
    // Fan is on
    #if BI_COLOR == true
        panel_leds = panel_leds | fan_led << 8;
    #else
        panel_leds = panel_leds | fan_led;
    #endif
  }

  if (preheat == true) {
    // Are we preheating?
    #if BI_COLOR == true
        panel_leds = panel_leds | preheat_led << 8;
    #else
        panel_leds = panel_leds | preheat_led;
    #endif
  }

  if (bake_element == HIGH) {
    // Is an oven element currently supplied power (heating?)
    #if BI_COLOR == true
        panel_leds = panel_leds | bake_element_led << 8;
    #else
        panel_leds = panel_leds | bake_element_led;
    #endif
  }

  if (broil_element == HIGH) {
    #if BI_COLOR == true
        panel_leds = panel_leds | broil_element_led << 8;
    #else
        panel_leds = panel_leds | broil_element_led;
    #endif
  }

  if (menu == STANDBY && clock_is_set == true && bake == false && broil == false && alarm_active == false && preheat == false && preheat_alarm == false && timer == false) {
        panel_leds = panel_leds | second();
  }

  panel->setLEDs(panel_leds);
  //Serial.println(panel_leds);
}



void timer_control() {
  // Controls count down timer.
    //Serial.println(F("Timer control"));
  // Check watchdog.
  test_dog(456, 500, F("Timer control, begin"));


  // Get current millisecond time.
  unsigned long current_time = millis();

  // Temp for scratch pad.
  unsigned long temp_time = 0;

  unsigned long hours = 0;
  unsigned long minutes = 0;
  unsigned long seconds = 0;
  //Serial.print("Timer control: ");
  //Serial.println(timer_seconds);

  if (timer == true) {
    if (current_time > previous_timer) {
      temp_time = current_time - previous_timer;

    } else if (previous_timer > current_time) {
      // We had a millis overflow.
      // How much time has elapsed? Take max value, an unsigned long can hold a 2^32 - 1 value, so find difference
      // between max radix (2^32, since adding one would overflow to 0 and we'd lose one millisecond in the following add)
      // and previous time, now add current time.
      temp_time = 4294967296L - previous_timer + current_time;

    }

    //Serial.print(F("After finding difference: "));
    //Serial.println(temp_time);

    if (timer_seconds > temp_time) {
      // We won't underflow
      timer_seconds -= temp_time;
    } else {
      // Too little time left, would underflow so set to zero.
      timer_seconds = 0;
    }

    // Set value for next loop
    previous_timer = current_time;

    // Reset to number of milliseconds
    temp_time = timer_seconds;

    //Serial.print(F("After Subtraction: "));
    //Serial.println(temp_time);

    // Calculate various standard times using division and modulo.
    // Times are stored in milliseconds, so adjust accordingly.
    hours = temp_time / 3600000L;

    temp_time = temp_time % 3600000L;

    //Serial.print(F("After hrs: "));
    //Serial.println(temp_time);

    minutes = temp_time / 60000L;
    seconds = (temp_time % 60000L) / 1000L;

    /*Serial.print("  ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.print(seconds);
    Serial.print("  ");
    Serial.println((unsigned long)hours * 10000 + minutes * 100 + seconds);*/


    if (menu == STANDBY && preheat_alarm == false && man_timer_enable == false) {
        // Display countdown time, but only if not within a menu, nor a mandatory timer.
        panel->setDisplayPosDecNumber((unsigned long)hours * 10000 + minutes * 100 + seconds, B00010100, 2, true);
    }

    if (timer_seconds == 0) {
        // We've arrived, start the party.
        bake = false;
        broil = false;
        preheat = false;
        preheat_alarm = false;

        bake_element = LOW;
        broil_element = LOW;

        _MemoryBarrier();
        // Write the element state to the pins
        digitalWrite(BAKE_ELEMENT_PIN, bake_element);
        digitalWrite(BROIL_ELEMENT_PIN, broil_element);

        timer = false;
        man_timer_enable = false;
        buzzer = true;

        // Set menu and other settings
        menu = oven_mode = oven_mode_temp = STANDBY;
        sub_menu = 0;

        // Reset setpoint for auxillary display
        setpoint = 0;

        panel->clearDisplay();
        panel->setDisplayToString("Timer   ");

        display_acc_temp();

        alarm_active = true;


    }
  }


  // Check watchdog.
  test_dog(500, 567, F("Timer control, end"));

}



void alarm() {
  //unsigned long previous_time = millis();
  //byte panel_leds = 0xFF;

  // Sound buzzer if applicable
  // digitalWrite(BUZZER_PIN, buzzer);

    // Check watchdog.
    test_dog(567, 600, F("Alarm, begin"));

    if (alarm_active == true) {
        // Get button press
        byte keys = panel->getButtons();

        if (keys == 0) {
            // No keys pressed, keep alarming.
            sound_buzzer(alarm_delay, alarm_beep);


        } else {
            // Button was pressed, cancel alarm
            buzzer = false;
            alarm_active = false;

            digitalWrite(BUZZER_PIN, LOW);
            panel->setLEDs(0);
            panel->clearDisplay();

            // Update last button press time.
            button_time = millis();

            // Update last button pressed
            last_button = keys;

        }
    }

    // Check watchdog.
    test_dog(600, 678, F("Alarm, end"));

}



void timer_set() {
    // Check watchdog.
    test_dog(567, 600, F("Timer set, begin"));

    // Check time since last button pressed
    if (time_diff(button_time) > (timeout * 1000L)) {
        // Long time, so cancel
        menu = 0;
        sub_menu = 0;

        // Timed out or menu/back button pressed
        // Flash fast warning
        flash_leds(10, 100);

        // Check watchdog.
        test_dog(600, 678, F("Timer set, timeout"));

        return;
    }

    // Get key
    byte keys = panel->getButtons();


    // Entering timer setting
    if (sub_menu == 0) {

        // Test for release of key
        if (keys == 0) {
            // No keys pressed, advance setting stage
            sub_menu = 1;

            // Reset keys
            last_button = keys;
            button_time = millis();

            // Clear display
            panel->clearDisplay();

            panel->setLEDs(timer_led);

            temp_timer = millis();

            dot = false;
            display_dots = B00000101;

            // Display here so that we are not constantly erasing the display. Eliminates flicker.
            panel->setDisplayToString("Hr      ");

            if (man_timer_enable == false) {
                // Use temporary setpoint until a valid timer is entered and CONFIRMED by user.
                // This assumes a timer is already set, so use that as a starting point.
                temp_hours = timer_seconds / 3600000;
                temp_minutes = (timer_seconds - temp_hours * 3600000) / 60000;
            } else {
                // Using the mandatory timer, so start at zero for user defined timer.
                temp_hours = 0;
                temp_minutes = 0;
            }
        }
    }


    if (sub_menu == 1) {


        if (time_diff(temp_timer) > dot_blink) {
            temp_timer = millis();
            if (dot == true) {
            dot = false;
            display_dots = B00000101;

            } else {
            dot = true;
            display_dots = B00000001;
            }
        }

        panel->setDisplayPosDecNumber(temp_hours * 100 + temp_minutes, display_dots, 4, true);

        // Check if cancel button has been pressed and valid button time.
        if (keys == timer_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Cancel timer setup
            // Reset keys
            last_button = keys;
            button_time = millis();

            // Canceling timer
            timer_seconds = 0;
            timer = false;

            menu = 0;
            sub_menu = 0;

            // Flash leds
            flash_leds(2, 250);

            panel->clearDisplay();

            // Check watchdog.
            test_dog(600, 678, F("Timer set, hour cancel"));

            return;
        } else if (keys == up_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            temp_hours++;
        } else if (keys == down_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            temp_hours--;
        } else if (keys == menu_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Abort, so cancel
            menu = 0;
            sub_menu = 0;

            // Timed out or menu/back button pressed
            // Flash fast warning
            flash_leds(2, 250);

        } else if (keys == enter_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            sub_menu = 2;

            // Setup for next prompt.
            // Display here so that we are not constantly erasing the display. Eliminates flicker.
            panel->setDisplayToString("Min     ");
        }
    }

    if (sub_menu == 2) {


        if (time_diff(temp_timer) > dot_blink) {
            temp_timer = millis();
            if (dot == true) {
            dot = false;
            display_dots = B00000101;

            } else {
            dot = true;
            display_dots = B00000100;
            }
        }

        panel->setDisplayPosDecNumber(temp_hours * 100 + temp_minutes, display_dots, 4, true);

        // Check if cancel button has been pressed and valid button time.
        if (keys == timer_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Cancel timer setup
            // Reset keys
            last_button = keys;
            button_time = millis();

            menu = 0;
            sub_menu = 0;

            // Canceling timer
            timer_seconds = 0;
            timer = false;

            // Flash leds
            flash_leds(2, 250);

            panel->clearDisplay();

            // Check watchdog.
            test_dog(600, 678, F("Timer set, min cancel"));

            return;

        } else if (keys == up_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();
            temp_minutes++;

        } else if (keys == down_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();
            temp_minutes--;

        } else if (keys == menu_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Abort, so cancel
            menu = 0;
            sub_menu = 0;

            // Timed out or menu/back button pressed
            // Flash fast warning
            flash_leds(2, 250);

        } else if (keys == enter_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();

            // Set timer in milliseconds
            timer_seconds = (temp_hours * 3600L + temp_minutes * 60) * 1000L;

            // Enable timer
            timer = true;
            man_timer_enable = false;

            // Flash confirmation
            flash_leds(2, 250);

            // Set previous check time for timer_control to current time.
            // Eg. Start the clock NOW!
            previous_timer = millis();

            menu = STANDBY;
            sub_menu = 0;

            Serial.print(F("Timer: "));
            Serial.println(timer_seconds);

            panel->clearDisplay();
        }


    }

    // Sanity checking
    if (temp_minutes < 0) {
        temp_minutes = 59;
        temp_hours--;
    }

    if (temp_minutes > 59) {
        temp_minutes = 0;
        temp_hours++;
    }

    if (temp_hours < 0) {
        temp_hours = 0;
    }

    if (temp_hours > 99) {
        temp_hours = 99;
    }

    // Check watchdog.
    test_dog(600, 678, F("Timer set, end"));

}

//////////////////////////////////////////////////////////

void clock_set() {
    // Check watchdog.
    test_dog(567, 600, F("Clock set, begin"));

    // Check time since last button pressed
    if (time_diff(button_time) > (timeout * 1000L)) {
        // Long time, so cancel
        menu = 0;
        sub_menu = 0;

        // Timed out or menu/back button pressed
        // Flash fast warning
        flash_leds(10, 100);

        // Clear display
        panel->clearDisplay();
        led.clearDisplay(0);

        // Check watchdog.
        test_dog(600, 678, F("Clock set, timeout"));

        return;
    }

    // Get key
    byte keys = panel->getButtons();


    // Entering timer setting
    if (sub_menu == 0) {

        // Test for release of key
        if (keys == 0) {
            // No keys pressed, advance setting stage
            sub_menu = 1;

            // Reset keys
            last_button = keys;
            button_time = millis();

            clock_hours = hour();
            clock_minutes = minute();

            // Clear display
            panel->clearDisplay();
            led.clearDisplay(0);

            temp_timer = millis();

            dot = false;
            display_dots = B00000101;

            // Display here so that we are not constantly erasing the display. Eliminates flicker.
            //panel->setDisplayToString("Hr      ");
            display_MAX7219_str("Hour    ");
            // Use temporary setpoint until a valid timer is entered and CONFIRMED by user.

        }
    }


    if (sub_menu == 1) {


        if (time_diff(temp_timer) > dot_blink) {
            temp_timer = millis();
            if (dot == true) {
            dot = false;
            display_dots = B00000001;

            } else {
            dot = true;
            display_dots = B00000101;
            }
        }

        panel->setDisplayPosDecNumber(clock_hours * 100 + clock_minutes, display_dots, 4, true);

        // Check if cancel button has been pressed and valid button time.
        if (keys == clock_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Cancel timer setup
            // Reset keys
            last_button = keys;
            button_time = millis();


            menu = STANDBY;
            sub_menu = 0;

            // Flash leds
            flash_leds(2, 250);

            panel->clearDisplay();
            led.clearDisplay(0);

            blocking_message = false;

            // Check watchdog.
            test_dog(600, 678, F("Clock set, hour cancel"));

            return;
        } else if (keys == up_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            clock_hours++;
        } else if (keys == down_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            clock_hours--;
        } else if (keys == menu_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Abort, so cancel
            menu = STANDBY;
            sub_menu = 0;

            // Clear display
            panel->clearDisplay();
            led.clearDisplay(0);

            blocking_message = false;

            // Timed out or menu/back button pressed
            // Flash fast warning
            flash_leds(2, 250);

        } else if (keys == enter_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();
            sub_menu = 2;

            // Setup for next prompt.
            // Display here so that we are not constantly erasing the display. Eliminates flicker.
            // Clear display
            panel->clearDisplay();

            blocking_message = true;
            led.clearDisplay(0);
            display_MAX7219_str("Minutes ");

        }
    }

    if (sub_menu == 2) {


        if (time_diff(temp_timer) > dot_blink) {
            temp_timer = millis();
            if (dot == true) {
            dot = false;
            display_dots = B00000100;

            } else {
            dot = true;
            display_dots = B00000101;
            }
        }

        panel->setDisplayPosDecNumber(clock_hours * 100 + clock_minutes, display_dots, 4, true);

        // Check if cancel button has been pressed and valid button time.
        if (keys == clock_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Cancel timer setup
            // Reset keys
            last_button = keys;
            button_time = millis();

            menu = STANDBY;
            sub_menu = 0;

            panel->clearDisplay();

            blocking_message = false;

            // Flash leds
            flash_leds(2, 250);

            panel->clearDisplay();
            led.clearDisplay(0);

            // Check watchdog.
            test_dog(600, 678, F("Clock set, min cancel"));

            return;

        } else if (keys == up_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();
            clock_minutes++;

        } else if (keys == down_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();
            clock_minutes--;

        } else if (keys == menu_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Abort, so cancel
            menu = STANDBY;
            sub_menu = 0;

            // Clear display
            panel->clearDisplay();
            led.clearDisplay(0);

            blocking_message = false;

            // Timed out or menu/back button pressed
            // Flash fast warning
            flash_leds(2, 250);

        } else if (keys == enter_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();

            // Set clock
            setTime(clock_hours, clock_minutes, 0, 1, 1, 1970);
            clock_is_set = true;

            // Flash confirmation
            flash_leds(2, 250);

            // Set previous check time for timer_control to current time.
            // Eg. Start the clock NOW!
            previous_timer = millis();

            menu = STANDBY;
            sub_menu = 0;

            Serial.print(F("Clock: "));
            Serial.println(now());

            panel->clearDisplay();
            led.clearDisplay(0);

            blocking_message = false;
        }


    }

    // Sanity checking
    if (clock_minutes < 0) {
        clock_minutes = 59;
    }

    if (clock_minutes > 59) {
        clock_minutes = 0;
    }

    if (clock_hours < 0) {
        clock_hours = 23;
    }

    if (clock_hours > 23) {
        clock_hours = 0;
    }

    // Check watchdog.
    test_dog(600, 678, F("Clock set, end"));

}

void get_keys() {

    // Check watchdog
    test_dog(567, 600, F("Get keys, begin"));

    byte keys = panel->getButtons();
    //Serial.print("Keys: ");
    //Serial.println(keys);
    if (keys != 0 && time_diff(button_time) > DEBOUNCE_TIME) {

        // Update button time and key
        last_button = keys;
        button_time = millis();

        // Are we in a preheat alarm?
        if (preheat_alarm == false) {
            // No, normal keys
            if (keys == bake_key) {
                menu = BAKE;
                sub_menu = 0;

                // Set oven mode
                oven_mode_temp = BAKE;

            } else if (keys == broil_key) {
                menu = BROIL;
                sub_menu = 0;

                // Oven mode
                oven_mode_temp = BROIL;

            } else if (keys == timer_key) {
                menu = TIMER;
                sub_menu = 0;
                //timer_set();

                // Set oven mode
                oven_mode_temp = TIMER;

            } else if (keys == clock_key) {

                menu = CLOCK;
                sub_menu = 0;
                blocking_message = true;
            }

        } else {
            // Cancel preheat alarm.
            preheat_alarm = false;
            buzzer = false;
            digitalWrite(BUZZER_PIN, LOW);

            // Update display?
            if (bake == true) {
                panel->clearDisplay();
                panel->setDisplayToString("Baking  ");
            } else if (broil == true) {
                panel->clearDisplay();
                panel->setDisplayToString("Broil   ");
            }

        }
    }

    // Check watch dog
    test_dog(600, 678, F("Get keys, end"));
}



void temp_set() {

    // Check watchdog.
    test_dog(567, 600, F("Temp set, begin"));

    // Check time since last button pressed
    if (time_diff(button_time) > (timeout * 1000L) ) {
        // Long time, so cancel
        menu = STANDBY;
        sub_menu = 0;

        // Timed out or menu/back button pressed
        // Flash fast warning
        flash_leds(10, 100);

        // Clear display
        panel->clearDisplay();

        // Check watchdog.
        test_dog(600, 678, F("Temp set, timeout"));

        return;
    }



    // Get key
    byte keys = panel->getButtons();

    // Entering bake setting
    if (sub_menu == 0) {

        // Test for release of key
        if (keys == 0) {
            // No keys pressed, advance setting stage
            sub_menu = 1;

            // Reset keys
            last_button = keys;
            button_time = millis();

            // Clear display
            panel->clearDisplay();

            // Are we setting bake or broil?
            if (oven_mode_temp == BAKE) {
                // What key for canceling function?
                CANCEL_KEY = bake_key;

                // Appropriate LED
                panel->setLEDs(bake_led);

            } else if (oven_mode_temp == BROIL) {
                // What key for canceling function?
                CANCEL_KEY = broil_key;

                // Preset setpoint for 500F for broil
                temp_setpoint = 500;

                // Appropriate LED
                panel->setLEDs(broil_led);

            } else {
                // Called, but no appropriate oven mode. Wait for reset.
                shutdown_oven();

                while(true) {
                    // Wait for reset
                }
            }
        }
    }

    if (sub_menu == 1) {

        display_temp(temp_setpoint);

        // Check if cancel button has been pressed and valid button time.
        if (keys == CANCEL_KEY && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();

            // Bake key pressed, cancel baking.
            bake = false;
            broil = false;
            preheat = false;
            preheat_alarm = false;

            // Are we using a mandatory timer?
            if (man_timer_enable == true) {
                // Yes, so disable timer.
                man_timer_enable = false;
                timer = false;
            }

            CANCEL_KEY = 0;
            setpoint = 0;

            // Reset mode
            menu = oven_mode = oven_mode_temp = STANDBY;

            sub_menu = 0;

            bake_element = LOW;
            broil_element = LOW;

            digitalWrite(BAKE_ELEMENT_PIN, bake_element);
            digitalWrite(BROIL_ELEMENT_PIN, broil_element);

            // Flash leds
            flash_leds(2, 250);

            // Clear display
            panel->clearDisplay();

            // Check watchdog.
            test_dog(600, 678, F("Temp set, cancel"));

            return;

        } else if (keys == menu_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Abort, so cancel
            oven_mode_temp = menu = STANDBY;
            sub_menu = 0;

            // Timed out or menu/back button pressed
            // Flash fast warning
            flash_leds(2, 250);

            // Clear display
            panel->clearDisplay();

        } else if (keys == up_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();

            // Increase temp
            temp_setpoint += BUTTON_TEMP_DIFF;

        } else if (keys == down_key && time_diff(button_time) > DEBOUNCE_TIME) {
            // Reset keys
            last_button = keys;
            button_time = millis();

            // Increase temp
            temp_setpoint -= BUTTON_TEMP_DIFF;
        } else if (keys == enter_key && time_diff(button_time) > DEBOUNCE_TIME) {

            // Reset keys
            last_button = keys;
            button_time = millis();


            // Is the oven not in bake mode and is the NEW setpoint greater than the old setpoint AND greater than the current oven temp?
            // Bake mode checks for upping of already cooking food, which could cause an issue with annoying user with preheat alarm
            // as well as burning top of food with broiler use in preheat.
            // We never preheat in broil mode, as one may want to simply brown an item.
            if (bake == false && broil == false && temp_setpoint > avg_temp + preheat_broil_setpoint && oven_mode_temp == BAKE) {
                // We need to preheat
                preheat = true;

                if (timer == false) {
                    // Not already in a timer, so display preheat
                    panel->setDisplayToString("Preheat ");

                } else {
                    // Clear display
                    panel->clearDisplay();
                }

            } else {
            // Oven already in appropriate mode, OR
            // We had a lowering of temperature setpoint, no need to preheat
            preheat = false;


            }

            // Do we need to set a mandatory oven timer?
            if (timer == false) {

                // Yes
                timer = true;

                // Flag mandatory
                man_timer_enable = true;

                // Set default timeout
                timer_seconds = DEFAULT_MAN_TIMER;

                // Set previous check time for timer_control to current time.
                // Eg. Start the clock NOW!
                previous_timer = millis();

                Serial.print(F("Timer: "));
                Serial.println(timer_seconds);

            }

            if (oven_mode_temp == BAKE) {
                // Ensure bake is true now.
                bake = true;

                // Turn off broil
                broil = false;

                // Oven mode.
                oven_mode = BAKE;

                // Display user mode?
                if (preheat == false) {
                    panel->setDisplayToString("bake    ");
                }

            } else if (oven_mode_temp == BROIL) {
                // Ensure bake is false now.
                bake = false;

                // Turn off broil
                broil = true;

                // Oven mode.
                oven_mode = BROIL;

                // Display user mode?
                if (preheat == false) {
                    panel->setDisplayToString("broil   ");
                }

            } else {
                // Called, but no appropriate oven mode. Wait for reset.
                shutdown_oven();

                while(true) {

                    // Wait for reset
                }
            }

            // Enter key was confirmed
            setpoint = temp_setpoint;



            // Cleanly exit menu system.
            menu = STANDBY;
            sub_menu = 0;

            // Flash leds
            flash_leds(2, 250);

        }

        // Over or under reasonable values?
        if (temp_setpoint > 500) {
            temp_setpoint = 500;
        } else if (temp_setpoint < 100) {
        temp_setpoint = 100;
        }

    }

    // Check watchdog.
    test_dog(600, 678, F("Temp set, end"));
}



void check_safe_temps() {
    // Check for safe oven temperatures, both internal and external.

    //Serial.print("Internal Temp: ");
    //Serial.println(internal);

    test_dog(123, 200, F("Check temps, begin"));



    /*
    Serial.print(F("Thermistor: "));
    Serial.println(thermistor);
    Serial.println((int)(fan_temp - hysteresis));
    Serial.println((int)fan_alarm_therm_lo);
    */
    // Need to turn on the fan? Check if EITHER thermocouple internal temp, OR thermistor is too high.
    if ((internal > (fan_temp + hysteresis)) || (thermistor < fan_temp_therm_hi)) {
        // Yes
        fan = HIGH;
        digitalWrite(FAN_PIN, HIGH);
        //Serial.println(F("Fan HIGH"));
        // Check if BOTH internal thermocouple AND thermistor are cool. Guards against either one giving
        // incorrect readings (burned/shorted/etc)
    } else if ((internal < (fan_temp - hysteresis)) && (thermistor > fan_temp_therm_lo) && alarm_active == false)  {
        // No, nice and cool, turn off fan.
        fan = LOW;
        digitalWrite(FAN_PIN, LOW);
        //Serial.println(F("Fan LOW"));
    }

    // Do we have an extremely high temp internally (controller temp on exterior of oven) or internal oven?
    if (internal > fan_alarm || avg_temp_c > shutdown_temp || thermistor < fan_alarm_therm_hi) {
        // Dangrously over internal controller temp or internal oven temperature, shutdown the stove!
        shutdown_oven();

        panel->clearDisplay();
        if (avg_temp_c > shutdown_temp) {
            // Notify user that oven was too hot.
            panel->setDisplayToString("OvenTemp");
        } else {
            // Notify user controller was too hot.
            panel->setDisplayToString("Int Temp");

            // What set us off?

            if (internal > fan_alarm && thermistor < fan_alarm_therm_hi) {
                // Both MAX31855 and thermistor
                display_MAX7219_str("Both    ");
            } else if (thermistor < fan_alarm_therm_hi) {
                // Just thermistor
                display_MAX7219_str("Thermist");
            } else if (internal > fan_alarm) {
                // Just MAX31855
                display_MAX7219_str("Thermo  ");

            }
        }

        alarm_active = true;

    }

    // Check wastch dog
    test_dog(200, 234, F("Check temps, end"));

}



void chirp() {
    // Chirp buzzer
    // Buzzer sounding?
    if (do_chirp == true && time_diff(beep_previous_time) < CHIRP_TIME) {
        // No, chirp it.
        buzzer = HIGH;
        digitalWrite(BUZZER_PIN, buzzer);
    } else if (time_diff(beep_previous_time) > CHIRP_TIME) {
        buzzer = LOW;
        digitalWrite(BUZZER_PIN, buzzer);
        do_chirp = false;
    }
}



void kick_watchdog() {
    // Is it time to kick the dog?

    // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
    _MemoryBarrier();

    if (watchdog == 678) {
        watchdog = 100;

        // Make sure that registers are flushed and optimizations don't fiddle with our watchdog variable.
        _MemoryBarrier();

        // Kick the dog
        digitalWrite(WATCHDOG_PIN, LOW);
        delayMicroseconds(WATCHDOG_LOW);
        digitalWrite(WATCHDOG_PIN, HIGH);
        //wdt_reset();
        //Serial.println(F("Watchdog passes"));


    } else {
        // Don't know what is going on, so attempt to safetly shutdown and wait for reset.
        shutdown_oven();
        Serial.print(F("Watchdog failed: "));
        Serial.println(watchdog);

        while(true) {
            // Something is wrong. Wait for reset.
        }
    }
}



void clock() {
    panel->setDisplayPosDecNumber((unsigned long)hour() * 10000 + minute() * 100 + second(), B00010100, 2, true);
/*    Serial.print(hour());
    printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println(); */
    //panel_leds = second();

}



void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}



void sort_array(double *unsorted, unsigned int size_of_array) {
    // Sort array being passed pointer to array and size of array

    // Temporary place to hold values for swapping
    double temp = 0;

    // Loop through array
    for (unsigned int i = 0; i < size_of_array; i++) {
        // Loop through remaining array
        for (unsigned int j = i; j < size_of_array; j++) {
            // Test if values need to be swapped
            if (*(unsorted + i) > *(unsorted + j)) {
                // Need to be swapped
                // Load temp value
                temp = *(unsorted + j);

                // Swap values
                *(unsorted + j) = *(unsorted + i);

                // Insert temp in correct place.
                *(unsorted + i) = temp;
            }
        }
    }
}

void sort_array(unsigned int *unsorted, unsigned int size_of_array) {
    // Sort array being passed pointer to array and size of array
    // Overloaded

    // Temporary place to hold values for swapping
    unsigned int temp = 0;

    // Loop through array
    for (unsigned int i = 0; i < size_of_array; i++) {
        // Loop through remaining array
        for (unsigned int j = i; j < size_of_array; j++) {
            // Test if values need to be swapped
            if (*(unsorted + i) > *(unsorted + j)) {
                // Need to be swapped
                // Load temp value
                temp = *(unsorted + j);

                // Swap values
                *(unsorted + j) = *(unsorted + i);

                // Insert temp in correct place.
                *(unsorted + i) = temp;
            }
        }
    }
}

double median(double *sorted, unsigned int length) {
    unsigned int middle = length / 2;

    if (length % 2 == 0) {
        // Even number length, need to grab two middle ones and average.
        return static_cast<double>(*(sorted + middle) + *(sorted + middle - 1))/2;

    } else {
        // Odd, return middle value.
        return *(sorted + middle);
    }
}

unsigned int median(unsigned int *sorted, unsigned int length) {
    unsigned int middle = length / 2;

    if (length % 2 == 0) {
        // Even number length, need to grab two middle ones and average.
        return static_cast<unsigned int>(*(sorted + middle) + *(sorted + middle - 1))/2;

    } else {
        // Odd, return middle value.
        return *(sorted + middle);
    }
}

double internal_MAX31855(uint32_t v) {
    // Read internal thermocouple temperature from cold junction.
    // Takes raw MAX31855 data, returns internal temp in C

    // ignore bottom 4 bits - they're just thermocouple data
    v >>= 4;

    // pull the bottom 11 bits off
    float internal = v & 0x7FF;
    // check sign bit!
    if (v & 0x800) {
        // Convert to negative value by extending sign and casting to signed type.
        int16_t tmp = 0xF800 | (v & 0x7FF);
        internal = tmp;
    }

    internal *= 0.0625; // LSB = 0.0625 degrees
    //Serial.print("\tInternal Temp: "); Serial.println(internal);
    return internal;
}



double convert_MAX31855(uint32_t v) {
    // Converts raw MAX31855 data and returns thermocouple temp in C.

    if (v & 0x80000000) {
        // Negative value, drop the lower 18 bits and explicitly extend sign bits.
        v = 0xFFFFC000 | ((v >> 18) & 0x00003FFFF);
    }
    else {
        // Positive value, just drop the lower 18 bits.
        v >>= 18;
    }

    //Serial.println(v, BIN);

    //double centigrade = v;

    // LSB = 0.25 degrees C
    //centigrade *= 0.25;
    return (double)(v * 0.25);
}



void display_message() {
    // Display normal panel messages
    // Check we don't clobber:
    // Timer
    // Alarm
    // A setting menu
    if ((timer == false || man_timer_enable == true) && alarm_active == false && menu == STANDBY) {

        // Are we in preheat?
        if (preheat == true || preheat_alarm == true) {
            panel->setDisplayToString("Preheat ");

        } else if (bake == true) {
            // Baking?
            panel->setDisplayToString("bake    ");
        } else if (broil == true) {
            // Broiling?
            panel->setDisplayToString("broil   ");
        }
    }
}



void fetch_temps() {
    // Grab raw temps and process them.
    int32_t raw_thermo{0};
    double c{0};
    double internal_c{0};
    unsigned int raw_thermistor = read_thermistor();

    raw_thermo = thermocouple.MAX31855_spiread32();
    // Read alternative thermistor for reliability.

    last_read_temperature = millis();

    //Serial.print("0x"); Serial.println(raw_thermo, HEX);
    //Serial.print("0b"); Serial.println(raw_thermo, BIN);

    if (raw_thermo & 0x7 == true || raw_thermo == 0 || raw_thermo == 0xffffffff) {
        // uh oh, a serious problem!
        // Bad thermocouple response
        // Return lower 3 bits. Default is 0 unless fault.
        // Bit 0 is Open circuit
        // Bit 1 is Short to Ground
        // Bit 2 is Short to VCC
        Serial.print(F("Bad response from thermocouple at: "));
        Serial.print(millis());
        Serial.print(F("milliseconds. Number of errors are: "));
        Serial.println(thermocouple_errors);

        if (blocking_message == false) {
            // Block normal messages
            blocking_message = true;

            // Display error
            if (raw_thermo & 0x1) {
                Serial.println(F("Open Circuit"));

                led.clearDisplay(0);
                display_MAX7219_str("Open    ");
                panel->setDisplayToString("circuit ");
            }

            if (raw_thermo & 0x2) {
                Serial.println(F("Short to Ground"));

                led.clearDisplay(0);
                display_MAX7219_str("Short to");
                panel->setDisplayToString("ground  ");
            }

            if (raw_thermo & 0x4) {
                Serial.println(F("Short to VCC"));
                led.clearDisplay(0);
                display_MAX7219_str("Short to");
                panel->setDisplayToString("Vcc     ");
            }

            // Chirp buzzer. Set time.
            beep_previous_time = last_message = millis();
            do_chirp = true;
        }

        // Check number of errors has reached threshold.
        if (thermocouple_errors >= thermocouple_error_max) {

            // For safety we will reset the cooking status and shutdown the oven while sounding an alarm.
            shutdown_oven();

            // Clear display
            panel->clearDisplay();

            // Display error message
            led.clearDisplay(0);
            display_MAX7219_str("Thermo  ");
            panel->setDisplayToString("Error   ");

            // Sound alarm
            alarm_active = true;

            Serial.println(F("Something wrong with thermocouple!"));

        } else {

            // Increase thermocouple errors
            thermocouple_errors++;
        }
    } else {
        // No thermocouple error

        c = convert_MAX31855(raw_thermo);
        internal_c = internal_MAX31855(raw_thermo);


        // Check if past errors to prevent underflow
        if (thermocouple_errors > 0) {
            thermocouple_errors--;

        // Check if underflow
        } else if (thermocouple_errors < 0) {
            thermocouple_errors = 0;
        }

        process_temps(c, internal_c, raw_thermistor);

        //Serial.print("C = ");
        //Serial.println(c);

    }

}


void loop() {
    loop_time = millis();

    // Test watchdog successful kicking
    test_dog(100, 123, F("Loop begin"));
    watchdog = 123;

    //Serial.println(loop_time);

    if (time_diff(last_read_temperature) > MAX_READING_TIMEOUT) {
        // Long time between readings...
        // For safety we will reset the cooking status and shutdown the oven while sounding an alarm.
        shutdown_oven();

        // Clear display
        panel->clearDisplay();

        // Display error message
        led.clearDisplay(0);
        display_MAX7219_str("Thermo  ");
        panel->setDisplayToString("time out");

        // Sound alarm
        alarm_active = true;

        Serial.println(F("Long time between temp readings!"));
    }

    // How are the safe temps. We place outside bad thermocouple response loop because internal temperature
    // is not dependant on bad external connection.
    // Additionally we have a thermistor for external oven temperature for a fail safe.
    check_safe_temps();

    //Serial.println(watchdog);

    // Time for new reading?
    if (time_diff(last_read_temperature) > read_temp_milli) {

        fetch_temps();

    }

    // Check if we need to continue blocking display
    if (blocking_message == true && time_diff(last_message) > MESSAGE_TIME && menu != CLOCK) {
        blocking_message = false;
    }

    chirp();

    // Check oven functions
    oven_cook_control();

    // Timer could be used only for timing, so run outside of temperature loop.
    timer_control();

    //Serial.println(menu);

    if (alarm_active ==  true) {
        //  Currently in an active alarm, so bypass normal key functions.
        alarm();

    } else {

        // Display temperatures
        if (blocking_message == false) {
            display_acc_temp();
        }

        // Check for whether we need to get a new key function (like timer setting) or if we are already
        // in a keyboard function.
        // Check if we changing any settings
        if (menu == BAKE || menu == BROIL) {
            // Currently in a bake or broil temp setting.

            if (time_diff(button_time) < THERMO_READ_TIME - DEBOUNCE_TIME) {
                fetch_temps();
            }

            temp_set();

        } else if (menu == TIMER) {
            //  Currently in a timer setting

            if (time_diff(button_time) < THERMO_READ_TIME - DEBOUNCE_TIME) {
                fetch_temps();
            }

            timer_set();

        } else if (menu == CLOCK) {
            // Currently in clock setting

            if (time_diff(button_time) < THERMO_READ_TIME - DEBOUNCE_TIME) {
                fetch_temps();
            }

            clock_set();

        } else {
            // No setting changes

            // Update normal display
            if (blocking_message == false) {
                display_panel();
                display_message();

            }

            // Check keys
            get_keys();
        }

    }
    //Serial.println(watchdog);
    kick_watchdog();
    //Serial.println(watchdog);
    //Serial.println(millis());
    //Serial.println(fan);

    unsigned long current_loop = time_diff(loop_time);
    if (current_loop > max_loop_time) {
        max_loop_time = current_loop;
        Serial.print(F("MAX main loop timing: "));
        Serial.println(max_loop_time);
        Serial.print(F("MIN main loop timing: "));
        Serial.println(min_loop_time);
    }

    if (current_loop < min_loop_time || min_loop_time == 0) {
        min_loop_time = current_loop;
        Serial.print(F("MAX main loop timing: "));
        Serial.println(max_loop_time);
        Serial.print(F("MIN main loop timing: "));
        Serial.println(min_loop_time);
    }

}
