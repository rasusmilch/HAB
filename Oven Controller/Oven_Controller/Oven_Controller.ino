
#define __AVR_ATmega328P__
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <TM1638.h>

// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

#define BAKE_ELEMENT_PIN  10
#define BROIL_ELEMENT_PIN 11
#define BUZZER_PIN        12
#define FAN_PIN           2

#define DEBOUNCE_TIME     200
#define BUTTON_TEMP_DIFF    10

// LEDs on panel are reversed. Eg, left most LED is LSB.
const byte timer_led        = B00000001;
const byte bake_led         = B00000010;
const byte broil_led        = B00000100;
const byte warm_led         = B00001000;
const byte fan_led          = B00010000;
const byte preheat_led      = B00100000;
const byte oven_element_led = B01000000;

const byte down_key   = B10000000;
const byte up_key     = B01000000;
const byte enter_key  = B00100000;
const byte menu_key   = B00010000;
const byte clean_key  = B00001000;
const byte broil_key  = B00000100;
const byte bake_key   = B00000010;
const byte timer_key  = B00000001;

// Maximum number of times thermocouple errors out from Maxim chip. Prevents spurious noise and other issues.
const byte thermocouple_error_max = 32;

// Thermocouple errors during read (broken wire, ground, etc)
unsigned int thermocouple_errors  = 0;

// Dot blink milliseconds for timer
const int dot_blink   = 500;

// Alarm delay between tones and led flashings
const int alarm_delay = 400;

// Delay between attention beeps, such as preheat done.
const int beep_delay = 10000;

// Time last beep was since power on.
unsigned long beep_previous_time = 0;
unsigned long flash_leds_previous_time = 0;

// Temperature scale. 0 is Celsius, 1 is Farhenheit
boolean temp_scale = 1;

// When to turn on oven is warm warning LED.
unsigned int warm_temp   = 140;

// Menu timeout in seconds
unsigned long timeout = 10;

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Example creating a thermocouple instance with hardware SPI (Uno/Mega only)
// on a given CS pin.
//#define MAXCS   10
//Adafruit_MAX31855 thermocouple(MAXCS);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif


// define a module on data pin 8, clock pin 9 and strobe pin 7
TM1638 module(8, 9, 7);
TM1638* panel = &module;

// Global panel led settings since there is no way to fetch them from the unit.
byte panel_leds = 0x00;

// Offset for thermocouple.
int offset = 0;

// Hysteresis for temperature, eg bounds above and below setpoint to turn elements on and off
// Fah
long hysteresis = 2L;

// Which direction are we approaching the hysteresis points from? 1 is above, 0 is below the setpoint.
boolean hysteresis_dir = 1;

// During preheat subtract this temperature (like hysteresis) to turn off broil element early to avoid overshooting setpoint due to thermal coasting.
int preheat_broil_setpoint = 50;

// Setpoint for oven temperature
// Fahrenheit
long setpoint = 0;

// Time MAX31855 was last read.
unsigned long last_read_temperature = 0L;

// How many milliseconds between MAX31855 readings.
const unsigned long read_temp_milli = 250L;

// Max time allowed between readings
const long MAX_READING_TIMEOUT = 60000L;

// Internal temperature
double internal = 0;

// Threashold for fan turn on temperature in Celsius. Based on internal MAX 31855 temp probe.
unsigned int fan_temp = 50;

// Shutdown oven if controller temp reaches this temp (most electrolytic capacitors and chips are rated to 85 C
// For industrial grade
unsigned int fan_alarm = 70;

// Shutdown the oven if internal temp exceeds 370 C (approx 700 F)
unsigned int shutdown_temp = 370;

// Timer set to zero
unsigned long timer_seconds = 0;

// Any advantage to combining booleans into a single bit flag array with masking? Probably not...
// Oven is idle
boolean bake = false;
boolean broil = false;
boolean self_clean = false;

// Timer off
boolean timer = false;

// Bake and broil elements are off.
byte bake_element = LOW;
byte broil_element = LOW;

// Buzzer is off
byte buzzer = LOW;

// For alarms, as panel leds all lit?
boolean panel_leds_alarm = LOW;

// Fan status
boolean fan = LOW;

// Array to average 8 readings together for smoothing and jitter control
double temperature[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double avg_temp = 0;
double avg_temp_c = 0;

// Preheat of oven (time until oven reaches temp)
boolean preheat = false;
boolean preheat_alarm = false;

unsigned long preheat_time;
unsigned long preheat_first_time;

// Auto-off timeout after preheat alarm goes off without user interaction.
// Milliseconds.
// 10 minutes * 60 seconds * 1000 ms.
unsigned long preheat_timeout = 10 * 60 * 1000L;

// Previous millisecond time
// Unsigned long is from 0 to 2^32 - 1.
// millis() function overflows back to zero. Beware. Number of milliseconds since power-on.
unsigned long previous_time = 0;

unsigned long counter = 0;

// Used for timer function. Hold previous time when function was called.
unsigned long previous_timer = 0;



void setup() {
  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif

  // Setup pins
  pinMode(BAKE_ELEMENT_PIN, OUTPUT);
  pinMode(BROIL_ELEMENT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  digitalWrite(BAKE_ELEMENT_PIN, LOW);
  digitalWrite(BROIL_ELEMENT_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  
  // LED panel display
  // On and max brightness
  panel->setupDisplay(true, 7);
  panel->clearDisplay();

  // Test the display and buzzer.
  panel->setDisplayToDecNumber(88888888, 0xFF, false);
  panel->setLEDs(0xFF);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(2000);

  // Clear display
  panel->clearDisplay();
  panel->setLEDs(0x00);
  digitalWrite(BUZZER_PIN, LOW);

  // Display intro
  panel->setDisplayToString("OvenCont");
  delay(500);
  panel->setDisplayToString("07-2016 ");
  delay(500);
  
  // Clear display
  panel->clearDisplay();

  
  Serial.begin(9600);
  
  Serial.println(F("Oven controller"));
  // wait for MAX chip to stabilize
  delay(500);
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
      return time2 - time1;

    } else if (time1 > time2) {
      // No, we had a millis overflow.
      // How much time has elapsed? Take max value, an unsigned long can hold a 2^32 - 1 value, so find difference
      // between max radix (2^32, since adding one would overflow to 0 and we'd lose one millisecond in the following add) 
      // and previous time, now add current time.
      return 4294967296L - time1 + time2;
      
    } else {
      // Neither greater or less then, must be equal.
      return 0;
    }

}



void sound_buzzer() {
    // Flash LEDs and beep the buzzer
    // Time since last beep
    unsigned long difference = time_diff(beep_previous_time);

    
    // Need to flash the LEDs?
    if (time_diff(flash_leds_previous_time) > alarm_delay) {

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
    if (difference < (unsigned long)alarm_delay) {
        // Yes
        buzzer = HIGH;
        //Serial.println("HIGH");
    
    } else {
        // Buzzer has sounded.
        //Serial.println("LOW");
        buzzer = LOW;
    }
    
    if (difference > (unsigned long)beep_delay) {
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



void bake_control() {
  // Control of baking element. May be useful to combine broil and bake into one function 
  // with a call to which element (or elements for self cleaning.)
  
  // Are we baking?
  if (bake == true) {
    
    // Preheating and less than preheat broil setpoint
    if (preheat == true && avg_temp < (setpoint - preheat_broil_setpoint)) {
      broil_element = HIGH;
        
    } else {
        // Over setpoint to allow broil to coast and allow bake element to continue.
        broil_element = LOW;
        //Serial.println(F("Over preheat setpoint: Broil off"));
    }
    
    /*if (avg_temp > (setpoint + hysteresis)) {
        hysteresis_dir = 1;
        
    } else if (avg_temp < (setpoint - hysteresis)) {
        hysteresis_dir = 0;
    }*/
    
    // Check if over temp. Due to thermal mass we turn the elements off below the setpoint to allow thermal
    // coasting over the setpoint.
    if ((avg_temp > (setpoint + hysteresis)) && bake_element == HIGH) {
      // Yes, over high range, turn off element
      bake_element = LOW;
      Serial.print(avg_temp);
      Serial.println(F(" Over setpoint: Bake LOW"));
      
      if (preheat == true) {
        // Done preheating
        // Turn off broiler element after preheat
        broil_element = LOW;
        preheat = false;
        preheat_alarm = true;

        // Set times.
        preheat_time = preheat_first_time = beep_previous_time = flash_leds_previous_time = millis();
                
        panel->clearDisplay();
        panel->setDisplayToString("Preheat ");

      }
      
    } else if ((avg_temp < (setpoint - hysteresis)) && bake_element == LOW) {
      // Are we baking? If so are we under temp? 
      // Yes, so turn on element
      bake_element = HIGH;

      Serial.print(avg_temp);
      Serial.println(F(" Under setpoint: Bake High"));

    // Are we self cleaning the oven so require the baking element?  
    } 
  }
   
  if (bake == false) {
      // No baking or self-cleaning, make sure element is off.   
      bake_element = LOW;
      broil_element = LOW;
  }
  


  if (preheat_alarm == true) {

    // Has the user not interacted with preheat timer soon enough before timeout?
    if (time_diff(preheat_first_time) > preheat_timeout) {
      // No user, shutdown.
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
        sound_buzzer();


      //preheat_time = millis();
      
      //panel->clearDisplay();
    }
   
  }

  //Serial.print(bake_element);
  //Serial.println(broil_element);
  
  // Write the element state to the pins
  digitalWrite(BAKE_ELEMENT_PIN, bake_element);
  digitalWrite(BROIL_ELEMENT_PIN, broil_element);

}



void broil_control() {

  // Check if over temp
  if (broil == true && avg_temp > (setpoint + hysteresis)) {
    // Yes, over high range, turn off element
    broil_element = LOW;
  }

  // Are we baking? If so are we under temp?
  if ((broil == true) && (avg_temp < (setpoint + hysteresis))) {
    // Yes, so turn on element
    broil_element = HIGH;
  }

  // Write the element state to the pin
  digitalWrite(BROIL_ELEMENT_PIN, broil_element);
  
}



void average_temp(double temp) {
  // Update and average temperature array.
  // Return averaged array

  // Initial average temp. Set it to current temperature, then add the rest of the previous values below
  avg_temp_c = temp;

  // Loop through last 7 values of FIFO buffer
  for (int i=0; i<7; i++) {

    // Move temps down one
    temperature[i] = temperature[i + 1];
    // Update average
    avg_temp_c += temperature[i];
  }

  // Insert current temp to end of the list.
  temperature[7] = temp;
  avg_temp_c /= 8;

  // Are we in Farhenheit scale?
  //if (temp_scale == 1) {
    // Yes, convert for display and use.
    avg_temp = avg_temp_c * 1.8 + 32;
  //}
}



void display_temp(long temp) {
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
    display_temp(avg_temp);

    light_panel_leds();    
    
  } 
  
  if (timer == true) {
    
    timer_control();

    light_panel_leds();
    
  } 
  
  if (preheat_alarm == false && timer == false && bake == false && broil == false && self_clean == false) {

    // We're idle... display.

    panel->setDisplayToString("Standby ");

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
    panel_leds = panel_leds | bake_led;
  }

  if (broil == true) {
    // Broiling
    panel_leds = panel_leds | broil_led;
  }

  if (timer == true) {
    // Timing
    panel_leds = panel_leds | timer_led;
  }

  if (avg_temp >= warm_temp) {
    // Oven is warm (> 60 C or 140 F)
    panel_leds = panel_leds | warm_led;
  }

  if (fan == HIGH) {
    // Oven is on self-clean cycle
    panel_leds = panel_leds | fan_led;
  }

  if (preheat == true) {
    // Are we preheating?
    panel_leds = panel_leds | preheat_led;
  }

  if (bake_element == HIGH || broil_element == HIGH) {
    // Is an oven element currently supplied power (heating?)
    panel_leds = panel_leds | oven_element_led;
  }
  
  panel->setLEDs(panel_leds);
  //Serial.println(panel_leds);
}



void timer_control() {
  // Controls count down timer.

  // Get current millisecond time.
  unsigned long current_time = millis();
  
  // Temp for scratch pad.
  unsigned long temp_time;
  
  int hours = 0;
  int minutes = 0;
  int seconds = 0;
  //Serial.print("Timer: ");
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

    // Calculate various standard times using division and modulo.
    // Times are stored in milliseconds, so adjust accordingly.
    hours = temp_time / 3600000;

    temp_time = temp_time % 3600000;
      
    minutes = temp_time / 60000;
    seconds = temp_time % 60000 / 1000;
    /*Serial.print("  ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.print(seconds);
    Serial.print("  ");
    Serial.println((unsigned long)hours * 10000 + minutes * 100 + seconds);*/

    // Display countdown time.
    panel->setDisplayPosDecNumber((unsigned long)hours * 10000 + minutes * 100 + seconds, B00010100, 2, true);

  }

  if (timer == true and timer_seconds == 0) {
    // We've arrived, start the party.
    bake = false;
    broil = false;
    preheat = false;
    
    bake_element = LOW;
    broil_element = LOW;

    // Write the element state to the pins
    digitalWrite(BAKE_ELEMENT_PIN, bake_element);
    digitalWrite(BROIL_ELEMENT_PIN, broil_element);

    timer = false;
    buzzer = true;

    panel->clearDisplay();
    // Try double 'n' to simulate better 'm' on 8 segment display.
    panel->setDisplayToString("Tinner  ");
    

    alarm();
    panel->clearDisplay();
  }
}



void alarm() {
  unsigned long previous_time = millis();
  byte panel_leds = 0xFF;
  
  // Sound buzzer if applicable
  // digitalWrite(BUZZER_PIN, buzzer);

  // Wait for button press
  byte keys = panel->getButtons();
  while (keys == 0) {
    
    // Check if need to flash leds
    
    if (millis() - previous_time > alarm_delay) {
      if (panel_leds == 0xFF) {
        panel_leds = 0;
        digitalWrite(BUZZER_PIN, LOW);
      } else {
        panel_leds = 0xFF;
        digitalWrite(BUZZER_PIN, HIGH);
      }
      
      previous_time = millis();  
    }

    // Set panel leds
    panel->setLEDs(panel_leds);
    
    // Get key panel
    keys = panel->getButtons();
  }

  // Stop alarm
  buzzer = false;
  panel_leds = 0x00;

  digitalWrite(BUZZER_PIN, LOW);
  panel->setLEDs(panel_leds);
  panel->clearDisplay();
  
  // Wait for idle panel
  while (panel->getButtons() != 0) {
    delay(1);
  }

  // Ensure no debouncing
  delay(100);
  
}



void timer_set() {
  byte keys = panel->getButtons();
  unsigned long previous_button = millis();
  unsigned long current_time = previous_button;
  //unsigned long temp_time = 0;
  unsigned long temp_timer = 0;
  boolean dot = false;
  byte display_dots = B00000101;
  
  // Clear display
  panel->clearDisplay();
  
  panel->setLEDs(timer_led);
  
  // Use temporary setpoint until a valid timer is entered and CONFIRMED by user.
  long temp_hours = timer_seconds / 3600000;
  int temp_minutes = (timer_seconds - temp_hours * 3600000) / 60000;
  //byte temp_seconds = seconds;
  
  //Serial.print("Keys: ");
  //Serial.println(keys);

  // Wait for bake key release
  while (keys == timer_key && ((current_time - previous_button) < (timeout * 1000))) {
    keys = panel->getButtons();
    current_time = millis();
  }

  // Reset timers
  previous_button = current_time = temp_timer = millis();
  
  while (keys != enter_key && keys != menu_key && keys != timer_key && ((current_time - previous_button) < (timeout * 1000))) {
    current_time = millis();
    //Serial.print("Current time: ");
    //Serial.println(current_time - previous_button);
    //Serial.print("Timer: ");
    //Serial.println(temp_hours * 100 + temp_minutes);
    
    //panel->setDisplayToDecNumber(temp_hours * 100 + temp_minutes, B00000100, false);
    if ((current_time - temp_timer) > dot_blink) {
      temp_timer = current_time;
      if (dot == true) {
        dot = false;
        display_dots = B00000101;
        
      } else {
        dot = true;
        display_dots = B00000001;
      }
    }

    panel->setDisplayPosDecNumber(temp_hours * 100 + temp_minutes, display_dots, 4, true);

    
    // Debounce buttons
    if (current_time - previous_button > DEBOUNCE_TIME) {

      // Check arrows
      if (keys == up_key) {
        previous_button = current_time;
        temp_hours++;
            
      } else if (keys == down_key) {
        previous_button = current_time;
        temp_hours--;
      }

      // Sanity checking
      if (temp_hours < 0) {
        temp_hours = 0;
      }

      if (temp_hours > 99) {
        temp_hours = 99;
      }
      
    }
    keys = panel->getButtons();
    
  }

  // Did we push enter?
  if (keys == enter_key) {
    // Yes, so now do minutes

    previous_button = current_time = millis();
    
    // First debounce enter key
    while (keys == enter_key && ((current_time - previous_button) < (timeout * 1000))) {
      keys = panel->getButtons();
      current_time = millis();
    }
  
    // Reset previous time
    previous_button = current_time;
    
    while (keys != enter_key && keys != menu_key && keys != timer_key && ((unsigned long)(current_time - previous_button) < (unsigned long)(timeout * 1000))) {
      current_time = millis();
      //Serial.print("Current time: ");
      //Serial.println(current_time - previous_button);
      //Serial.print("Timer: ");
      //Serial.println(temp_hours * 100 + temp_minutes);
      
      //panel->setDisplayToDecNumber(temp_hours * 100 + temp_minutes, B00000100, false);
      if ((current_time - temp_timer) > dot_blink) {
        temp_timer = current_time;
        if (dot == true) {
          dot = false;
          display_dots = B00000101;
          
        } else {
          dot = true;
          display_dots = B00000100;
        }
      }
  
      panel->setDisplayPosDecNumber(temp_hours * 100 + temp_minutes, display_dots, 4, true);      
  
      // Debounce buttons
      if (current_time - previous_button > DEBOUNCE_TIME) {
  
        // Check arrows
        if (keys == up_key) {
          previous_button = current_time;
          temp_minutes++;
              
        } else if (keys == down_key) {
          previous_button = current_time;
          temp_minutes--;
        }
  
        // Sanity checking
        if (temp_minutes < 0) {
          temp_minutes = 59;
        }
  
        if (temp_minutes > 59) {
          temp_minutes = 0;
        }
        
      }
      keys = panel->getButtons();
    }
  }

  if (keys == enter_key) {
    // Set timer in milliseconds
    timer_seconds = (temp_hours * 3600 + temp_minutes * 60) * 1000;

    // Enable timer
    timer = true;
    // Flash confirmation
    flash_leds(2, 250);
    previous_timer = millis();
    
  } else if (keys == menu_key) {
    // Flash confirmation
    flash_leds(4, 250);
    
  } else if (keys == timer_key) {
    // Canceling timer
    timer_seconds = 0;
    timer = false;
    flash_leds(2, 250);
    
  } else {
    // Flash fast warning
    // Timed out
    flash_leds(10, 100);
    
  }
  Serial.print("Timer: ");
  Serial.println(timer_seconds);
  //delay(2000);
  panel->clearDisplay();
}



void get_keys() {
  byte keys = panel->getButtons();
  //Serial.print("Keys: ");
  //Serial.println(keys);
  if (preheat_alarm == false) {
    if (keys == bake_key) {
      bake_temp_set();
      
    } else if (keys == timer_key) {
      timer_set();
    }
  } else {
    
    if (keys != 0) {
      preheat_alarm = false;
      buzzer = false;
      digitalWrite(BUZZER_PIN, LOW);
      
    }
  }
  
}



void bake_temp_set() {
  byte keys = panel->getButtons();
  unsigned long previous_button = millis();
  unsigned long current_time = previous_button;
  
  // Use temporary setpoint until a valid temp is entered and CONFIRMED by user.
  long temp_setpoint = setpoint;

  // Clear display
  panel->clearDisplay();

  panel->setLEDs(bake_led);
  
  //Serial.print("Keys: ");
  //Serial.println(keys);

  // Wait for bake key release
  while (keys == bake_key && ((current_time - previous_button) < (timeout * 1000))) {
    keys = panel->getButtons();
    current_time = millis();
  }

  // Reset previous time
  previous_button = current_time;
  
  while (keys != enter_key && keys != menu_key && keys != bake_key && ((unsigned long)(current_time - previous_button) < (unsigned long)(timeout * 1000))) {
    current_time = millis();
    //Serial.print("Current time: ");
    //Serial.println(current_time - previous_button);
    
    // Debounce buttons
    if (current_time - previous_button > DEBOUNCE_TIME) {
      //Serial.print("Temp setpoint: ");
      //Serial.println(temp_setpoint);
      //Serial.print("Keys: ");
      //Serial.println(keys);
      // Loop and display the temp setpoint
      display_temp(temp_setpoint);

      if (keys == up_key) {
        // Increase temp 
        temp_setpoint += BUTTON_TEMP_DIFF;
        previous_button = current_time;
      }

      if (keys == down_key) {
        // Decrease temp
        temp_setpoint -= BUTTON_TEMP_DIFF;
        previous_button = current_time;
      } 

      // Over or under reasonable values?
      if (temp_setpoint > 500) {
        temp_setpoint = 500;
      } else if (temp_setpoint < 100) {
       temp_setpoint = 100;
      }
    }
      // Short button delay
      //delay(500);
      keys = panel->getButtons();
  }


  //Serial.print("Keys: ");
  //Serial.println(keys);
  // Exiting temp loop. Check for user confirmation via enter key.
  
  if (keys == enter_key) {

    // Is the oven not in bake mode and is the NEW setpoint greater than the old setpoint AND greater than the current oven temp?
    // Bake mode checks for upping of already cooking food, which could cause an issue with annoying user with preheat alarm
    // as well as burning top of food with broiler use in preheat.
    if (bake == false && temp_setpoint > setpoint && temp_setpoint > avg_temp) {
      // We need to preheat
      preheat = true;
      
    } else {
      // Oven already in bake mode, OR
      // We had a lowering of temperature setpoint, no need to preheat
      preheat = false;
    }
    
    // Enter key was confirmed
    setpoint = temp_setpoint;

    // Ensure bake is true now.
    bake = true;
    
    // Flash leds
    flash_leds(2, 250);

  } else if (keys == bake_key) {
    // Bake key pressed, cancel baking.
    bake = false;
    preheat = false;
    preheat_alarm = false;
    setpoint = 0;
    // Flash leds
    bake_element = LOW;
    broil_element = LOW;

    digitalWrite(BAKE_ELEMENT_PIN, bake_element);
    digitalWrite(BROIL_ELEMENT_PIN, broil_element);
    
    flash_leds(2, 250);

  } else {
    // Timed out or menu/back button pressed
    // Flash fast warning
    flash_leds(10, 100);
  }

    //Serial.print(F("Current time: "));
    //Serial.println(current_time - previous_button);


  // Clear display
  panel->clearDisplay();

  
}



void check_safe_temps(double external) {
    
    //Serial.print("Internal Temp: ");
    //Serial.println(internal);
    
    // Need to turn on the fan?
    if (internal > (fan_temp + hysteresis)) {
        // Yes
        fan = HIGH;
        digitalWrite(FAN_PIN, HIGH);
    } else if (internal < (fan_temp - hysteresis)) {
        // No, nice and cool, turn off fan.
        fan = LOW;
        digitalWrite(FAN_PIN, LOW);
    }

    // Do we have an extremely high temp internally (controller temp on exterior of oven) or internal oven?
    if (internal > fan_alarm || external > shutdown_temp) {
        // Dangrously over internal controller temp or internal oven temperature, shutdown the stove!
        shutdown_oven();
        
        panel->clearDisplay();
        if (external > shutdown_temp) {
            // Notify user that oven was too hot.
            panel->setDisplayToString("OvenTemp");
        } else {
            // Notify user controller was too hot.
            panel->setDisplayToString("Brd Temp");
        }
        
        alarm();
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
    timer = false;
    
    // Not cooking so no preheat
    preheat = false;
    
    // Set elements off
    bake_element = LOW;
    broil_element = LOW;
    
    // Turn on fan in case of residual heat buildup
    fan = HIGH;
    
    // Update pins
    digitalWrite(BAKE_ELEMENT_PIN, LOW);
    digitalWrite(BROIL_ELEMENT_PIN, LOW);
    digitalWrite(FAN_PIN, HIGH);
    
}



void chirp() {
    // Chirp buzzer
    // Buzzer sounding?
    if (buzzer == LOW) {
        // No, chirp it.
        buzzer = HIGH;
        digitalWrite(BUZZER_PIN, buzzer);
        delay(100);
        buzzer = LOW;
        digitalWrite(BUZZER_PIN, buzzer);
    }
}



void loop() {
    double c;
    
    if (time_diff(last_read_temperature) > MAX_READING_TIMEOUT) {
        // Long time between readings...
        // For safety we will reset the cooking status and shutdown the oven while sounding an alarm.
        shutdown_oven();
        
        // Clear display
        panel->clearDisplay();

        // Display error message
        panel->setDisplayToString("LongTime");
            
        // Sound alarm
        alarm();
            
        Serial.println(F("Long time between temp readings!"));
    }
    
    // Time for new reading?
    if (time_diff(last_read_temperature) > read_temp_milli) {
        c = thermocouple.readCelsius();
        internal = thermocouple.readInternal();
        last_read_temperature = millis();
        
        
        
        // How are the safe temps. We place outside bad thermocouple response loop because internal temperature
        // is not dependant on bad external connection.
        check_safe_temps(avg_temp_c);
    
        // Check if MAX chip returned a bad thermocouple response.
        if (isnan(c)) {
            // Bad thermocouple response
            // Display
            Serial.print(F("Bad response from thermocouple at: "));
            Serial.print(millis());
            Serial.print(F("milliseconds. Errors are: "));
            Serial.println(thermocouple_errors);
            
            // Chirp buzzer.
            chirp();
        
            // Check number of errors has reached threashold.
            if (thermocouple_errors >= thermocouple_error_max) {
                
                // For safety we will reset the cooking status and shutdown the oven while sounding an alarm.
                shutdown_oven();
            
                // Clear display
                panel->clearDisplay();

                // Display error message
                panel->setDisplayToString("Th Error");
                
                // Sound alarm
                alarm();
                
                Serial.println(F("Something wrong with thermocouple!"));
            } else {
                // Increase thermocouple errors
                thermocouple_errors++;
            }
            
        } else {
            // No thermocouple error
            // Check if past errors to prevent underflow
            if (thermocouple_errors > 0) {
                thermocouple_errors--;
            
                // Check if underflow
            } else if (thermocouple_errors < 0) {
                thermocouple_errors = 0;
            }
        
            average_temp(c);

            
            //Serial.print("C = "); 
            //Serial.println(c);

        
        
            bake_control();
            broil_control();

        
        }
    }
    
    timer_control();
    
    display_panel();
    // Check keys
    get_keys(); 

   // light the first 4 red LEDs and the last 4 green LEDs as the buttons are pressed
   //panel->setLEDs(keys);
   
   //delay(1000);
}
