/*
** Example Arduino sketch for SainSmart I2C LCD Screen 16x2
** based on https://bitbucket.org/celem/sainsmart-i2c-lcd/src/3adf8e0d2443/sainlcdtest.ino
** by
** Edward Comer
** LICENSE: GNU General Public License, version 3 (GPL-3.0)

** This example uses F Malpartida's NewLiquidCrystal library. Obtain from:
** https://bitbucket.org/fmalpartida/new-liquidcrystal

** Modified - Ian Brennan ianbren at hotmail.com 23-10-2012 to support Tutorial posted to Arduino.cc

** Written for and tested with Arduino 1.0
**
** NOTE: Tested on Arduino Uno whose I2C pins are A4==SDA, A5==SCL

*/
#include <Wire.h>
#include <LCD.h>
#include "./library/NewliquidCrystal/LiquidCrystal_I2C.h"
#include "./library/Adafruit_SI1145_Library-master/Adafruit_SI1145.h"
#include "./library/OneWire2/OneWire.h"
#include "./library/Arduino-Temperature-Control-Library/DallasTemperature.h"

Adafruit_SI1145 uv = Adafruit_SI1145();

#define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define COLS      20
#define ROWS       4
#define PAUSE    400
char msg1[] = "From: Jill";
char msg2[] = "We will meet for lunch at noon at the Twin Lakes Restaurant";

int n = 1;

LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20(&oneWire);

void setup()
{
  Serial.begin(9600);

 lcd.begin (20,4); //  <<----- My LCD was 16x2
  DS18B20.begin();
 
// Switch on the backlight
lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
//lcd.setBacklight(255);
lcd.noBacklight();

lcd.home (); // go home
//lcd.autoscroll();
// lcd.print("SainSmartI2C16x2");
 //lcd.print("Super duper long thingamabobish!!!"); 

 if (! uv.begin()) {
    Serial.println(F("Didn't find Si1145"));
    lcd.print(F("Didn't find Si1145"));
    while (1);
  }

    // "Normal" range for visible and IR gain. Read-modify-write registers.
  uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
    uv.setRegister(SI1145_PARAM_ALSIRADCMISC, IRADC_value & 0b11011111);
    uv.setRegister(SI1145_PARAM_ALSVISADCMISC, uv.getRegister(SI1145_PARAM_ALSVISADCMISC) & 0b11011111);
}

void loop()
{
  unsigned long visible = uv.readVisible();
  unsigned long infrared = uv.readIR();
  unsigned long ultraViolet = uv.readUV();
  uint8_t IRADC_value = uv.getRegister(SI1145_PARAM_ALSIRADCMISC);
  float DS18B20_temp_f{0};
  
  DS18B20.requestTemperatures(); // Send the command to get temperatures

  DS18B20_temp_f = DS18B20.getTempCByIndex(0);
  //if (visible == 0 || infrared == 0) {
    //uv.reset();
  //}
  
  lcd.clear();
  //lcd.setCursor(0, 0);
  
  lcd.print(F("Vis: "));
  lcd.print(visible);
  lcd.setCursor(0, 1);
  lcd.print(F(" IR: "));
  lcd.print(infrared);
  lcd.setCursor(0, 2);
  lcd.print(F(" UV: "));
  lcd.print(ultraViolet);
  lcd.setCursor(0, 3);
  lcd.print(DS18B20_temp_f);
  lcd.print(F("F"));

  Serial.print(F("Vis: "));
  Serial.print(visible);
  Serial.print(F(" IR: "));
  Serial.print(infrared);
  Serial.print(F(" UV: "));
  Serial.print(ultraViolet);
  Serial.print(F(" "));
  Serial.print(DS18B20_temp_f);
  Serial.println(F("F"));


  //uv.nop();
  uv.force_convert();
  delay(1000);
  
  
}

/*****
  The purpose of this function is to scroll a message across
  a line of the display. This is only called if the message
  is longer than the display is wide.
 
  Parameter list:
    char msg[]        the message to scroll
    int row           the row for scrolling
    int howLong       the length of the message
   
  Return value:
    void
*****/
void ScrollDisplay(char msg[], int row, int howLong)
{
  int i;
  int j;
  char window[COLS];  // Enough room for message + null
 
  strncpy(window, msg, COLS);
  //window[COLS + 1] = '\0';
  lcd.setCursor(0, row);        // Show first part...
  lcd.print(window);
  delay(PAUSE);

  j = COLS;
  do {
    if (j == (howLong - COLS))            // Seen everything?
      break;                              // Yep.
    memcpy(window, &msg[j], COLS);    // Slide the message down 1 char
    //window[COLS-1] = msg[j + COLS];       // Add new character at the end
    lcd.setCursor(0, row);
    lcd.print(window);
    delay(PAUSE);
    j++;
  }while (true); 
} 
