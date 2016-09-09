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
#include <LiquidCrystal_I2C.h>

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

void setup()
{
 lcd.begin (20,4); //  <<----- My LCD was 16x2

 
// Switch on the backlight
lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
lcd.setBacklight(HIGH);
lcd.home (); // go home
//lcd.autoscroll();
// lcd.print("SainSmartI2C16x2");
 //lcd.print("Super duper long thingamabobish!!!"); 
}

void loop()
{
  int len;
  len = strlen(msg1);
  if (len > COLS) {        // Truncate From details if too long
    msg1[COLS] = '\0';
  }
  lcd.setCursor(0,0);      //Start at character 4 on line 0
  lcd.print(msg1);
  len = strlen(msg2);
  if (len <= COLS) {       // Second part short enough to fit?
    lcd.setCursor(1, 0);
    lcd.print(msg2);
  } else {
    ScrollDisplay(msg2, 1, sizeof(msg2)); // Need to scroll the message
  }
  delay(4000);
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
