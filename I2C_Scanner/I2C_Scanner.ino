// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x3F // Blue non-addressable LCD
//#define I2C_ADDR    0x27 // Green solder-link addressable LCD
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
  Wire.begin();

//  Serial.begin(9600);
//  while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
lcd.begin (20,4); //  <<----- My LCD was 16x2

 
// Switch on the backlight
lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
lcd.setBacklight(HIGH);
lcd.home (); // go home
}


void loop()
{
  byte error, address;
  int nDevices;
  int row = 0;
  lcd.home();
  //Serial.println("Scanning...");
  lcd.print("Scanning...");
  delay(2000);
  lcd.clear();
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      //Serial.print("0x");
      lcd.setCursor(0, row);
      lcd.print(" 0d");
      if (address<16) 
        //Serial.print("0");
        lcd.print("0");
      lcd.print(address);
//      Serial.print(address,HEX);
//      Serial.println("  !");

      nDevices++;
      row++;
      delay(1000);
    }
    else if (error==4) 
    {
      lcd.print(" Unknown error at 0d");
//      Serial.print(" Unknow error at address 0x");
      if (address<16) 
        lcd.print("0");
      lcd.print(address);      
//        Serial.print("0");
//      Serial.println(address,HEX);
      delay(1000);
      row++;
    }
  if (row > 3)
    row = 0;
  }
  if (nDevices == 0)
    lcd.print("No I2C devices found");
    //Serial.println("No I2C devices found\n");
  else
    lcd.print(" Fin");
    //Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
