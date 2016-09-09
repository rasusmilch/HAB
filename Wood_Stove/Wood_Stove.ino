#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MAX31855.h"

const int clock = 7; //SCK
const int latch = 8; //RCK 
const int data = 6;  //DIO

// Values for digits, no decimal (MSB is decimal off)
/*byte value[] ={ B11000000, // 0
                B11111001, // 1
                B10100100, // 2
                B10110000, // 3
                B10011001, // 4
                B10010010, // 5
                B10000010, // 6
                B11111000, // 7
                B10000000, // 8
                B10010000, // 9
                B11111111};// display nothing

byte negative = B10111111;

// Digit with decimal. MSB is decimal (0 is on)
byte valued[] ={ B01000000, // 0.
                 B01111001, // 1.
                 B00100100, // 2.
                 B00110000, // 3.
                 B00011001, // 4.
                 B00010010, // 5.
                 B00000010, // 6.
                 B01111000, // 7.
                 B00000000, // 8.
                 B00010000, // 9.
                 B11111111};// display nothing
                 
byte digit[] ={ B00000001, // left segment
                B00000010,
                B00000100,
                B00001000,
                B00010000,
                B00100000,
                B01000000,
                B10000000}; // right segment*/

byte ii = 0;

//int32_t c_int;

boolean sign_bit;
//double loop_celsius = -200;

byte v0, v1, v2, v3, v4, v5, v6, v7;
//byte vvalue = value[10];

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  pinMode(clock, OUTPUT);
  pinMode(latch, OUTPUT);
  pinMode(data, OUTPUT);
  
  cli();//stop interrupts
  //set timer0 interrupt at 980Hz
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; //initialize counter value to 0
  OCR0A = 255; //(must be <256) --> 16000000 / (prescaler*255) = 980 Hz
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS01) | (1 << CS00);   //prescaler = 64
  TIMSK0 |= (1 << OCIE0A);  
  sei();//allow interrupts
  
  v0 = v1 = v2 = v3 = v4 = v5 = v6 = v7 = B11111111;

  Serial.begin(9600);
  
  //Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);

}
 


ISR(TIMER0_COMPA_vect){ 
  
  byte digit[] ={ B00000001, // left segment
                  B00000010,
                  B00000100,
                  B00001000,
                  B00010000,
                  B00100000,
                  B01000000,
                  B10000000}; // right segment
  
  byte vvalue = B11111111;                
  ii++;
  if (ii==8) ii=0;

  if (ii==0) { vvalue = v0; }
  else if (ii==1) { vvalue = v1; }    
  else if (ii==2) { vvalue = v2; }
  else if (ii==3) { vvalue = v3; }
  else if (ii==4) { vvalue = v4; }
  else if (ii==5) { vvalue = v5; }
  else if (ii==6) { vvalue = v6; }
  else if (ii==7) { vvalue = v7; }

  digitalWrite(latch,LOW);
  shiftOut(data,clock,MSBFIRST,B11111111);
  shiftOut(data,clock,MSBFIRST,B11111111);
  digitalWrite(latch,HIGH); 

  digitalWrite(latch,LOW);
  shiftOut(data,clock,MSBFIRST,digit[ii]);
  shiftOut(data,clock,MSBFIRST,vvalue);
  digitalWrite(latch,HIGH);   
}



void next_digit(long& temp, int& digit) {
// Digit bit numbers, starting at bit 1 as LSB. Low output (0) is LED on.
//  1111
// 6    2
// 6    2
//  7777
// 5    3
// 5    3
//  4444  8

// Values for digits, no decimal (MSB is decimal off)
byte value[] ={ B11000000, // 0
                B11111001, // 1
                B10100100, // 2
                B10110000, // 3
                B10011001, // 4
                B10010010, // 5
                B10000010, // 6
                B11111000, // 7
                B10000000, // 8
                B10010000, // 9
                B11111111, // display nothing
                B10001110};// F

byte negative = B10111111;
  
  // Check if current temp is 0 (from division this is the end of the number then.
    if (temp != 0) {
      // Get current digit then, working from the right hand side.
      digit = temp % 10;
     
      // Get new temp, remove right hand digit.
      temp = temp / 10;
 
      // Modulus gives negative remainders if negative number. Check,
      if (digit < 0) {
        // Negative number, invert digit for array index
        digit = digit * -1;
        // Seet sign bit for the future.
        sign_bit = 1;
      }
      
      // Grab bitmap for LED digit.
      digit = value[digit];
      
    } else {
      // Temp is already zero.
      // Preload blank LED digit.
      digit = value[10];

      // Check if we still need to output a negative sign.
      if (sign_bit == 1) {
        // Temp was negative, display negative sign.
        digit = negative;
        // Reset sign bit.
        sign_bit = 0;
      }

    }
    /*Serial.print("New Temp: ");
    Serial.println(temp);
    Serial.print("New Digit: ");
    Serial.println(digit);
    Serial.print("Sign: ");
    Serial.println(sign_bit);*/
}

long correct_temp() {
    // Initialize variables.
    int i = 0; // Counter for arrays
    double internalTemp = thermocouple.readInternal(); // Read the internal temperature of the MAX31855.
    double rawTemp = thermocouple.readCelsius(); // Read the temperature of the thermocouple. This temp is compensated for cold junction temperature.
    
    //Serial.print("Raw temp: ");
    //Serial.println(rawTemp);
    double thermocoupleVoltage = 0;
    double internalVoltage = 0;
    double correctedTemp = 0;
    //long correctedInt = 0;

    // Check to make sure thermocouple is working correctly.
    if (isnan(rawTemp)) {
    //Serial.println("Something wrong with thermocouple!");
    }
    else {
        // Steps 1 & 2. Subtract cold junction temperature from the raw thermocouple temperature.
        thermocoupleVoltage = (rawTemp - internalTemp)*0.041276;  // C * mv/C = mV

        // Step 3. Calculate the cold junction equivalent thermocouple voltage.

        if (internalTemp >= 0) { // For positive temperatures use appropriate NIST coefficients
            // Coefficients and equations available from http://srdata.nist.gov/its90/download/type_k.tab

            double c[] = {-0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04, -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,  0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22, -0.121047212750E-25};

            // Count the the number of coefficients. There are 10 coefficients for positive temperatures (plus three exponential coefficients),
            // but there are 11 coefficients for negative temperatures.
            int cLength = sizeof(c) / sizeof(c[0]);

            // Exponential coefficients. Only used for positive temperatures.
            double a0 =  0.118597600000E+00;
            double a1 = -0.118343200000E-03;
            double a2 =  0.126968600000E+03;


            // From NIST: E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2), where E is the thermocouple voltage in mV and t is the temperature in degrees C.
            // In this case, E is the cold junction equivalent thermocouple voltage.
            // Alternative form: C0 + C1*internalTemp + C2*internalTemp^2 + C3*internalTemp^3 + ... + C10*internaltemp^10 + A0*e^(A1*(internalTemp - A2)^2)
            // This loop sums up the c_i t^i components.
            for (i = 0; i < cLength; i++) {
            internalVoltage += c[i] * pow(internalTemp, i);
            }
            // This section adds the a0 exp(a1 (t - a2)^2) components.
            internalVoltage += a0 * exp(a1 * pow((internalTemp - a2), 2));
        }
        else if (internalTemp < 0) { // for negative temperatures
            double c[] = {0.000000000000E+00,  0.394501280250E-01,  0.236223735980E-04, -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10, -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16, -0.198892668780E-19, -0.163226974860E-22};

            // Count the number of coefficients.
            int cLength = sizeof(c) / sizeof(c[0]);

            // Below 0 degrees Celsius, the NIST formula is simpler and has no exponential components: E = sum(i=0 to n) c_i t^i
            for (i = 0; i < cLength; i++) {
            internalVoltage += c[i] * pow(internalTemp, i) ;
            }
        }

        // Step 4. Add the cold junction equivalent thermocouple voltage calculated in step 3 to the thermocouple voltage calculated in step 2.
        double totalVoltage = thermocoupleVoltage + internalVoltage;

        // Step 5. Use the result of step 4 and the NIST voltage-to-temperature (inverse) coefficients to calculate the cold junction compensated, linearized temperature value.
        // The equation is in the form correctedTemp = d_0 + d_1*E + d_2*E^2 + ... + d_n*E^n, where E is the totalVoltage in mV and correctedTemp is in degrees C.
        // NIST uses different coefficients for different temperature subranges: (-200 to 0C), (0 to 500C) and (500 to 1372C).
        if (totalVoltage < 0) { // Temperature is between -200 and 0C.
            double d[] = {0.0000000E+00, 2.5173462E+01, -1.1662878E+00, -1.0833638E+00, -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02, -5.1920577E-04, 0.0000000E+00};

            int dLength = sizeof(d) / sizeof(d[0]);
            for (i = 0; i < dLength; i++) {
            correctedTemp += d[i] * pow(totalVoltage, i);
            }
        }
        else if (totalVoltage < 20.644) { // Temperature is between 0C and 500C.
            double d[] = {0.000000E+00, 2.508355E+01, 7.860106E-02, -2.503131E-01, 8.315270E-02, -1.228034E-02, 9.804036E-04, -4.413030E-05, 1.057734E-06, -1.052755E-08};
            int dLength = sizeof(d) / sizeof(d[0]);
            for (i = 0; i < dLength; i++) {
            correctedTemp += d[i] * pow(totalVoltage, i);
            }
        }
        else if (totalVoltage < 54.886 ) { // Temperature is between 500C and 1372C.
            double d[] = {-1.318058E+02, 4.830222E+01, -1.646031E+00, 5.464731E-02, -9.650715E-04, 8.802193E-06, -3.110810E-08, 0.000000E+00, 0.000000E+00, 0.000000E+00};
            int dLength = sizeof(d) / sizeof(d[0]);
            for (i = 0; i < dLength; i++) {
            correctedTemp += d[i] * pow(totalVoltage, i);
            }
        } else { // NIST only has data for K-type thermocouples from -200C to +1372C. If the temperature is not in that range, set temp to impossible value.
            // Error handling should be improved.
            //Serial.print("Temperature is out of range. This should never happen.");
            correctedTemp = NAN;
        } 
  }

  //Serial.print("Corrected C: ");
  //Serial.println(correctedTemp);
  //Serial.print("Difference: ");
  //Serial.println(rawTemp - correctedTemp);
  //Serial.println();
  if (correctedTemp != NAN) {
    //correctedInt = (long)(correctedTemp * 100);
    //Serial.print("Corrected C: ");
    //Serial.println(correctedInt);
    
    // Convert to a long int for serial LED output. So 24.43 C is returned as 2443.
    return (long)(correctedTemp * 100);
  } else {
    return correctedTemp;
  }
}

void loop() {
  //float c;
  long c_int;
  //c_int = 35;
  int digit = 0;
  sign_bit = 0;
  
  Serial.print("Internal Temp = ");
  Serial.println(thermocouple.readInternal() * 1.8 + 32);
  //c_int = thermocouple.readCelsius();

  c_int = correct_temp();
  Serial.print("F = ");
  Serial.println(c_int * 1.8 + 32);
//  c = c_int * 0.25 *1.8 + 32;
  //c = c_int * 0.018 + 32;
  //Serial.println(c_int);
//  c_int = c_int * 45 + 3200;

  // Convert corrected (Celsius * 100) to Fahrenheit. Conversion introduces extra decimal places, must divide by 100 to retain
  // two decimal places.
  //c_int = (c_int * 180 + 320000) / 100;
  
  // Now to zero decimal places, don't need a quarter of a degree accuracy at 1300 degrees F.
  c_int = (c_int * 180 + 320000) / 10000;

//  c_int = 0b111110;
  //Serial.print("Passed Digit: ");
  //Serial.println(digit);
  //Serial.print("Passed Temp: ");
  //Serial.println(c_int);
  
  // Parse all the digits for LED display. Could be it's own function.
  //next_digit(c_int, digit);
  
  // Check if under 1000 F. If so display a F as least significant digit.
  if (c_int < 537) {
    v7 = B10001110; // display F
  } else {
    next_digit(c_int, digit);
    v7 = digit;
  }
  
  next_digit(c_int, digit);
  
  v6 = digit;

  next_digit(c_int, digit);
  
  // Hardcoded two decimal digits. This next digits needs the decimal point masked in.
  // Now zero decimal points.
  //v5 = digit & B01111111;
  v5 = digit;
  
  next_digit(c_int, digit);

  v4 = digit;

/*  next_digit(c_int, digit);

  v3 = digit;

  next_digit(c_int, digit);

  v2 = digit;

  next_digit(c_int, digit);

  v1 = digit;

  next_digit(c_int, digit);

  v0 = digit;*/
  v3 = v2 = v1 = v0 = B11111111;

  

  //Serial.println(c, BIN);
  delay(1000);
  //while(true) { }
}
