
/*
Example for TM1638 with an 8 digit common anode display.
The module also contains 16 buttons
The button value is returned in an unsigned integer with each bit representing a button
The keys are in a 4x4 grid (returned value):
  2     8   32    128
512  2048 8192  32768
  1     4   16     64
256  1024 4096  16384
*/

#include <TM1638.h>

// define a module on data pin 8, clock pin 9 and strobe pin 7
TM1638 module(8, 9, 7);

// declare variables / constants
 const byte segmentEncode[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111}; // for digits from 0 to 9
 uint16_t keys;  // to hold key value - each key is 1 bit in this unsigned int

void setup() {
// set up serial monitor
 Serial.begin(9600);
}

// Function to format and send digits to the display module, fixed to send to 8 digit display currently
void outputDisplay(byte dDigits[8], byte dDecimal) {
 byte values[8];
 byte vout[8];
// set values array to the segments required for each displayed digit into values array, only for decimal 0-9 (could extend for HEX)
 for (int i=0; i < 8; i++) {
   values[i] = segmentEncode[dDigits[i]];
 }

// fill output array - need to pack for common anode display - brute force method, this is for 8 digits
// digit order on display from left to right is 4,3,2,1,8,7,6,5  
 vout[0] = (values[3] & 1) | ((values[2] & 1) << 1) | ((values[1] & 1) << 2) | ((values[0] & 1) << 3) | ((values[7] & 1) << 4) | ((values[6] & 1) << 5) | ((values[5] & 1) << 6) | ((values[4] & 1) << 7);
 vout[1] = ((values[3] & 2) >> 1) | ((values[2] & 2)) | ((values[1] & 2) << 1) | ((values[0] & 2) << 2) | ((values[7] & 2) << 3) | ((values[6] & 2) << 4) | ((values[5] & 2) << 5) | ((values[4] & 2) << 6);
 vout[2] = ((values[3] & 4) >> 2) | ((values[2] & 4) >> 1) | ((values[1] & 4)) | ((values[0] & 4) << 1) | ((values[7] & 4) << 2) | ((values[6] & 4) << 3) | ((values[5] & 4) << 4) | ((values[4] & 4) << 5);
 vout[3] = ((values[3] & 8) >> 3) | ((values[2] & 8) >> 2) | ((values[1] & 8) >> 1) | ((values[0] & 8)) | ((values[7] & 8) << 1) | ((values[6] & 8) << 2) | ((values[5] & 8) << 3) | ((values[4] & 8) << 4);
 vout[4] = ((values[3] & 16) >> 4) | ((values[2] & 16) >> 3) | ((values[1] & 16) >> 2) | ((values[0] & 16) >> 1) | ((values[7] & 16)) | ((values[6] & 16) << 1) | ((values[5] & 16) << 2) | ((values[4] & 16) << 3);
 vout[5] = ((values[3] & 32) >> 5) | ((values[2] & 32) >> 4) | ((values[1] & 32) >> 3) | ((values[0] & 32) >> 2) | ((values[7] & 32) >> 1) | ((values[6] & 32)) | ((values[5] & 32) << 1) | ((values[4] & 32) << 2);
 vout[6] = ((values[3] & 64) >> 6) | ((values[2] & 64) >> 5) | ((values[1] & 64) >> 4) | ((values[0] & 64) >> 3) | ((values[7] & 64) >> 2) | ((values[6] & 64) >> 1) | ((values[5] & 64)) | ((values[4] & 64) << 1);

// To display a decimal point to the right of a digit set the value of vout[7] to
// 8, 4, 2, 1, 128, 64, 32, 16 (for digits from left to right), can set more than 1 decimal by adding the numbers
 vout[7] = dDecimal;
 
 module.setDisplay(vout);  // send to display module
}  // end outputDisplay
void loop() {
 keys = module.getButtons16(); // get a key (button)
// output key value to serial monitor
 if (keys) {
 Serial.println(keys);
 }
 
// Digits to output on 8 display digits
 byte displayDigits[8] = {0, 1, 2, 3, 4, 5, 6, 7};
// The second parameter is the decimal point - set 1 bit for each decimal displayed
 outputDisplay(displayDigits, 8);

 delay(200); // delay .2 seconds then loop
}

