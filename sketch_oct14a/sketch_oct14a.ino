#include <EEPROM.h>

// P H Y S I C A L   C O N S T A N T S
  const double FT2METERS      = 0.3048;     // mult. ft. to get meters (exact)
  const double KELVIN2RANKINE = 1.8;              // mult kelvins to get deg R
  const double PSF2NSM        = 47.880258;      // mult lb/sq.ft to get N/sq.m
  const double SCF2KCM        = 515.379;    // mult slugs/cu.ft to get kg/cu.m
  const double TZERO          = 288.15;      // sea level temperature, kelvins
  const double PZERO          = 101325.0;        // sea-level pressure, N/sq.m
  const double RHOZERO        = 1.225;           // sea level density, kg/cu.m
  const double AZERO          = 340.294;    // sea-level speed of sound, m/sec
  
struct {
    // Holds various configuration and memory settings.
    // User settings such as alarm thresholds, and weather the device has taken off are stored here.

    // Location alarms. These will be user settable through EEPROM in the future.
    // If under this pressure, we start the alarm for location retrieval. In Pascals
    float pressure_alarm = 82553;

    // If under this GPS altitude, we start the alarm for location retrieval. In meters.
    float altitude_alarm = 1800;

    // Did we take off from the ground already?
    bool launched = false;

    // Are we in alarm?
    bool alarm = false;

    bool decent = false;

    // Number of seconds above altitude threshold to "arm"
    uint16_t altitude_time = 120;

    // Number of minutes to assume balloon has launched or has landed.
    uint32_t launch_time = 120;
    uint32_t location_time = 240;

} configuration, default_config;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void Atmosphere(const double  alt,                  // geometric altitude, km.
    double& sigma,           // density/sea-level standard density
    double& delta,         // pressure/sea-level standard pressure
    double& theta)   // temperature/sea-level standard temperature
// Compute the temperature,density, and pressure in the standard atmosphere
// Correct to 86 km.  Only approximate thereafter.
{
  const double REARTH=6369.0;    // radius of the Earth (km)
  const double GMR = 34.163195;
  const int NTAB = 8;
  int i,j,k;

  static double htab[NTAB] = {0.0,  11.0, 20.0, 32.0, 47.0,
           51.0, 71.0, 84.852 };
  static double ttab[NTAB] = { 288.15, 216.65, 216.65, 228.65, 270.65,
            270.65, 214.65, 186.946 };
  static double ptab[NTAB] = { 1.0, 2.2336110E-1, 5.4032950E-2, 8.5666784E-3,
     1.0945601E-3, 6.6063531E-4, 3.9046834E-5, 3.68501E-6 };
  static double gtab[NTAB] = { -6.5, 0, 1.0, 2.8, 0, -2.8, -2.0, 0 };

  double h=alt*REARTH/(alt+REARTH);     //  geometric to geopotential altitude

  i=0; j=NTAB-1;  // starting values for binary search
  do
    {
      k=(i+j)/2;
      if (h < htab[k]) j=k; else i=k;
    }  while (j > i+1);

  double tgrad=gtab[i];                      // temp. gradient of local layer
  double tbase=ttab[i];                      // base temp. of local layer
  double deltah=h-htab[i];                   // height above local base
  double tlocal=tbase+tgrad*deltah;          // local temperature
  theta=tlocal/ttab[0];                                  // temperature ratio

  if (0.0 == tgrad)                                         // pressure ratio
    delta=ptab[i]*exp(-GMR*deltah/tbase);
  else
    delta=ptab[i]*pow(tbase/tlocal, GMR/tgrad);

  sigma=delta/theta;                                        //  density ratio
}


void set_alt_threshold(float altitude, String entered) {
    double sigma,delta,theta;
    double temp,pressure,density,asound;
    double viscosity,kinematicViscosity;
    if (altitude > 0 && altitude <= 100000) {
        Atmosphere(altitude/1000, sigma,delta,theta);
        pressure=PZERO*delta;
        configuration.altitude_alarm = altitude;
        configuration.pressure_alarm = pressure;
        EEPROM.put(0, configuration);
        Serial.println(F("Note that a low altitude will disable the altitude arming since\n\rthe GPS signal will never trigger the threshold.\n\rAlso note that valid positive numbers are parsed. Invalid entries are converted to 0."));
        Serial.print(F("Entered value: "));
        Serial.println(entered);
        Serial.print(F("Altitude threshold set to: "));
        Serial.print(altitude);
        Serial.println(F(" meters"));
        Serial.print(F("Calculated barometric pressure at altitude is set to: "));
        Serial.print(pressure);
        Serial.println(F(" Pascals"));
    } else {
        Serial.println(F("Invalid altitude.\n\rAcceptable range is 0-100000 m."));
    }
}

void print_serial_help() {
    // Prints the serial command list

    Serial.println(F("\n\n\n\rCommand help list\n\rCommands NOT case sensitive\n\r===========================\n"));
    Serial.println(F("help                  This command list.\n\r"));
    Serial.println(F("help sdcard           Help with SDCard commands."));
    Serial.println(F("help altitude         Help with altitude threshold commands."));
    Serial.println(F("help pressure         Help with pressure threshold commands."));
    Serial.println(F("help launch timer     Help with launch timer commands"));
    Serial.println(F("help location timer   Help with location timer commands\n\r"));
    
    Serial.println(F("read sdcard           Output the SDCard data as a CSV to serial console."));
    Serial.println(F("                      Output needs to be saved from your serial program"));
    Serial.println(F("                      or through console redirection."));
    Serial.println(F("set alt ###           Set GPS altitude threshold to ### meters."));
    Serial.println(F("==========================="));
    //delay(1000);
}



void set_pressure_threshold(float pressure) {

}

void set_launch_timeout(uint32_t timeout) {
}

void set_location_timeout(uint32_t timeout) {

}

void read_sdcard() {
  Serial.println(F("Reading SDCard..."));
}

void read_serial() {

    static String incoming = "";
    String original = "";
    char c;
    uint8_t loops = 0;
    float altitude;
    
    while (Serial.available() && loops < 35) {

        c = Serial.read();
        incoming += c;

        // Echo to the screen.
        /*Serial.print(c);
        Serial.print(F(" 0x"));
        Serial.print(c, HEX);
        Serial.print(F(" "));
        Serial.println(incoming.length());*/

        if (incoming.length() > 34) {
            // Max length of string to prevent eating all memory.
            // Adds 2 characters for a 32 byte command plus a new line and carriage return.

            incoming = "";

            Serial.println(F("\n\r*** Entry to long, please retry. Max limit is 32 characters. ***"));
        }

        loops++;
    }

    if (c == '\n' || c == '\r') {
      
        incoming.trim();

        if (incoming.length() > 0) {
            // The string has length, let's see what it says.
            original = incoming;
            // Convert to lower case.
            incoming.toLowerCase();

            Serial.print(F("Command received: "));
            Serial.println(incoming);
            if (incoming == "read sdcard") {
                read_sdcard();
            } else if (incoming == "help" || incoming == "?" || incoming == "??" || incoming == "/?") {
                print_serial_help();
            } else if (incoming.startsWith("set altitude")) {
                incoming.remove(0,12);
                incoming.trim();
                altitude = incoming.toFloat();
                set_alt_threshold(altitude, incoming);
                
            } else {
            
                Serial.print(F("*** Command unknown: "));
                Serial.print(original);
                Serial.println(F(" ***"));
                print_serial_help();
                //delay(1000);
            }
        }
    incoming = "";
    }

}

void loop() {
  static uint32_t counter = 0;
  // put your main code here, to run repeatedly:
  while(true) {
    //Serial.print(F("Counter is: "));
    //Serial.println(counter);
    //counter++;
    read_serial();
  }
  
}
