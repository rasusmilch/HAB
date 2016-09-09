#include <avr/cpufunc.h>
#include <avr/interrupt.h>

// Watchdog reset pin
#define WATCHDOG_PIN        2   // Bare metal pin 7
#define FAN_PIN             0   // Bare metal pin 5
#define ELEMENT_PIN         4   // Bare metal pin 3
#define THERMISTOR_PIN      A3  // Bare metal pin 2
#define RESET_PIN           1   // Bare metal pin 6. Used to reset the oven.
//#define EXTERNAL_WATCHDOG   0   // Bare metal pin 5. May be used for the external watchdog of this chip.

// Give the reset controller this many *MILLIseconds* to start up since the bootloader is approx 2 seconds of loading.
#define RESET_INIT_TIME 15000L

// Minimum and maximum time in *MICROseconds* for valid reset window. We multiply at compiler time for speed.
#define RESET_WINDOW_MIN    2 * 1000L
#define RESET_WINDOW_MAX    1500 * 1000L

// Number of errors before resetting
#define KICK_ERRORS         4

// Pulse limits in *MICROseconds* for low watchdog kicking
#define MIN_PULSE           900L
#define MAX_PULSE           1100L

// Maximum number of resets before lockout
#define MAX_RESETS          3

// Delay between disabling oven and resetting controller. Allows serial or other debugging to hopefully
// make it out of the controller before resetting. Milliseconds.
#define DELAY_BEFORE_RESET  2 * 2000L

// How many milliseconds between temp readings.
const unsigned long read_temp_milli = 1000L;

// Max time allowed between readings
const long MAX_READING_TIMEOUT = 30000L;

// Number of samples per thermistor reading to average out noise.
const byte NUM_SAMPLES = 10;

// Shutdown oven if controller temp reaches this temp (most electrolytic capacitors and chips are rated to 85 C for industrial grade
const unsigned int fan_alarm = 70;
const int fan_alarm_therm_hi = 120;
const int fan_alarm_therm_lo = 175;

// ISR variables to hold pin state and times
volatile byte curr_watchdog_pin = LOW;
volatile byte prev_watchdog_pin = LOW;

volatile unsigned long prev_watchdog_time = 0;
volatile unsigned long curr_watchdog_time = 0;

// Did we have an update to pin status?
volatile boolean update = false;

unsigned int errors = 0;

// Did we just boot up ourselves, or
boolean init_chip = true;

// Time we reset the controller
unsigned long reset_time;

int number_of_resets = 0;

// Time temp was last read.
unsigned long last_read_temperature = 0L;

// Array to find the median of X readings together for smoothing and jitter control
#define TEMP_ARRAY_SIZE 9

// Array to hold thermistor readings for filter.
// Init thermistor array to larger value to prevent alarm on boot
unsigned int thermistor_array[TEMP_ARRAY_SIZE] = {1023};
unsigned int thermistor = 0;


void setup() {

    pinMode(FAN_PIN, OUTPUT);
    pinMode(ELEMENT_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);

    // Watchdog pin as input.
    pinMode(WATCHDOG_PIN, INPUT);

    // Possible external watchdog?
    //pinMode(EXTERNAL_WATCHDOG, OUTPUT);

    // Hold reset LOW to hold controller in reset
    digitalWrite(RESET_PIN, LOW);

    disable_control_func();

      // Prime the analog capacitor by reading now.
      analogRead(THERMISTOR_PIN);

      // Read current value into array to prevent boot alarm
      unsigned int therm_prime = analogRead(THERMISTOR_PIN);
      for (unsigned int i{0}; i < TEMP_ARRAY_SIZE; i++) {
        thermistor_array[i] = therm_prime;
      }

      thermistor = therm_prime;


    // Halt interrupts
    cli();

    // Memory barrier to stop reordering
    _MemoryBarrier();

    // Setup MCU Control Register so that any logical change on INT0 generates an interrupt
    MCUCR = MCUCR | 0b00000001;

    // General Interrupt Mask Register to enable INT0
    GIMSK = GIMSK | 0b01000000;

    // Memory barrier to stop reordering
    _MemoryBarrier();

    // Enable interrupts
    sei();

    // Start up the controller
    digitalWrite(RESET_PIN, HIGH);

    // Record the time and flag reset.
    init_chip = true;
    curr_watchdog_time = prev_watchdog_time = micros();
    reset_time = millis();
}

ISR(INT0_vect) {
    prev_watchdog_pin = curr_watchdog_pin;
    curr_watchdog_pin = digitalRead(WATCHDOG_PIN);

    prev_watchdog_time = curr_watchdog_time;
    curr_watchdog_time = micros();

    update = true;

}


/*
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
*/

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

/*
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
*/

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


void process_temps(unsigned int thermistor_raw) {
    // Process raw temp values.
    unsigned int sorted_thermistor[TEMP_ARRAY_SIZE] = {0};

    // Make room to nsert new value at end of array
    for (int i = 0; i < TEMP_ARRAY_SIZE - 1; i++) {

        // Move temps down one
        thermistor_array[i] = thermistor_array[i + 1];
  }


    thermistor_array[TEMP_ARRAY_SIZE - 1] = thermistor_raw;

    // Unsorted lists are copied
    for (unsigned int i = 0; i < TEMP_ARRAY_SIZE; i++) {

        sorted_thermistor[i] = thermistor_array[i];
    }

    // Sort lists
    sort_array(sorted_thermistor, TEMP_ARRAY_SIZE);

    // Prepare median for their new home.
    thermistor = median(sorted_thermistor, TEMP_ARRAY_SIZE);

}


void fetch_temps() {
    // Grab raw temps and process them.
    unsigned int raw_thermistor = read_thermistor();

    last_read_temperature = millis();

    process_temps(raw_thermistor);

}

void reset_controller() {

    // Disable oven
    disable_control_func();

    // Delay resetting controller to allow debugging to potentially be received.
    delay(DELAY_BEFORE_RESET);

    // Hold reset high.
    digitalWrite(RESET_PIN, LOW);

    // Delay to hold reset low. Datasheet states min of 2.5 microseconds.
    delayMicroseconds(10);
    //delay(1000);

    digitalWrite(RESET_PIN, HIGH);

    // We're flagging the boot
    init_chip = true;

    // Increase number of resets.
    number_of_resets++;

    // Reset errors for this reboot
    errors = 0;

    // Record the time.
    curr_watchdog_time = prev_watchdog_time = micros();
    reset_time = millis();
    _MemoryBarrier();
}



void disable_control_func() {
    // Turn on fan
    digitalWrite(FAN_PIN, HIGH);

    // Disable elements
    digitalWrite(ELEMENT_PIN, LOW);

}


void enable_control_func() {
    // Turn fan over to controller
    digitalWrite(FAN_PIN, LOW);

    // Enable elements
    digitalWrite(ELEMENT_PIN, HIGH);
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

void check_safe_temps() {

    // If thermistor is cooler than shutdown alarm and we're error free
    if (thermistor > fan_alarm_therm_hi) {

        if (errors == 0 && init_chip == false) {
            // Enable controller auto functions
            enable_control_func();
        }

    } else {
        // Temp is high, controller malfunction? Turn on fans and disable elements.
        disable_control_func();
    }

}


void loop() {
    if (update == true && init_chip == false) {
        update = false;

        // Time difference
        unsigned long difference = curr_watchdog_time - prev_watchdog_time;

        // Did we transistion from LOW to HIGH?
        if (curr_watchdog_pin == HIGH && prev_watchdog_pin == LOW) {

            if (difference < MIN_PULSE || difference > MAX_PULSE) {
                // Too short or long of a LOW pulse, increase errors
                errors++;
            } else if (errors > 0) {
                // Subtract an error for good behavior, but only if not already 0.
                    errors--;
            }

        } else if (curr_watchdog_pin == LOW && prev_watchdog_pin == HIGH) {
            // Transistion from HIGH to LOW. The start of a kick pulse
            if (difference < RESET_WINDOW_MIN || difference > RESET_WINDOW_MAX) {
                // We were too fast or slow kicking the dog.
                errors++;
            } else if (errors > 0) {
                // Subtract an error for good behavior, but only if not already 0.
                    errors--;
            }
        }


    } else if (micros() - curr_watchdog_time > RESET_WINDOW_MAX) {
        // Check for no kicking at all.

        // First boot or reset?
        if (init_chip == false) {
            // Not booting
            errors++;
        }

    }

    // Time to reset the controller boot flag?
    if (init_chip == true && (millis() - reset_time) > RESET_INIT_TIME) {
        init_chip = false;
    }

    if (errors > KICK_ERRORS) {
        if (number_of_resets < MAX_RESETS) {
            reset_controller();
        } else {
            // We're over the maximum resets. Hold controller in reset to disable
            digitalWrite(RESET_PIN, LOW);

            // Disable elements and turn on fan.
            disable_control_func();
        }
    }


    // Time for new reading?
    if (time_diff(last_read_temperature) > read_temp_milli) {

        fetch_temps();

    }

    // How are the safe temps. We place outside bad thermocouple response loop because internal temperature
    // is not dependant on bad external connection.
    // Additionally we have a thermistor for external oven temperature for a fail safe.
    check_safe_temps();

}
