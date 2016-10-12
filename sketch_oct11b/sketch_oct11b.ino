#include <avr/interrupt.h>
volatile int reset_flag = 0;
#define RED_LED 22
#define YELLOW_LED 23
#define GREEN_LED 24
volatile uint8_t previous_pins_e = 0;
volatile uint8_t current_state_e = 0;

ISR(INT4_vect) {
    // Pin change interrupt for Geiger counters.
    //No falling or rising edge detection on these pins so we
    //need to track previous pin states.
    //When a pin has changed value we update the appropriate counter.
    //Geiger counter outputs a low pulse of 150 microseconds on particle detection.
    //

    // Previous pin value for port. Static to preserve between calls.

    static bool button = false;
    static uint32_t button_time = 0;
    static bool red_status, yellow_status, green_status;

    uint32_t current_time = millis();

    // Grab current pin state quickly. Low pulse from Geiger is only 150 microseconds.
    current_state_e = PINE;
/*
    // If the current pin state is high, and previous it was low, we have
    // a rising edge.
    if ((current_state_e & (1<<PE4)) >= 1 && (previous_pins_e & (1<<PE4)) == 0) {
        button = true;
        button_time = current_time;
        red_status = digitalRead(RED_LED);
        yellow_status = digitalRead(YELLOW_LED);
        green_status = digitalRead(GREEN_LED);
        digitalWrite(RED_LED, LOW);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
    } else if (current_state_e & (1 << PE4) == 0 && (previous_pins_e & (1 << PE4) >= 1)) {
        digitalWrite(RED_LED, red_status);
        digitalWrite(YELLOW_LED, yellow_status);
        digitalWrite(GREEN_LED, green_status);

        if (current_time - button_time > 2000L && current_time - button_time < 3000L) {
            reset_flag = true;
        } else {
            reset_flag = false;
        }
    }*/

    if ((current_state_e & (1 << PE4)) > 0) {
      DDRA |= (1 << PA2);
      PORTA |= (1 << PA2);
    } else {
      PORTA &= ~(1 << PA2);
    }
    // Store for later.
    previous_pins_e = current_state_e;
    reset_flag++;
}


/*ISR(INT4_vect) {
  reset_flag++;
}
*/
void configure_button() {
    pinMode(2, INPUT_PULLUP);

    // Clear interrupts
    cli();

    EIMSK &= ~(1 << INT4);
    
    EICRB &= ~(1 << ISC41);
    EICRB |= (1 << ISC40);

    EIMSK |= (1 << INT4);
    
    // Enable pin change interrupts for the needed port.
    //PCICR |= (1<<PCIE0);

    // Change the mask to allow interrupts ONLY for this pin.
    //PCMSK0 |= (1<<PCINT4);


    // Set interrupts
    sei();

    // Now pin changes will call the ISR(PCINT0_vect) for button presses.
}

void setup() {
  // put your setup code here, to run once:
  configure_button();
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.print(current_state_e & (1 << PE4), HEX);
  Serial.print(F(" "));
  Serial.print(previous_pins_e & (1 << PE4), HEX);
  Serial.print(F(" "));
  Serial.println(reset_flag);
  delay(1000);
}
