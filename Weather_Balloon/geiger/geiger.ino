//byte previous_pins = 0xFF;
unsigned long geiger1 = 0;
unsigned long test, current_true, previous_true = 0;
bool updated = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(14, INPUT);
    // Enable pin interrupts for Geiger counter on
    // pin 64 (PJ1, PCINT10, TXD3), on PCIE1.
    // Arduino D14 (TX3)
    // Clear interrupts
    cli();
    // Enable pin change interrupts for the needed port.
    PCICR |= 1<<PCIE1;
    
    // Change the mask to allow interrupts ONLY for this pin.
    PCMSK1 |= 1<<PCINT10;
    
    // Disable to interrupt per datasheet
    //EIMSK |= (0<<INT1);
    
    // 
    //EICRA |= (1<<ISC11) & (0<<ISC10);
    

    
    // Set interrupts
    sei();

    Serial.print(1<<PJ1, BIN);
    delay(1000);
}

ISR(PCINT1_vect) {
  static byte previous_pins = 0xFF;
  byte current_state = PINJ;
  //byte changed_pins = previous_pins ^ current_state;
  //if (updated == false) {
    test++;
    if ((current_state & (1<<PJ1)) == 0 && (previous_pins & (1<<PJ1)) > 0) {
      geiger1++;
    }
  
    previous_pins = current_state;
    updated = true;
  //}
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(previous_pins, BIN);
  /*if (updated == true) {
    Serial.print(test);
    Serial.print(F(", "));
    Serial.print(current_true);
    Serial.print(F(", "));
    Serial.print(previous_true);
    Serial.print(F(", "));*/
    Serial.println(geiger1);
    /*Serial.print(F(", "));
    Serial.print(previous_pins, BIN);
    Serial.print(F(", "));
    Serial.println(previous_pins & (1<<PJ1));
    updated = false;*/
  //}
}
