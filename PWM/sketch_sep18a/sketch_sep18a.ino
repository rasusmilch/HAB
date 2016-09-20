void setup() {
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);
  //Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  cli();
  while (true) {
    for (unsigned long i=1; i<=10L; i++) {
      
      //Serial.print(i);
      //Serial.print(" ");
      //Serial.print(500000L/i);
      for (unsigned long j=0; j < 5000L/i; j++) { 
        //for (int k = 255; k > 0; k=k-10) {
        int k = 127;
          //Serial.print(" ");
          //Serial.println(k);
          //for (unsigned long m=0; m < 20000L; m++) {
            //digitalWrite(9, HIGH);
            PORTH = (1 << PH6);
            for (int n = 5*i * k; n>0; n--) {
              _NOP();
            }

            
            PORTH = (0 << PH6);
            //digitalWrite(9, LOW);
            for (int n = 5*i * (255 - k); n > 0; n--) {
              _NOP();
            }
            //delayMicroseconds(i*(255-k));
        
          //}
        //}
      }
    }
  }
}
