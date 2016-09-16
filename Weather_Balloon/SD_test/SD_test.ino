#include <Sd2Card.h>

Sd2Card SDcard;

// Number of 512 byte blocks in the card.
uint32_t CARD_BLOCKS{0};

uint8_t SDbuffer[512] = {0};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Here we go..."));
  SDcard.init();
  Serial.println(F("SD Initialized..."));

  CARD_BLOCKS = SDcard.cardSize();
  Serial.print(CARD_BLOCKS);

  for (uint32_t j{1}; j < CARD_BLOCKS; j++) {
    SDcard.readBlock(j, SDbuffer);
    
    
    
    for (unsigned int i{0}; i < 512; i++) {
      Serial.println(SDbuffer[i]);
    } 
Serial.println(F("Read buffer"));
      delay(2000);
    for (unsigned int i{0}; i < 512; i++) {
      SDbuffer[i] = i;
      Serial.println(SDbuffer[i]);
    }
    
    int result = SDcard.writeBlock(j, SDbuffer);
    if (result == 1) {
      Serial.println(F("Wrote buffer"));
  
      SDcard.readBlock(j, SDbuffer);
    
    
      for (unsigned int i{0}; i < 512; i++) {
        Serial.println(SDbuffer[i]);
      } 
              Serial.println(F("Read buffer"));

    }else {
        Serial.println(F("Write failure"));
        
    }

    delay(2000);
  }


}

void loop() {
  

}
