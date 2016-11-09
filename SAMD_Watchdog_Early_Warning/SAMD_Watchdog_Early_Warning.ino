#include "Adafruit_SleepyDog.h"
#include "ZeroRegs.h"

volatile bool early = false;

void setup() {
  // put your setup code here, to run once:
  Watchdog.disable();
  delay(10000);
  SerialUSB.begin(9600);
  //while(!SerialUSB) ;
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);

  SerialUSB.println("Restarting...");
  Watchdog.enable_early(WDT_CONFIG_PER_11_Val, WDT_EWCTRL_EWOFFSET_10_Val);
  ZeroRegOptions opts = {SerialUSB, false};
  printZeroRegWDT(opts);
}

void WDT_Handler(void) {
  early = true;
  Watchdog.clear_warning();
  digitalWrite(13, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  

  if (USB->DEVICE.FSMSTATUS.bit.FSMSTATE == USB_FSMSTATUS_FSMSTATE_SUSPEND_Val) {
    // Test if USB cable is in suspend, or not attached. If it is NOT attached go to sleep.
    
    // Disable USB
    USBDevice.detach();

    // SAMD sleep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;    
    __DSB();
    __WFI();

    // Enable USB and wait for resume if attached
    USBDevice.attach();
    USB->DEVICE.CTRLB.bit.UPRSM = 0x01u;
    while (USB->DEVICE.CTRLB.bit.UPRSM);
  }
 
  //SerialUSB.begin(9600);

  if (early == true) {
    SerialUSB.println(F("Early warning watch dog barked..."));
    Watchdog.reset();
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    early = false;
  }
  //delay(1000);
  SerialUSB.print(F("Waiting for interrupt: "));
  SerialUSB.println(millis());
}
