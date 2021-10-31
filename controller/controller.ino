#include <transceiver.h>

// Set pins for status leds
#define MANUAL_LED_PIN    1
#define AUTO_LED_PIN      2

#define RADIO_TX_ADDRESS     96
#define RADIO_RX_ADDRESS     69

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

void setup() {
  transceiverSetup(rf69, rf69_manager);

  // display 0 degrees angle after setup
  displayDeflectionAngle(0);
}

// 0 - manual
// 1 - neutral
// 2 - auto
int mode = 1;

int potValue = 0;

void loop() { 
  int switchPos = getSwitchPosition();
  
  // If the switch is on Manual mode
  if (switchPos == 0)
  {
    // If the switch was NOT on Manual
    if (mode != 0) 
    {
      // Transmit the instructions to activate a Manual mode on aircraft
      String cmd = "Manual";
      String responseExpected = "Manual Mode Activated!";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response == responseExpected) 
      {
        // switch Manual status led ON
        // switch Auto status led OFF
        statusLEDS(true, false);
         
        // log success
      }
    }

    // if potentiometer value has changed
    int currentPotValue = getCurrentPotValue();
    
    if (currentPotValue != potValue) 
    {
      // TODO: also add some tolerance, pot value might fluctuate even when potentiometer is still??
      potValue = currentPotValue;

      // convert the pot value to string
      String potValueString = String(potValue);

      String responseExpected = "PotValue: " + potValueString;

      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, potValueString);

      if (response == responseExpected) 
      {
        // blink Manual status led 1 time, 20 + 20ms delay
        Blink(MANUAL_LED_PIN, 20, 1);

        displayDeflectionAngle(potValue);
        
        // log success
      }  
    }

    mode = 0;
  }
  // If the switch is on Auto mode
  else if (switchPos == 2) 
  {
    // If the switch was NOT on Auto
    if (mode != 2) 
    {
      // Transmit the instructions to activate an Auto mode on aircraft
      String cmd = "Auto";
      String responseExpected = "Auto Mode Activated!";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response == responseExpected) 
      {
        // switch Auto status led ON
        // switch Manual status led OFF
        statusLEDS(false, true);
        
        // log success
      } 
    }

    mode = 2;
  }
  // If the switch is on Neutral
  else if (switchPos == 1) 
  {
    // If the switch was NOT on Neutral
    if (mode != 1) 
    {
      // Transmit the instructions to activate a Neutral mode on aircraft
      String cmd = "Neutral";
      String responseExpected = "Neutral Mode Activated!";
  
      String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
  
      if (response == responseExpected) 
      {
        // switch Auto status led OFF
        // switch Manual status led OFF
        statusLEDS(false, false);
        
        // log success
      } 
    }
    
    mode = 1;    
  }
}

int getSwitchPosition() {

  // return the switch position
  return 1;
}

int getCurrentPotValue() {

  // return the potentiometer value
  return 0;
}

void displayDeflectionAngle(int potValue) {
  // calculate the angle from the potentiometer voltage 
  // display the angle
  // log the angle
}

void statusLEDS(bool manualLedOn, bool autoLedOn) {
  if (manualLedOn) {
    digitalWrite(MANUAL_LED_PIN, HIGH);
  }
  else {
    digitalWrite(MANUAL_LED_PIN, LOW);
  }

  if (autoLedOn) {
    digitalWrite(AUTO_LED_PIN, HIGH);
  }
  else {
    digitalWrite(AUTO_LED_PIN, LOW);
  }
}
