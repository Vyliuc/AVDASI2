#include <transceiver.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     420

void setup() {
  transceiver_setup(RADIO_TX_ADDRESS);
}
  float refPotVal=0;
void loop() {
  // constantly listen to the transceiver & check if any data has been received
  String response = receive();

  if (response == "Manual") 
  {  refPotVal=getPotValue;

    // convert pot value to string and transmit to controller
    String refPotValstring= String (refPotVal);
    String responseExpected = "CurrentPotValue: " + refPotValstring;
    String response = transmit(RADIO_TX_ADDRESS, refPotValstring);
  }
  
    else if (response == "Auto") 
  {

  }
  else if (response == "Neutral") 
  { float currentPotVal=refPotVal;
  
    // convert pot value to string and transmit to controller
    String currentPotValstring= String (currentPotVal);
    String responseExpected = "CurrentPotValue: " + currentPotValstring;
    String response = transmit(RADIO_TX_ADDRESS, currentPotValstring);
  }
}
