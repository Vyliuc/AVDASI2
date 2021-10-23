#include <tranceiver.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     420

void setup() {
  tranceiver_setup(RADIO_TX_ADDRESS);
}

void loop() {
  // constantly listen to the tranceiver & check if any data has been received
  String response = receive();

  if (response == "Manual") 
  {

  }
  else if (response == "Auto") 
  {

  }
  else if (response == "Neutral") 
  {

  }
}
