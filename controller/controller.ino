#include <tranceiver.h>

void setup() {
  tranceiver_setup();
}

// 0 - manual
// 1 - neutral
// 2 - auto
int mode = 1;

void loop() { 
  // TODO: create a function that monitors the position of the switch
  int switchPos = getSwitchPosition();
  
  // If the switch is on Manual mode & if the potentiometer has changed the value
  if (switchPos == 0)
  {
    mode = 0;
    
    int potValue = 0; // set to potentiometer value
    std::string potValueString = std::to_string(potValue);
    char const *potValueChar = potValueString.c_str();

    char responseExpected[] = "PotValue: ";
    strcpy(responseExpected, potValueChar);
    
    char* response = transmit(data, potValueChar);

    if (response == responseExpected) 
    {
      // set Manual status led to GREEN
      // blink the Manual mode status led
      // switch Auto status led OFF
      // log success
    }
  }
  // If the switch is on Auto mode & was on a different mode before
  // Transmit the instructions to active an Auto mode on aircraft
  else if (switchPos == 2 && mode != 2) 
  {
   mode = 2;

   char cmd[] = "Auto";
   char responseExpected[] = "Auto Mode Activated!";

   char* response = transmit(data, cmd);

   if (response == responseExpected) 
    {
      // set Auto mode status led to GREEN
      // switch Manual status led OFF
      // log success
    }
  }
  // If the switch is on Neutral & was NOT on neutral before
  else if (switchPos == 1 && mode != 1) 
  {
    mode = 1;

    
  }
  // set to neutral mode otherwise
  else {
    mode = 1;

   char cmd[] = "Neutral";
   char responseExpected[] = "Neutral Mode Activated!";

   char* response = transmit(data, cmd);

   if (response == responseExpected) 
    {
      // log success
      // switch Manual and Auto leds off
    }
  }

  // TODO: Receive if needed
  //receive(data);
}

int getSwitchPosition() {

  // return the switch position
  return 1;
}
