#include <transceiver.h>
#include <Servo.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     420

//function to extract number from string
int getval(String str){
  //search for the first digit
  size_t i = 0;
  for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break; }

  // remove the first characters, which aren't digits
  str = str.substring(i, str.length() - i );

  // convert the remaining text to an integer
  int id = atoi(str.c_str());
  return (id);
}

//allocate memory
int i = 0; // counter
float V = 20; // velocity (MAY BE VARIABLE LATER - MOVE INTO LOOP IF THIS IS THE CASE)
float St = 0.141; // tailplane area
float at = 4.079; // tail lift curve slope
float adel = 2.37; // elevator lift curve slope
float xg = 31.75; // (xcg - xpivot)
float xtail = 815; // (xact - xpivot)
float m = 10; // mass (assumed 10kg)
float it = -2; // tail setting angle
float const g = 9.80665; //g
float const rho = 1.225; //density
float q = 0.5*rho*(pow(V,2)); // dynamic pressure (MAY BE VARIABLE LATER - MOVE INTO LOOP IF THIS IS THE CASE)
float def = 0; // initial elevator deflection
float const lim = 180; //elevator deflection limit (change as required)
float M = 100 ; //second moment of area (REQUEST FROM CAD TEAM)
int mode = 1; //gives current mode
float refPotVal = 0;

//initialise servo
Servo elevator;

void setup() {
  transceiver_setup(RADIO_TX_ADDRESS);
  // setting up servo pin
  elevator.attach(9); // fill with the pin that the servo is on
}

void loop() {
  //receive values for pitch data - WAITING ON HAMISH
  float ang = 10;
  float angvel = 10;
  float angacc = 10;
  
  // constantly listen to the transceiver & check if any data has been received
  String response = receive();
  
  //SET MODES
  if (response == "Manual") 
  {
    //Set mode
    mode = 0;
  }
  else if (response == "Auto") 
  {
    //Set mode
    mode = 2;
  }
  else if (response == "Neutral") 
  {
    //Set mode
    mode = 1;
  }
  
  //MANUAL MODE
  //search for whether it contains "PotValue:"
  String key = "PotValue:";
  //if it contains the key, and manual mode is active, use getval and write the potvalue to the servo
  if (response.indexOf(key) != -1 && mode == 0) {
    float currentPotVal = getval(response);
    // scale it to use it with the servo (value between 0 and 180)
    float outval = map(currentPotVal, 0, 1023, 0, 180);  
    // sets the servo position according to the scaled value   
    elevator.write(outval); 
    refPotVal = currentPotVal;                
  } 

  //AUTO MODE
  if(mode == 2){
    // stability reference
    float ref = (q*xtail*St*((at*(ang+it+((angvel*xtail)/V)))+(adel*def)))+(xg*m*g)- M*angacc;

    //while reference is non zero, loop over moment balance until it's zero
    if(ref != 0) {
      //run moment balance
      float def = (((M*angacc)/(q*xtail*St))-((xg*m*g)/(q*xtail*St))-(at*ang)-(at*((angvel*xtail)/(pow(V,2))))-(at*it))/adel;
    
      //write deflection onto servo after some scaling and limiting
      if (abs(def) <= lim) {
        elevator.write(def);
      }
      else if (def > lim) {
        elevator.write(lim);
      }
      else if (def < (-1*lim)) {
        elevator.write((-1*lim));
      }
    }
  }
  
  //NEUTRAL MODE

}
