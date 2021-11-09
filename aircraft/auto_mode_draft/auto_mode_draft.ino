#include <transceiver.h>
#include <Servo.h>

float V = 20;
float St = 0.141;
float at = 4.079;
float adel = 2.37;
float xg = 31.75;
float xtail = 815;
float m = 10;
float it = -2;
float const g = 9.80665;
float const rho = 1.225;
float q = 0.5*rho*(pow(V,2));
float def = 0;
float const lim = 3;
float M = 11920; //REQUEST FROM CAD TEAM


//initialise servo
Servo elevator;

void setup() {
  // setting up servo pin
  elevator.attach(23); // fill with the pin that the servo is on
}

void loop() {
  //receive values for pitch data
  float ang = 10;
  float angvel = 10;
  float angacc = 10;
  
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

  //when reference reaches zero, stop deflecting angle 
  else {
    elevator.write(0);
  }
}
