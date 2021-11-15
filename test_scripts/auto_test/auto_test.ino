#include <Servo.h>
#define SERVO_PIN            23
Servo elevator;

//declare variables
float t = 0;
float ang = 0;
float angvel = 0;
float angacc = 0;
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
float const e = 2.7182818284; //e
float const rho = 1.225; //density
float q = 0.5*rho*(pow(V,2)); // dynamic pressure (MAY BE VARIABLE LATER - MOVE INTO LOOP IF THIS IS THE CASE)
float def = 0; // initial elevator deflection
float error = 0; // excess moment from the balance of moments
float const lim = 50; //elevator deflection limit (change as required)
float M = 19208; //second moment of area (upper estimate 24010, lower estimate 19208)
//EXCITATION PARAMETERS
float a = 40;
float b = 0.009;

void setup() {
  // put your setup code here, to run once:
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  elevator.attach(SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  ang = a*sin(t)*(pow(e,-b*t)); //without decay a*sin(t)
  angvel = a*cos(t)*(pow(e,-b*t)) + (pow(e,-b*t))*(-b)*a*sin(t); //without decay a*cos(t)
  angacc = (a*sin(t)*(pow(e,-b*t)))*(pow(b,2) - 1) + (a*cos(t)*(-2*b*pow(e,-b*t))); //without decay a*(-1)*sin(t);

  // calculate error (in this case, excess moment)
  error = (q*xtail*St*((at*(ang+it+((angvel*xtail)/V)))+(adel*def))) + (xg*m*g)- M*angacc;

  //while the error is non zero, loop over moment balance until it's zero
  if (error != 0) 
  {
    //run moment balance
    def = (((M*angacc)/(q*xtail*St))-((xg*m*g)/(q*xtail*St))-(at*ang)-(at*((angvel*xtail)/(pow(V,2))))-(at*it))/adel;

    //write deflection onto servo after some scaling and limiting
    if (abs(def) <= lim) 
    {
      elevator.write(def);
    }
    else if (def > lim) 
    {
      elevator.write(lim);
    }
    else if (def < (-1*lim)) 
    {
      elevator.write((-1*lim));
    }
   }

  Serial.print("\t time = ");
  Serial.print(t);
  Serial.print("\t pitch = ");
  Serial.println(ang);
  Serial.print("\t response = ");
  Serial.println(def);
  
  t = t + 0.01;
  delay (1);
}
