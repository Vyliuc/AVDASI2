#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

// TODO: set pins
#define INTERRUPT_PIN        2 
#define SERVO_PIN            3

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     96

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// horizontal balance variables
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
float error = 0; // excess moment from the balance of moments
float const lim = 180; //elevator deflection limit (change as required)
float M = 0; //second moment of area (REQUEST FROM CAD TEAM)
float refPotVal = 0;

int mode = 1; //gives current mode

Servo elevator;

File logsFile;

void setup() 
{
  initSD();
  transceiverSetup(rf69, rf69_manager);

  Serial.begin(9600);

  int mpuStatus = mpuSetup();

  if (mpuStatus != 0) 
  {
    Serial.println("Error when setting up MPU");
  }

  elevator.attach(SERVO_PIN);
}

void loop() {
  // Display roll, pitch, yaw from MPU
  mpuLoop();

  //receive values for pitch data - WAITING ON HAMISH
  float ang = 10;
  float angvel = 10;
  float angacc = 10;
  
  // constantly listen to the transceiver & check if any data has been received
  String response = receive(rf69, rf69_manager);

  // set the mode
  if (response == "Manual") 
  {
    mode = 0;
  }
  else if (response == "Auto") 
  {
    mode = 2;
  }
  else if (response == "Neutral") 
  {
    mode = 1;
  }
  
  //MANUAL MODE
  //search for whether it contains "PotValue:"
  String key = "PotValue:";
  //if it contains the key, and manual mode is active, use getIntFromString and write the potvalue to the servo
  if (response.indexOf(key) != -1 && mode == 0) 
  {
    float currentPotVal = getIntFromString(response);

    refPotVal = currentPotVal;

    // scale it to use it with the servo (value between 0 and 180)
    float angle = map(currentPotVal, 0, 1023, 0, 180);  

    // sets the servo position according to the scaled value   
    elevator.write(angle);
  } 

  //AUTO MODE
  if (mode == 2)
  {
    // save the previous deflection in def_previous
    float def_previous = def;

    // calculate error (in this case, excess moment)
    error = (q*xtail*St*((at*(ang+it+((angvel*xtail)/V)))+(adel*def))) + (xg*m*g)- M*angacc;
    
    //while the error is non zero, loop over moment balance until it's zero
    if (error != 0) 
    {
      //run moment balance
      def = (((M*angacc)/(q*xtail*St))-((xg*m*g)/(q*xtail*St))-(at*ang)-(at*((angvel*xtail)/(pow(V,2))))-(at*it))/adel;
    }

    if (def != def_previous) 
    {
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
  }
  
  //NEUTRAL MODE
  if (mode == 1)
  {
    elevator.write(0);
  }

  //print to serial monitor for debug
  Serial.print("potentiometer = ");
  Serial.print(refPotVal);
  Serial.print("\t del = ");
  Serial.print(def);
  Serial.print("\t pitch =");
  Serial.print(ang);
  Serial.print("\t vel =");
  Serial.print(angvel);
  Serial.print("\t acc =");
  Serial.print(angacc);
  Serial.print("\t error =")
  Serial.println(error);
}

/** 
 * SETUP MPU6050
 * Initialises MPU, initialises Digital Motion Processor (DMP), calibrates device, enables interrupt detection.
 * The DMP is the onboard processor that computes orientation from the accelerometer and gyro data.
 * \return device status (0 = success, 1 = initial memory load failed, 2 = DMP configuration updates failed)
 */
int mpuSetup() {
  // intitialise device
  Serial.println(F("Initialising MPU..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initialising DMP..."));
  devStatus = mpu.dmpInitialize();

  // gyro offsets
  // TODO: calibrate gyro and set values here
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // TODO: factory default?

  // Check whether DMP initialised successfully (returns 0 if successful)
  if(devStatus == 0) {
    // calibration: generate offsets and calibrate the MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    
    // turn on DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection (we may or may not need this)
    Serial.print(F("Enabling interrupt detection... (Arduino external interrupt "));
    Serial.print(F(digitalPinToInterrupt(INTERRUPT_PIN)));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set the DMP Ready flag so the main loop() knows it's ok to use it
    Serial.println("DMP ready! Waiting for the first interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialisation failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  return devStatus;
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void mpuLoop() {
  // If MPU setup failed, don't do anything
  if (!dmpReady) return;
  // Read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // get the latest packet
    // Get yaw, pitch, roll from DMP
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Display yaw, pitch, roll
    // TODO: make this do something useful
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
  }
}

void initSD()
{
  Serial.begin(9600);
  Serial.println("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("Initialization failed!");
  }
  else 
  {
    Serial.println("Initialization done");
    logToSD("---------------------------------");
    logToSD("SD card initialized successfully!");
  }
}

void logToSD(String msg) 
{
  logsFile = SD.open("logs.txt", FILE_WRITE);
  
  if (logsFile) 
  {
    logsFile.println(msg);
    logsFile.close();
  } 
  else 
  {
    Serial.println("Error opening logs.txt");
  }
}


int getIntFromString(String str) 
{
  //function to extract number from string

  //search for the first digit
  size_t i = 0;
  for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break; }

  // remove the first characters, which aren't digits
  str = str.substring(i, str.length() - i );

  // convert the remaining text to an integer
  int id = atoi(str.c_str());
  return (id);
}