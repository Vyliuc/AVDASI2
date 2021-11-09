#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

// TODO: set pins
#define INTERRUPT_PIN        2 

#define SERVO_PIN            23

#define RFM69_CS      10
#define RFM69_INT     4
#define RFM69_RST     9
#define LED           digitalPinToInterrupt(RFM69_INT)

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     96

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t dmpStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]        quaternion container
VectorFloat gravity; // [x, y, z]           gravity vector
float ypr[3];        // [yaw, pitch, roll]  yaw/pitch/roll container and gravity vector
float vel[3];        // [vx, vy, vz]        angular velocity vector
float acc[3];        // [ax, ay, az]        angular acceleration vector
float Time[2];       // [current, last]     timestamp container

// struct for attitude output
struct att {
  float time;     // time since program started
  float yaw;      // yaw in degrees
  float pitch;    // pitch in degrees
  float roll;     // roll in degrees
  float yawVel;   // yaw rate in degrees/second
  float pitchVel; // pitch rate in degrees/second
  float rollVel;  // roll rate in degrees/second
  float yawAcc;   // yaw angular acceleration in degrees/second^2
  float pitchAcc; // pitch angular acceleration in degrees/second^2
  float rollAcc;   // roll angular acceleration in degrees/second^2
};


float refPotVal = 0;

double deflAngle = 0; // initial elevator deflection
double pitchAngle = 0;
double angvel = 0;
double angacc = 0;

int mode = 1; //gives current mode

Servo elevator;

File logsFile;

void setup() 
{
  initSD();
  transceiverSetup(rf69, rf69_manager, RFM69_CS, RFM69_INT, RFM69_RST, LED);

  Serial.begin(115200);
  
  mpuSetup();

  elevator.attach(SERVO_PIN);
}

void loop() {
  // constantly listen to the transceiver & check if any data has been received
  String response = receive(rf69, rf69_manager, pitchAngle, LED);

  // set the mode
  if (response.indexOf("Manual Mode Activated!") != -1) 
  {
    mode = 0;
  }
  else if (response.indexOf("Auto Mode Activated!") != -1) 
  {
    mode = 2;
  }
  else if (response.indexOf("Neutral Mode Activated!") != -1) 
  {
    mode = 1;
  }
  
  //MANUAL MODE
  //search for whether it contains "PotValue:"
  //if it contains the key, and manual mode is active, use getIntFromString and write the potvalue to the servo
  if (response.indexOf("PotValue: ") != -1 && mode == 0) 
  { 
    // Get roll, pitch, yaw from MPU
    att attitude = getAttitude();

    pitchAngle = attitude.pitch;
    angvel = attitude.pitchVel;
    angacc = attitude.pitchAcc;
  
    // Display pitch and timestamp
    Serial.print(attitude.pitch,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchVel,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchAcc,4);
    Serial.print(F("\t"));
    Serial.println(attitude.time,6);
    
    refPotVal = getIntFromString(response, "PotValue: ");

    // scale it to use it with the servo (value between 0 and 180)
    deflAngle = map(refPotVal, 0, 1023, 0, 180);  
    
    // sets the servo position according to the scaled value   
    elevator.write(deflAngle);
    delay(15);
  } 

  //AUTO MODE
  if (mode == 2)
  {
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
    float p_dyn = 0.5*rho*(pow(V,2)); // dynamic pressure (MAY BE VARIABLE LATER - MOVE INTO LOOP IF THIS IS THE CASE)
    float error = 0; // excess moment from the balance of moments
    float const lim = 180; //elevator deflection limit (change as required)
    float M = 19208; //estimated moment of inertia (upper estimate 24010, lower estimate 19208)

    // Get roll, pitch, yaw from MPU
    att attitude = getAttitude();

    pitchAngle = attitude.pitch;
    angvel = attitude.pitchVel;
    angacc = attitude.pitchAcc;
  
    // Display pitch and timestamp
    Serial.print(attitude.pitch,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchVel,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchAcc,4);
    Serial.print(F("\t"));
    Serial.println(attitude.time,6);

    // save the previous deflection in def_previous
    float deflAnglePrevious = deflAngle;

    // calculate error (in this case, excess moment)
    error = (p_dyn*xtail*St*((at*(pitchAngle+it+((angvel*xtail)/V)))+(adel*deflAngle))) + (xg*m*g)- M*angacc;

    Serial.print("Excess moment: ");
    Serial.println(error);
    
    //while the error is non zero, loop over moment balance until it's zero
    if (error != 0) 
    {
      //run moment balance
      deflAngle = (((M*angacc)/(p_dyn*xtail*St))-((xg*m*g)/(p_dyn*xtail*St))-(at*pitchAngle)-(at*((angvel*xtail)/(pow(V,2))))-(at*it))/adel;

      Serial.print("Defl angle needed: ");
      Serial.println(deflAngle);

      if (deflAngle != deflAnglePrevious) 
      {
        //write deflection onto servo after some scaling and limiting
        if (abs(deflAngle) <= lim) 
        {
          elevator.write(deflAngle);
          delay(15);
        }
        else if (deflAngle > lim) 
        {
          elevator.write(lim);
          delay(15);
        }
        else if (deflAngle < (-1*lim)) 
        {
          elevator.write((-1*lim));
          delay(15);
        }
      }
    }
  }
  
  //NEUTRAL MODE
  if (mode == 1)
  {
    elevator.write(0);
    delay(15);
  }
}

/** 
 * SETUP MPU6050
 * Initialises MPU, initialises Digital Motion Processor (DMP), calibrates device, enables interrupt detection.
 * The DMP is the onboard processor that computes orientation from the accelerometer and gyro data.
 */
void mpuSetup() {
    // join I2C bus
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    
    // intitialise device
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // initalise the DMP
    dmpStatus = mpu.dmpInitialize();

    // calibrated offsets
    mpu.setXAccelOffset(-1471);
    mpu.setYAccelOffset(-2535);
    mpu.setZAccelOffset(16382);
    mpu.setXGyroOffset(35);
    mpu.setYGyroOffset(23);
    mpu.setZGyroOffset(14);

    // Check whether DMP initialised successfully (devStatus == 0 if successful)
    if(dmpStatus == 0) {
        // calibration: generate offsets and calibrate the MPU6050
        mpu.CalibrateAccel(16);
        mpu.CalibrateGyro(16);
        mpu.PrintActiveOffsets();
        
        // enable DMP
        mpu.setDMPEnabled(true);
        dmpReady = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialisation failed (code "));
        Serial.print(dmpStatus);
        Serial.println(F(")"));
    }
}

/**
 * Polls the MPU then converts the pitch value to degrees.
 * \return MPU pitch in degrees 
 */
att getAttitude() {
    readYPR();
    struct att a;
    a.time = Time[0];
    a.yaw = ypr[0] * 180.0/M_PI;
    a.pitch = ypr[1] * 180.0/M_PI;
    a.roll = ypr[2] * 180.0/M_PI;
    a.yawVel = vel[0] * 180.0/M_PI;
    a.pitchVel = vel[1] * 180.0/M_PI;
    a.rollVel = vel[2] * 180.0/M_PI;
    a.yawAcc = acc[0] * 180.0/M_PI;
    a.pitchAcc = acc[1] * 180.0/M_PI;
    a.rollAcc = acc[2] * 180.0/M_PI;
    return a;
}

/**
 * Polls the IMU Digital Motion Processor (DMP) for its current yaw, pitch, and roll.
 * Updates the global ypr[] array.
 */
void readYPR() {
    // If MPU setup failed, don't do anything
    if (!dmpReady) return;
    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // get the latest packet
        // Log timestamp
        timeStamp(Time);
        // Get yaw, pitch, roll from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Differentiate to find velocity and acceleration
        diff(ypr, vel, Time);
        diff(vel, acc, Time);
    }
}

void timeStamp(float *Time) {
    // Time[0] stores current time value, Time[1] stored the last time value
    Time[1] = Time[0];
    Time[0] = micros()/1.0E6; // seconds passed since program started
}

void diff(float *input, float *result, float *Time) {
    float interval = Time[0] - Time[1];
    for(uint8_t i = 0; i < sizeof(input); i++) {
        result[i] = input[i] / interval;
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

int getIntFromString(String str, String startPhrase) 
{
  //function to extract number from string
  int index = str.indexOf(startPhrase);
  String substr = str.substring(index);

  int intStartIndex = -1;
  int intEndIndex = -1;
  int intToReturn = 0;

  for (int i = 0 ; i < substr.length(); i++) 
  {
    if (isdigit(substr[i]))
    {
      if (intStartIndex == -1) 
      {
        intStartIndex = i;
      }

      if ((i + 1) <= (substr.length() - 1))
      {
        if (!isdigit(substr[i+1]))
        {
          intEndIndex = i + 1;
        }
      }
      else 
      {
        intEndIndex = i;
      }
    }
  }

  substr = substr.substring(intStartIndex, intEndIndex);

  // convert the remaining text to an integer
  intToReturn = atoi(substr.c_str());
  
  return intToReturn;
}
