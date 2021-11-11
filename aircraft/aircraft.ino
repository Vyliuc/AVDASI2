#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>
#include <String.h>

#define SERVO_PIN            23

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
float Time;          // [current time]      timestamp

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
  float rollAcc;  // roll angular acceleration in degrees/second^2
};

float deflAngle = 0; // initial elevator deflection
float pitchAngle = 0;
double angvel = 0;
double angacc = 0;

float cmdInterval = micros()/1.0E6;

int mode = 1; //gives current mode

Servo elevator;

File logsFile;

void setup() 
{
  initSD();
  transceiverSetup(rf69, rf69_manager);

  Serial.begin(115200);
  
  mpuSetup();

  elevator.attach(SERVO_PIN);
}

void loop() {
  // constantly listen to the transceiver & check if any data has been received
  String response = receive(rf69, rf69_manager, pitchAngle);

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
  //search for whether it contains "DeflAngle:"
  //if it contains the key, and manual mode is active, use getNumberFromString and write the potvalue to the servo
  if (response.indexOf("DeflAngle: ") != -1 && mode == 0) 
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
    
    deflAngle = getNumberFromString(response, "DeflAngle: ");

    Serial.print("DeflAngle extracted from the response: ");
    Serial.println(deflAngle);
    
    // sets the servo position according to the scaled value   
    elevator.write(deflAngle);
    delay(50);
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

      // SEND BACK PITCH RESPONSE TO CONTROLLER
      if (cmdInterval + 1 <= micros()/1.0E6)
      {
        String cmd = "Pitch Angle: " + String(pitchAngle) + " Defl Angle: " + String(deflAngle);
    
        String response = transmit(rf69, rf69_manager, RADIO_RX_ADDRESS, cmd);
        
        cmdInterval = micros()/1.0E6;     
      }

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
    // is that needed? i dont think so...
    //elevator.write(0);
    //delay(15);
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
 * @return MPU pitch in degrees 
 */
att getAttitude() {
    readYPR();
    struct att a;
    a.time = Time;
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

    // store previous values of ypr, vel and time for differentiation
    float yprOld[3];
    float velOld[3];
    copy(ypr, yprOld, 3);
    copy(vel, velOld, 3);
    float timeOld = Time;

    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // get the latest packet
        // Log timestamp
        Time = micros()/1.0E6;
        // Get yaw, pitch, roll from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Differentiate to find velocity and acceleration
        diff(ypr, yprOld, Time, timeOld, vel, 3);
        diff(vel, velOld, Time, timeOld, acc, 3);
    }
}

/**
 * Copies one array to another
 * 
 * @param src source array
 * @param dst destination array
 * @param len length of arrays
 */
void copy(float* src, float* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}

/**
 * Differentiates a quantity
 * 
 * @param currVal pointer to array of values at the current timestep
 * @param lastVal pointer to array of values at the previous timestep
 * @param currTime time at current timestep
 * @param lastTime time at previous timestep
 * @param result pointer to  array for differentiated values to be stored
 * @param len length of the arrays
 */
void diff(float *currVal, float *lastVal, float currTime, float lastTime, float *result, int len) {
    for(int i = 0; i < len; i++) {
        result[i] = (currVal[i] - lastVal[i]) / (currTime - lastTime);
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

float getNumberFromString(String str, String startPhrase) 
{
  //function to extract number from string
  int index = str.indexOf(startPhrase);

  int numberStartIndex = index + startPhrase.length();
  int numberEndIndex = str.length();
  float numberToReturn = 0;

  for (int i = numberStartIndex; i < str.length(); i++) 
  {
    // if not digit and not a minus sign and not dot
    if (!isdigit(str[i]) && str[i] != '-' && str[i] != '.')
    {
      numberEndIndex = i;
      break;
    }
  }

  String numberToReturnStr = str.substring(numberStartIndex, numberEndIndex);
  
  numberToReturn = atof(numberToReturnStr.c_str());
  
  return numberToReturn;
}
