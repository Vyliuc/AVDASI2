#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>
#include <String.h>
#include <PID.h>

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

// 0 - controlled
// 1 - neutral
// 2 - manual deflection
int mode = 1;

double servoOffset = 90;

double deflAngle = servoOffset;
double refPitchAngle = servoOffset;
double currentPitchAngle = servoOffset;
double Kp = servoOffset + 1;
double Ki = servoOffset;
double Kd = servoOffset;

double pitchAngleTolerance = 0.5;

double angvel = 0;
double angacc = 0;

float lastTimeTaken = micros()/1.0E6;

Servo elevator;
PID controlPID(&currentPitchAngle, &deflAngle, &refPitchAngle, Kp-servoOffset, Ki-servoOffset, Kd-servoOffset, DIRECT);

File logsFile;

void setup() 
{
  Serial.begin(115200);

  // initialize the SD card
  initSD();

  // initialize the radio
  transceiverSetup(rf69, rf69_manager);
  
  // initialize the MPU unit
  mpuSetup();

  // initialize the Servo motor
  elevator.attach(SERVO_PIN);
  // initisl deflection 90 degrees
  elevator.write(90);

  // initialize the PID controller
  controlPID.SetMode(AUTOMATIC);
}

void loop() {
  // constantly listen to the transceiver & check if any data has been received
  // also send back the acknowledgment with Pitch and Defl. angles
  String response = receive(rf69, rf69_manager, currentPitchAngle, deflAngle);
  
  // set the mode
  if (response.indexOf("Controlled Activated!") != -1) 
  {
    mode = 0;

    // set gains to default values
    Kp = servoOffset + 1;
    Kd = servoOffset;
    Ki = servoOffset;
    
    logToSD("\nCONTROLLED MODE\n");
    Serial.println("On Controlled");
  }
  else if (response.indexOf("Manual Defl Activated!") != -1) 
  {
    mode = 2;
    logToSD("\nMANUAL DEFLECTION MODE\n");
    Serial.println("On Manual Deflection");
  }
  else if (response.indexOf("Neutral Mode Activated!") != -1) 
  {
    mode = 1;
    logToSD("\nNEUTRAL MODE\n");
    Serial.println("On Neutral");
  }

  // if Controlled or Manual Deflection mode
  // monitor MPU data, log it
  if (mode == 0 || mode == 2)
  {
    // Get roll, pitch, yaw from MPU
    att attitude = getAttitude();

    currentPitchAngle = servoOffset + attitude.pitch;
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

    // log time, pitch, vel, acc
    String mpuData = String(attitude.time) +
    ":    PITCH: " + String(currentPitchAngle) +
    ",    VEL: " + String(angvel) +
    ",    ACC: " + String(angacc) +
    ",    DEFL: " + String(deflAngle) +
    ",    Kp: " + String(Kp) +
    ",    Ki: " + String(Ki) +
    ",    Kd: " + String(Kd);

  /*  if (currentPitchAngle > 0) currentPitchAngleNegative = true;
  else currentPitchAngleNegative = false;

  if (deflAngle > 0 ) deflAngleNegative = true;
  else deflAngleNegative = false;

  if (refPitchAngle > 0) refPitchAngleNegative = true;
  else refPitchAngleNegative = false;
  */
    
    logToSD(mpuData);
  }
  
  // CONTROLLED MODE
  if (mode == 0)
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

    if (response.indexOf("Ref Pitch: ") != -1) // if Ref. Pitch has been sent
    {
      refPitchAngle = getNumberFromString(response, "Ref Pitch: ");

      String logMsg = "\nNEW REF. PITCH: " + String(refPitchAngle) + "\n";
      logToSD(logMsg);

      Serial.print("Received Ref. Pitch: ");
      Serial.println(refPitchAngle);
    }
    else if (response.indexOf("Kp: ") != -1) // if P Gain has been sent
    {
      Kp = getNumberFromString(response, "Kp: ");

      controlPID.SetTunings(Kp, Ki, Kd);
      
      String logMsg = "\nNEW Kp: " + String(Kp)+ "\n";
      logToSD(logMsg);

      Serial.print("Received Kp: ");
      Serial.println(Kp);
    }
    else if (response.indexOf("Ki: ") != -1) // if I Gain has been sent
    {
      Ki = getNumberFromString(response, "Ki: ");

      controlPID.SetTunings(Kp, Ki, Kd);

      String logMsg = "\nNEW Ki: " + String(Ki)+ "\n";
      logToSD(logMsg);

      Serial.print("Received Ki: ");
      Serial.println(Ki);
    }
    else if (response.indexOf("Kd: ") != -1) // if D Gain has been sent
    {
      Kd = getNumberFromString(response, "Kd: ");

      controlPID.SetTunings(Kp, Ki, Kd);

      String logMsg = "\nNEW Kd: " + String(Kd) + "\n";
      logToSD(logMsg);

      Serial.print("Kd: ");
      Serial.println(Kd);
    }

    // DEFLECT THE ELEVATOR TO MAINTAIN THE REF. PITCH ANGLE
    // ALSO APPLY SOME TOLERANCE TO THE FINAL PITCH ANGLE (AFTER DISTURBANCES)

    // calculate error (in this case, excess moment)
    //error = (p_dyn*xtail*St*((at*(currentPitchAngle+it+((angvel*xtail)/V)))+(adel*deflAngle))) + (xg*m*g)- M*angacc;
    
    // while the pitch angle varies, calculate the corresponding elevator deflection
    if (currentPitchAngle > refPitchAngle + pitchAngleTolerance || currentPitchAngle < refPitchAngle - pitchAngleTolerance) 
    {
      //deflAngle = (((M*angacc)/(p_dyn*xtail*St))-((xg*m*g)/(p_dyn*xtail*St))-(at*currentPitchAngle)-(at*((angvel*xtail)/(pow(V,2))))-(at*it))/adel;
      
      // run PID control
      controlPID.Compute();

      Serial.print("Defl angle needed: ");
      Serial.println(deflAngle);
    }
 
    if (abs(deflAngle) <= lim) 
    {
      elevator.write(deflAngle);
      delay(50);
    }
    else if (deflAngle > lim) 
    {
      elevator.write(lim);
      delay(50);
    }
    else if (deflAngle < (-1*lim)) 
    {
      elevator.write((-1*lim));
      delay(50);
    }
  } 

  // MANUAL DEFLECTION MODE
  if (mode == 2)
  {
    if (response.indexOf("Deflection: ") != -1) // if Defl. Angle has been sent
    {
      deflAngle = getNumberFromString(response, "Deflection: ");
      String logMsg = "\nNEW DEFLECTION SETTING: " + String(deflAngle)+ "\n";
      logToSD(logMsg);

      Serial.print("Received Deflection: ");
      Serial.println(deflAngle);

      // sets the servo position  
      elevator.write(deflAngle);
      delay(50);
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
    mpu.setXAccelOffset(-601);
    mpu.setYAccelOffset(1457);
    mpu.setZAccelOffset(1226);
    mpu.setXGyroOffset(102);
    mpu.setYGyroOffset(38);
    mpu.setZGyroOffset(5);

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
    delay(15);
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
