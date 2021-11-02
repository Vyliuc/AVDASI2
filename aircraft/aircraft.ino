#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <Wire.h>
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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t dmpStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]        quaternion container
VectorFloat gravity; // [x, y, z]           gravity vector
float ypr[3];        // [yaw, pitch, roll]  yaw/pitch/roll container and gravity vector

// struct for attitude output
struct att {
    float time = micros() / 1.0E6; // time since program started
    float yaw;                           // yaw in degrees
    float pitch;                         // pitch in degrees
    float roll;                          // roll in degrees
};

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

  Serial.begin(115200);

  mpuSetup();

  elevator.attach(SERVO_PIN);
}

void loop() {
  // Get roll, pitch, yaw from MPU
  att attitude = getAttitude();

  // Display pitch and timestamp
  Serial.print(attitude.yaw);
  Serial.print(F("\t"));
  Serial.print(attitude.pitch);
  Serial.print(F("\t"));
  Serial.print(attitude.roll);
  Serial.print(F("\t"));
  Serial.println(attitude.time,6);

  //receive values for pitch data - WAITING ON HAMISH
  float ang = attitude.pitch;
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
    struct att attitude;
    attitude.yaw = ypr[0] * 180.0/M_PI;
    attitude.pitch = ypr[1] * 180.0/M_PI;
    attitude.roll = ypr[2] * 180.0/M_PI;
    return attitude;
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
        // Get yaw, pitch, roll from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetEuler(ypr, &q);
    }
}

void initSD()
{
  Serial.begin(9600);
  Serial.println("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("Initialization failed!");
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