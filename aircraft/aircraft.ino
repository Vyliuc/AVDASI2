#include <SD.h>
#include <SPI.h>
#include <transceiver.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define INTERRUPT_PIN        2 // TODO: check this value

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
}

void loop() {
  // Display roll, pitch, yaw from MPU
  mpuLoop();

  // constantly listen to the transceiver & check if any data has been received
  String response = receive(rf69, rf69_manager);

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
