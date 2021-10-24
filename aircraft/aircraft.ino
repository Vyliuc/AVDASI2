#include <transceiver.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     420

void setup() {
  transceiver_setup(RADIO_TX_ADDRESS);
  mpuSetup();
}

void loop() {
  // constantly listen to the transceiver & check if any data has been received
  String response = receive();

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


void mpuSetup() {
  MPU6050 mpu;

  #define OUTPUT_READABLE_YAWPITCHROLL
  #define INTERRUPT_PIN 2 // TODO: check this value
  #define LED_PIN 11 // TODO: check this value
  bool blinkState = false;

  // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
}