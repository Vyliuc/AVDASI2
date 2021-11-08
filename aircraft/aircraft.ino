#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <String.h>

/*****************************************************************************
 * MPU6050 SETUP                                                             *
 * Adapted from the MPU6050_DMP6.ino example in the i2cdevlib library        *
 * MPU6050 explanation: https://mjwhite8119.github.io/Robots/mpu6050         *
 *****************************************************************************/

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
float time;          // [current time]      timestamp

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
    a.time = time;
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
    float timeOld = time;

    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // get the latest packet
        // Log timestamp
        time = micros()/1.0E6;
        // Get yaw, pitch, roll from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Differentiate to find velocity and acceleration
        diff(ypr, yprOld, time, timeOld, vel, 3);
        diff(vel, velOld, time, timeOld, acc, 3);
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

void setup() {
    Serial.begin(115200);
    while (!Serial);
    mpuSetup();
}

void loop() {
    // Get roll, pitch, yaw from MPU
    att attitude = getAttitude();

    // Display pitch and timestamp
    Serial.print(attitude.pitch,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchVel,4);
    Serial.print(F("\t"));
    Serial.print(attitude.pitchAcc,4);
    Serial.print(F("\t"));
    Serial.println(attitude.time,6);
}
