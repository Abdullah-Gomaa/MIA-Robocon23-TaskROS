#include <ros.h>
#include <std_msgs/Float32.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float YawAngle = 0.0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
/*****************************************************************************************************************************************/
// Create an instance of MPU6050
  MPU6050 gyro;

// Create a node handler
  ros :: NodeHandle node;

// Define msg type
  std_msgs :: Float32 msg;


// Inistantiate publisher with topic name YawAngle 
  ros :: Publisher pub_MPU6050("YawAngle", &msg);
/*****************************************************************************************************************************************/
// Functions Prototypes
  void MPU6050_CalculateYawAngle();
  void setupDMP();
/*****************************************************************************************************************************************/
void setup() {
  
// Initialize node 
  node.initNode();
// Advertise to the topic published by pub_MPU6050  
  node.advertise(pub_MPU6050);

  setupDMP();
}
/*****************************************************************************************************************************************/
void loop() {

// Calculate angle  
  MPU6050_CalculateYawAngle();

// Set msg data with angle value 
  msg.data = YawAngle;

// Publish msg  
  pub_MPU6050.publish(&msg);

// Spin Once for Call Backs
  node.spinOnce();
  delay(100);
}

/*****************************************************************************************************************************************/
void MPU6050_CalculateYawAngle()
{
  if (!dmpReady) return;
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            YawAngle = ypr[0] * 180/M_PI;
            
        #endif
}
/*****************************************************************************************************************************************/
void setupDMP()
{
  // put your setup code here, to run once:
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(57600);
    while (!Serial);
    mpu.initialize();
     devStatus = mpu.dmpInitialize();
      // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
