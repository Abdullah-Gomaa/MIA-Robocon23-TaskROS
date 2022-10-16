#include <ros.h>
#include <std_msgs/Float32.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define NoOfSamples             1000
#define gyroFullScaleRange      MPU6050_GYRO_FS_2000

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
// Variables for angle calibration and calculations
  int16_t gyroZ = 0;
  int16_t gyroZOffset = 0;
  float YawAngle = 0.0;
  unsigned long prevT = 0;
  unsigned long currentT = 0;
  float deltaT = 0.0;
/*****************************************************************************************************************************************/
// Functions Prototypes
  void MPU6050_CalculateOffsetGyroZ();
  void MPU6050_CalculateYawAngle();
/*****************************************************************************************************************************************/
void setup() {
  
// Initialize node 
  node.initNode();
// Advertise to the topic published by pub_MPU6050  
  node.advertise(pub_MPU6050);

// Begin I2C communitcation
  Wire.begin();
  
// Initialize gyro
  gyro.initialize();
  
// Set Full Scale Range -2000 ---> +2000 deg/s  
  gyro.setFullScaleGyroRange(gyroFullScaleRange);

// Calculate Offset for gyroZ  
  MPU6050_CalculateOffsetGyroZ();
  delay(5);
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
void MPU6050_CalculateOffsetGyroZ()
{
  for(int i=0; i< NoOfSamples; i++)
  {
    gyroZ = gyro.getRotationZ();
    gyroZOffset += gyroZ; 
  }
  gyroZOffset /= NoOfSamples;
}
/*****************************************************************************************************************************************/
void MPU6050_CalculateYawAngle()
{
  // Time measurements to calculate angle
  prevT =  currentT;
  currentT = millis();
  deltaT = (currentT - prevT)*1000;
  // Reading the gyroZ register value
  gyroZ = gyro.getRotationZ();
  // Calculate yaw angle
  YawAngle = ((gyroZ * gyroFullScaleRange ) / 32786)*deltaT;
}
/*****************************************************************************************************************************************/
