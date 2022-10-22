#include <ros.h>
#include <std_msgs/Int32.h>

#define encoderPinA    PA8
#define encoderPinB    PA9
/*****************************************************************************************************************************************/
// Create a node handler
  ros :: NodeHandle node;

// Define msg type
  std_msgs :: Int32 msg;


// Inistantiate publisher with topic name Encoder_Counts  
  ros :: Publisher pub_enc("Encoder_Counts", &msg);

  HardwareSerial Serial3(PB11, PB10); 
/*****************************************************************************************************************************************/
// Declare counts variable
int32_t count = 0;
/*****************************************************************************************************************************************/
void setup() {
  Serial3.begin(115200);
  (node.getHardware())-> setPort(&Serial3);
  (node.getHardware())-> setBaud(115200);
// Initialize node 
  node.initNode();
// Advertise to the topic published by pub_enc
  node.advertise(pub_enc);

// Setting pins directions
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

// Set interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA) , &encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB) , &encoderB, CHANGE);
  
}
/*****************************************************************************************************************************************/
void loop() {
// Set msg data with angle value 
  msg.data = count;

// Publish msg  
  pub_enc.publish(&msg);

// Spin Once for Call Backs
  node.spinOnce();
}
/*****************************************************************************************************************************************/
// ISR for encoder PinA
void encoderA()
{
  if( (digitalRead(encoderPinA) != (digitalRead(encoderPinB) )))
    count++;  // CW
  else
    count--;  // CCW
}
// ISR for encoder PinB
void encoderB()
{
  if( (digitalRead(encoderPinA) != (digitalRead(encoderPinB) )))
    count--;  // CCW
  else
    count++; // CW
}
/*****************************************************************************************************************************************/
