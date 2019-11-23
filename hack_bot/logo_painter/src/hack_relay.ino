
/*
 * Hack Day Relay Board script
 */
#include <ros.h>
#include <std_srvs/SetBool.h>

// Paint pins
const int PAINT_ON = 5;
const int PAINT_OFF = 6;

// Encoder pins
const int ENCODER_A = 2; // interrupt pin
const int ENCODER_B = 8;

ros::NodeHandle nh;
void paint_cb(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> server("paint",&paint_cb);

// Encoder counts
volatile unsigned long count = 0;
int prev_count = 0;
int count_changed = 0;
float N = 800; // ???? TBD Pulse per revolution (PPR) from motor specs

void paint_cb(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
{
  // Either turn motor forward or backward
  if (req.data)
  {
    digitalWrite(PAINT_ON, LOW);
    // Monitor encoders to write HIGH
    res.message = "PAINT ON!";
  } else
  {
    digitalWrite(PAINT_OFF, LOW);
    // Monitor encoders to write HIGH
    res.message = "PAINT OFF!";
  }
  res.success = true;
}

void encoderEvent2x()  // 2X Encoder resolution
{
  if (digitalRead(ENCODER_B) == 0) {
    if (digitalRead(ENCODER_A) == 0) {
      // A fell, B is LOW
      count--; // Moving reverse
    } else {
      // A rose, B is LOW
      count++; // Moving reverse
    }
  } else {
    if (digitalRead(ENCODER_A) == 0) {
      // A fell, B is HIGH
      count++; // Moving reverse
    } else {
      // A rose, B is HIGH
      count--; // Moving forward
    }
  }
  count_changed = 1;
}
//####################################################
void setup() {
  // ROS
  nh.initNode();
  nh.advertiseService(server);

  // Pins
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(PAINT_ON, OUTPUT);
  pinMode(PAINT_OFF, OUTPUT);

  // Init interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderEvent2x, CHANGE);

}

void loop() {
  nh.spinOnce();
  delay(100);
}