/* 
 *  @ ros command
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600 (for Nano)
 */


#include <Servo.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

Servo servoSteer;
Servo servoThrottle;

// Pin number
const byte interruptPinSteer    = 2;
const byte interruptPinThrottle = 3;

int pinSteer    = 4;
int pinThrottle = 5;

// Params
volatile unsigned long pwm_values[2] = {1500, 1500}; // {steer, throttle}
volatile unsigned long timer[2] = {0, 0};

int steerCmd = 1500; // filtered steer command
int throttleCmd = 1500; // filtered throttle command
int autoSteer = 1500;   // steer command from ros
int autoThrottle = 1500;// throttle command from ros

bool autoMode = false;
int gear = 0;      // 1: forward / 0: neutral / -1: backward

// ROS Callback
void callbackAutoMode(const std_msgs::Bool& msg) {
  autoMode = msg.data;
}
void callbackAutoSteer(const std_msgs::Int16& msg) {
  autoSteer = msg.data;
}
void callbackAutoThrottle(const std_msgs::Int16& msg) {
  autoThrottle = msg.data;
}

// ROS
ros::NodeHandle nh;

std_msgs::Int16 rcCmdSteer;
std_msgs::Int16 rcCmdThrottle;
std_msgs::Int16 gearNum;       // for forward/neutral/backward

ros::Subscriber<std_msgs::Bool> subAutoMode("/auto_mode", callbackAutoMode);
ros::Subscriber<std_msgs::Int16> subAutoSteer("/auto_cmd/steer", callbackAutoSteer);
ros::Subscriber<std_msgs::Int16> subAutoThrottle("/auto_cmd/throttle", callbackAutoThrottle);
ros::Publisher pubGearNum("/car/gear_num", &gearNum);
ros::Publisher pubRcCmdSteer("/rc_cmd/steer", &rcCmdSteer);
ros::Publisher pubRcCmdThrottle("/rc_cmd/throttle", &rcCmdThrottle);

void setup() {
  // Serial communication
  Serial.begin(57600); // nan0: 57600, uno: 115200, 9600

  // ROS
  nh.initNode();
  nh.subscribe(subAutoMode);
  nh.subscribe(subAutoSteer);
  nh.subscribe(subAutoThrottle);
  nh.advertise(pubRcCmdSteer);
  nh.advertise(pubRcCmdThrottle);
  nh.advertise(pubGearNum);
  
  // Servo attach - servo motor
  servoSteer.attach(pinSteer);
  servoSteer.writeMicroseconds(1500); // set servo to mid point
  // Servo attach - DC motor
  servoThrottle.attach(pinThrottle);
  servoThrottle.writeMicroseconds(1500); // set dc stop
  
  // Interupt (for transmitter)
  attachInterrupt(digitalPinToInterrupt(interruptPinSteer), calcSteerPWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinThrottle), calcThrottlePWM, CHANGE);

}

void loop() {
  // Check Gear number (State)
  
  
  if (autoMode == true) {
    // Autonomous Control
    
    // ----- Auto Steer ----- //
    servoSteer.writeMicroseconds(min(1900, max(1100, autoSteer)));   // steer
    
    // ----- Auto Throttle ----- //
    if (abs(autoThrottle - 1500) < 20) {
      autoThrottle = 1500;
    }
    servoThrottle.writeMicroseconds(min(1900, max(1100, autoThrottle))); // throttle
  }
  else {
    // Manual Control : Transmitter > Arduino > Motor
    // ----- Steer command ----- //
    steerCmd = pwm_values[0];
    servoSteer.writeMicroseconds(min(1900, max(1100, steerCmd))); // steer
    
    // ----- Throttle command ----- //
    throttleCmd = pwm_values[1];
    // Dead zone
    if (abs(throttleCmd - 1500) < 20) {
      throttleCmd = 1500;
    }
    servoThrottle.writeMicroseconds(min(1900, max(1100, throttleCmd))); // throttle
    
    // Publish
    rcCmdSteer.data = steerCmd;
    rcCmdThrottle.data = throttleCmd;
    pubRcCmdSteer.publish(&rcCmdSteer);
    pubRcCmdThrottle.publish(&rcCmdThrottle);
  }

  nh.spinOnce();
  delay(10);
}


void calcSteerPWM(){
  if(digitalRead(interruptPinSteer) == HIGH) {
    timer[0] = micros();
  }
  else {
    if(timer[0] != 0) {
      pwm_values[0] = micros() - timer[0];
    }
  }
}

void calcThrottlePWM(){
  if(digitalRead(interruptPinThrottle) == HIGH) {
    timer[1] = micros();
  }
  else {
    if(timer[1] != 0) {
      pwm_values[1] = micros() - timer[1];
    }
  }
}

/* Visualization */
void duration() {
  Serial.println("Steer : " + String(pwm_values[0]) + " Throttle : " + String(pwm_values[1]));
}