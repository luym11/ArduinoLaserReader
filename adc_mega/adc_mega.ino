#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>

ros::NodeHandle nh; 
std_msgs::Float32 SICK_msg; 
ros::Publisher SICK_time("SICK_time", &SICK_msg); 
std_msgs::Int32 SICK_debug; 
ros::Publisher SICK_distance("SICK_distance", &SICK_debug); 


int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int outputValue = 0; 
float alpha = 0.2; 
long t0 = 0; 
long t1 = 0;
bool t0_set_flag = false;  

void setup() {
  // declare the ledPin as an OUTPUT:
  nh.initNode(); 
  nh.advertise(SICK_time); 
  nh.advertise(SICK_distance);
  Serial.begin(57600);
}

void loop() {
  // read the value from the sensor:
  //delay(5); 
  sensorValue = analogRead(sensorPin);
  outputValue = (1-alpha) * outputValue + alpha * sensorValue; 
  //Serial.println(outputValue);
  SICK_debug.data = (int)millis(); 
  SICK_distance.publish(&SICK_debug); 

  if(outputValue < 900 && t0_set_flag == false){
    t0 = millis(); 
    t0_set_flag = true; 
    //SICK_msg.data = (float)t0;
    //SICK_time.publish(&SICK_msg);
  }
  if(outputValue > 900 && t0_set_flag == true && millis()-t0 > 1500){
    t1 = millis(); 
    t0_set_flag = false; 
    SICK_msg.data = (float)t1-t0;
    SICK_time.publish(&SICK_msg);
  }
  
  nh.spinOnce(); 
  delay(3); 
   
}
