#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh; 
std_msgs::Int32 SICK_msg; 
ros::Publisher SICK_sensor("SICK_sensor", &SICK_msg); 


int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int outputValue = 0; 
float alpha = 0.2; 

void setup() {
  // declare the ledPin as an OUTPUT:
  nh.initNode(); 
  nh.advertise(SICK_sensor); 
  Serial.begin(57600);
}

void loop() {
  // read the value from the sensor:
  //delay(5); 
  sensorValue = analogRead(sensorPin);
  outputValue = (1-alpha) * outputValue + alpha * sensorValue; 
  //Serial.println(outputValue);
  SICK_msg.data = outputValue; 
  SICK_sensor.publish(&SICK_msg);
  nh.spinOnce(); 
  delay(2); 
   
}
