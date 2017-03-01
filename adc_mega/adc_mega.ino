#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>

#define ERR_THR 1500
#define VALVE_THR 800
#define DELAY_MILI 2
#define SMOOTH_PARAM 20

ros::NodeHandle nh;  
std_msgs::Int32 SICK_debug; 
std_msgs::Float32 SICK_rdistance; 
ros::Publisher SICK_distance("SICK_distance", &SICK_debug); 
ros::Publisher SICK_realdistance("SICK_realdistance", &SICK_rdistance);

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;   
int outputValue = 0; 
int lastValues[20]; 
int counter = 0; 

void setup() {
  // declare the ledPin as an OUTPUT:
  nh.initNode(); 
  nh.advertise(SICK_distance);
  nh.advertise(SICK_realdistance);
  Serial.begin(57600);
}

void loop() {
  
  sensorValue = analogRead(sensorPin);
 
  //Serial.println(outputValue);

  if(counter < SMOOTH_PARAM){
    lastValues[counter] = sensorValue; 
    outputValue = sensorValue; 
   }else{
    lastValues[counter%SMOOTH_PARAM] = sensorValue; 
    outputValue = 0;
    for(int i = 0; i < SMOOTH_PARAM; i++){
      outputValue += lastValues[i]; 
    }
    outputValue /= SMOOTH_PARAM; 
   }
  counter++; 
  
if(counter == 2 * SMOOTH_PARAM){ counter = SMOOTH_PARAM; }
  
  // 5cm to 100cm, 4mA-20mA, 200-1023.
  //8.895 for 1 cm
  SICK_debug.data = (int)outputValue; 
  SICK_rdistance.data = ((int)sensorValue-200)/8.895 + 5.0;
  SICK_distance.publish(&SICK_debug); 
  SICK_realdistance.publish(&SICK_rdistance); 
  
  nh.spinOnce(); 
  delay(DELAY_MILI); 
   
}
