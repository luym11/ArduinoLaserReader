//------------------------------------------------------------------------------------------------------------------------

//  Push button feedback code for autonomous aerial grasping; MBZIRC Challenge 3 
//  Authors: Usman Amin Fiaz & Mohammed Abdel Kader
//  Last Updated: January 27, 2017

//------------------------------------------------------------------------------------------------------------------------

#include <ArduinoHardware.h>  //  ROS Header files
#include <ros.h>
#include <ros/time.h> //  Required for timing
#include <std_msgs/Bool.h>  // Bool type header file

//------------------------------------------------------------------------------------------------------------------------

ros::NodeHandle  pushButton;  //  Define the ROS Node
std_msgs::Bool decision_msg;  //  Declare std_msgs for both subscriber and publisher
std_msgs::Bool drop_signal_msg; 

//------------------------------------------------------------------------------------------------------------------------

bool new_msg = false; // New incoming message confirmation

void sub(const std_msgs::Bool& incoming_msg)  //  Call back subscribing function
{
   drop_signal_msg.data = incoming_msg.data;
   new_msg = true;  //  New message confirmation set to "1"
}

//------------------------------------------------------------------------------------------------------------------------

ros::Subscriber<std_msgs::Bool> drop_signal("/Quad1/gripper_command", &sub); //  Subscriber declaration
ros::Publisher decision("/Quad1/gripper_status",&decision_msg);  //  Publisher "gripper_status" declaration

// Quad2 for 2nd quadcopter and Quad3 for Quadcopter 3

//------------------------------------------------------------------------------------------------------------------------

// Define Digital input pins for push buttons
int pushButton1_1 = 22; //  Feedback Push buttons for EPM 1
int pushButton1_2 = 24;
int pushButton1_3 = 26;
int pushButton1_4 = 28;

int pushButton2_1 = 32; //  Feedback Push buttons for EPM 2
int pushButton2_2 = 34;
int pushButton2_3 = 36;
int pushButton2_4 = 38;

int pushButton3_1 = 42; //  Feedback Push buttons for EPM 3
int pushButton3_2 = 44;
int pushButton3_3 = 46;
int pushButton3_4 = 48;

int button_status_1;  //  Accumulative status of all 4 buttons for EPM 1
int button_status_2;  //  Accumulative status of all 4 buttons for EPM 2
int button_status_3;  //  Accumulative status of all 4 buttons for EPM 3

int button_status;  //Accumulative status of all 3 EPMs

int PWM1 = 3; //  Define PWM Signal pin for EPMs
int PWM2 = 4;
int PWM3 = 5;

//------------------------------------------------------------------------------------------------------------------------

const float DC = 20; // Total time period of the PWM signal @ 50 Hz
const float PWM_ON = 2; // Range: 1.75 - 2.25 ms
const float PWM_NEU = 1.5; // Neutral Inactive Range/:
const float PWM_OFF = 1.1; // Range: 0.75 - 1.25 ms

//------------------------------------------------------------------------------------------------------------------------

// Drop routine:
void drop() 
{
  
  // Deactivate the Electropermanent magnet
  digitalWrite(PWM1, HIGH);
  //digitalWrite(PWM2, HIGH);
  //digitalWrite(PWM3, HIGH);
  delay(PWM_OFF);
  digitalWrite(PWM1, LOW);
  //digitalWrite(PWM2, LOW);
  //digitalWrite(PWM3, LOW);
  delay((DC - PWM_OFF));
  delay(300);
  
  digitalWrite(PWM1, HIGH);
  digitalWrite(PWM2, HIGH);
  digitalWrite(PWM3, HIGH);
  delay(PWM_NEU);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(PWM3, LOW);
  delay((DC - PWM_NEU));
  delay(300);
}

//------------------------------------------------------------------------------------------------------------------------
//  Pick up routine:

void pick()
{
  digitalWrite(PWM1, HIGH);
  digitalWrite(PWM2, HIGH);
  digitalWrite(PWM3, HIGH);
  delay(PWM_ON);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(PWM3, LOW);
  delay((DC - PWM_ON));
  delay(300);
      
  digitalWrite(PWM1, HIGH);
  digitalWrite(PWM2, HIGH);
  digitalWrite(PWM3, HIGH);
  delay(PWM_NEU);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(PWM3, LOW);
  delay((DC - PWM_NEU));
  delay(300);
}

//------------------------------------------------------------------------------------------------------------------------

// the setup routine runs once when you press reset:
void setup() 
{
  // initialize serial communication at 115200 per second:
  //Serial.begin(115200);
  pushButton.getHardware()->setBaud(115200);
  pushButton.initNode();
  pushButton.advertise(decision);
  pushButton.subscribe(drop_signal);

  
  // Label the pushbutton pins an inputs:
  pinMode(pushButton1_1, INPUT);
  pinMode(pushButton1_2, INPUT);
  pinMode(pushButton1_3, INPUT);
  pinMode(pushButton1_4, INPUT);
  pinMode(pushButton2_1, INPUT);
  pinMode(pushButton2_2, INPUT);
  pinMode(pushButton2_3, INPUT);
  pinMode(pushButton2_4, INPUT);
  pinMode(pushButton3_1, INPUT);
  pinMode(pushButton3_2, INPUT);
  pinMode(pushButton3_3, INPUT);
  pinMode(pushButton3_4, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);

// Activate the Electropermanent magnet by default

  digitalWrite(PWM1, HIGH);
  digitalWrite(PWM2, HIGH);
  digitalWrite(PWM3, HIGH);
  delay(PWM_ON);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(PWM3, LOW);
  delay((DC - PWM_ON));
  delay(300);

  drop_signal_msg.data = false; //  Reset the subscriber
}

//------------------------------------------------------------------------------------------------------------------------

// the loop routine runs over and over again forever:
long t1=millis();
long t2=millis();
long delay_r = 20; // [millis]

void loop() 
{
  t1 = millis();

    if (new_msg && drop_signal_msg.data == false)
    { 
      drop_signal_msg.data = false;
      drop();
    
      new_msg = false;
    }
    if(new_msg && drop_signal_msg.data == true)
    {
      pick();
      new_msg = false;
    }
  
    // read the input pins from push buttons:
    
    int buttonState1_1 = (digitalRead(pushButton1_1));
    int buttonState1_2 = (digitalRead(pushButton1_2));
    int buttonState1_3 = (digitalRead(pushButton1_3));
    int buttonState1_4 = (digitalRead(pushButton1_4));
    
    int buttonState2_1 = (digitalRead(pushButton2_1));
    int buttonState2_2 = (digitalRead(pushButton2_2));
    int buttonState2_3 = (digitalRead(pushButton2_3));
    int buttonState2_4 = (digitalRead(pushButton2_4));
  
    int buttonState3_1 = (digitalRead(pushButton3_1));
    int buttonState3_2 = (digitalRead(pushButton3_2));
    int buttonState3_3 = (digitalRead(pushButton3_3));
    int buttonState3_4 = (digitalRead(pushButton3_4));
    
    // Add all buttons status in one value
    int button_status1 = buttonState1_1 + buttonState1_2 + buttonState1_3 + buttonState1_4;
    int button_status2 = buttonState2_1 + buttonState2_2 + buttonState2_3 + buttonState2_4;
    int button_status3 = buttonState3_1 + buttonState3_2 + buttonState3_3 + buttonState3_4;
  
    //Decision Logic
    //Decision Rules for individual EPMs
  
  // EPM 1
  
    switch (button_status1)
    {
      case 0:
        button_status_1 = 0;
        break;
      case 1:
        button_status_1 = 1;
        break;
      case 2:
        button_status_1 = 1;
        break;
      case 3:
        button_status_1 = 2;
        break;
      case 4:
        button_status_1 = 3;
        break;
      default:
        button_status_1 = 0;
        break;
    }  
  
  // EPM 2
  
     switch (button_status2)
    {
      case 0:
        button_status_2 = 0;
        break;
      case 1:
        button_status_2 = 1;
        break;
      case 2:
        button_status_2 = 1;
        break;
      case 3:
        button_status_2 = 2;
        break;
      case 4:
        button_status_2 = 3;
        break;
      default:
        button_status_2 = 0;
        break;
    } 
  
  //EPM 3
  
     switch (button_status3)
    {
      case 0:
        button_status_3 = 0;
        break;
      case 1:
        button_status_3 = 1;
        break;
      case 2:
        button_status_3 = 1;
        break;
      case 3:
        button_status_3 = 2;
        break;
      case 4:
        button_status_3 = 3;
        break;
      default:
        button_status_3 = 0;
        break;
    } 
  
    // Decision Rule for all three EPMs
    button_status = button_status_1 + button_status_2 + button_status_3;
  
    if (button_status >= 1)
      decision_msg.data = 1; // buttons pushed, pick up confirmation
    else
      decision_msg.data = 0;

    decision.publish(&decision_msg); // Publish the decision message
    
    pushButton.spinOnce();

    t2 = millis();

    while( (t2 - t1) < delay_r){
      t2 = millis();
    }

//------------------------------------------------------------------------------------------------------------------------

//  Debugging Serial Monitor Code

//  {
  // print out the state of the buttons:
  //Serial.println("PB1\tPB2\tPB3");
/*
Serial.print(buttonState1_1);
Serial.print("\t");
Serial.print(buttonState1_2);
Serial.print("\t");
Serial.print(buttonState1_3);
Serial.print("\t");
Serial.print(buttonState1_4);
Serial.println();
Serial.print(buttonState2_1);
Serial.print("\t");
Serial.print(buttonState2_2);
Serial.print("\t");
Serial.print(buttonState2_3);
Serial.print("\t");
Serial.print(buttonState2_4);
Serial.println();
Serial.print(buttonState3_1);
Serial.print("\t");
Serial.print(buttonState3_2);
Serial.print("\t");
Serial.print(buttonState3_3);
Serial.print("\t");
Serial.print(buttonState3_4);
Serial.println();


Serial.println();

Serial.println(decision_msg.data);
Serial.println();
Serial.println();
delay(50);       // delay in between reads for stability
*/

//------------------------------------------------------------------------------------------------------------------------

}
