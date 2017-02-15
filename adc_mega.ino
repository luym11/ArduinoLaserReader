
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int outputValue = 0; 
float alpha = 0.2; 

void setup() {
  // declare the ledPin as an OUTPUT:
  
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  //delay(5); 
  sensorValue = analogRead(sensorPin);
  outputValue = (1-alpha) * outputValue + alpha * sensorValue; 
  Serial.println(outputValue);
   
}
