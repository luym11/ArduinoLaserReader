
int sensorPin = A1;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  
  Serial.begin(57600);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
 
  Serial.println(sensorValue);
   
}
