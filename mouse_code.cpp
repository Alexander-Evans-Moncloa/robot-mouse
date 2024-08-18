// Define pins for left and right motors
const int leftMotorPin = 5; //1217 rpm 
const int rightMotorPin = 6; //1203 rpm 

// Define pins for left and right sensors
const int leftSensorPin = A0;
const int rightSensorPin = A1;

// Define variables for PID controller
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;
int timer = 0;

// Define variables for sensor readings
float leftSensorValue = 0;
float rightSensorValue = 0;
bool tiltSwitch = 0;
bool firstSwitch = 0;

// define base movment speed
int cruseThrottle = 90; //80 is possable with fresh batteries and 150 on the straight

//create system timer
int time = millis();

void setup() {
  // Set the motor pins as outputs
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(13, INPUT);
  // Initialize the serial monitor
  Serial.begin(9600);
}

void loop() {
  // Define constants for PID controller
  float kp = 0.15; //works at 7.8v
  float ki = 0.0; 
  float kd = 0.3; 
  // Read the sensor values
  //leftSensorValue = int(sqrt(1024.0/float(analogRead(leftSensorPin)+1))*1024.0);
  //rightSensorValue = int(sqrt(1024.0/float(analogRead(rightSensorPin)+1))*1024.0);
  leftSensorValue = sqrt(analogRead(leftSensorPin)+1)*40;
  rightSensorValue = sqrt(analogRead(rightSensorPin)+1)*40;
  tiltSwitch = ! digitalRead(13);

  // Calculate the error
  error = rightSensorValue - leftSensorValue + 47;

  // Calculate the integral and derivative
  integral += error;
  derivative = error - previousError;

  // Calculate the PID output
  pidOutput = kp * error + kd * derivative + ki * integral;

  // Update the previous error
  previousError = error;

  //set value of throttle for this loop
  int throttle = cruseThrottle;

  //tilt ball switch latching timer
  if (tiltSwitch){
    firstSwitch = 1;
    timer = 50;
  }

  //decrement tilt ball switch timer
  if (timer > 0){
    throttle = 200;
    timer = timer - 1;
  }
  
  // Set the motor speeds based on the PID output
  int leftMotorSpeed = (throttle - pidOutput);
  int rightMotorSpeed = (throttle + pidOutput);

  //ensure motor values are valid
  leftMotorSpeed = abs(leftMotorSpeed);
  rightMotorSpeed = abs(rightMotorSpeed);

  // Set the motor speeds using PWM
  analogWrite(leftMotorPin, leftMotorSpeed);
  analogWrite(rightMotorPin, rightMotorSpeed);

  // Print the sensor values and PID output to the serial monitor
  /*
  Serial.print("Left sensor value: ");
  Serial.print(leftSensorValue);
  Serial.print(",");
  Serial.print("\tRight sensor value: ");
  Serial.println(rightSensorValue);
  /*
  Serial.print(",PIDoutput ");
  Serial.print(pidOutput);
  Serial.print(",leftmotorspeed ");
  Serial.print(leftMotorSpeed);
  Serial.print(",rightmotorspeed ");
  Serial.print(rightMotorSpeed);
  Serial.println();
  Serial.println(error);
  */
  
  // Delay for a short period of time
  delay(1);
}
