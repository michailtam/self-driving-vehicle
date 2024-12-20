/*
In this project a self driving car gets programmed which bypasses obstacles.
*/

// Left motor
const int enable1 = 3;  // Speed control pin used to turn on/off the motor and control its speed
const int input1 = 2;   // IN1 Control the spinning direction of the motor
const int input2 = 4;   // IN2 Control the spinning direction of the motor 

// Right motor
const int enable2 = 6;  // Speed control pin used to turn on/off the motor and control its speed
const int input3 = 7;   // IN3 Control the spinning direction of the motor
const int input4 = 8;   // IN4 Control the spinning direction of the motor

// Ultrasonic sensor
const int sensor_trig = 10;  // Trigger pin of the sensor (transmitter)
const int sensor_echo = 11;  // Echo pin of the sensor (receiver)

// Status LED's
const int red_led = 12;     // Indicates that the robot has detected an obstacle
const int blue_led = 13;    // Indicates that the robot is moving

float time_ = 0.0;  // The time the ultrasound signal travels and returns
float current_distance = -1; // The calculated distance to the obstacle (if there is any)
float ultrasound = 0.034;  // Ultrasounds are traveling with the speed of sound (i.e. 340 m/s = 0.034 cm/μs) 
bool obstacle_detected = false; // Flag which determines if there is an obstacle

const int DEGREES_45 = 220; // Match delay to degrees (NOTE: this is only an approximation)


void setup() {
  // Configure the left motor and turn it off
  pinMode(enable1, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);

  // Configure the right motor and turn it off
  pinMode(enable2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  digitalWrite(input3, LOW);
  digitalWrite(input4, LOW);

  // Configure the Ultrasonic sensor and set it's state to off
  pinMode(sensor_trig, OUTPUT);
  pinMode(sensor_echo, INPUT);
  digitalWrite(sensor_trig, LOW);
  
  // Configure the status LED's and turn them off
  pinMode(red_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  digitalWrite(red_led, LOW);
  digitalWrite(blue_led, LOW);

  Serial.begin(9600); // Enable printing via serial port
}

// Controls the LED's 
void indication()
{
  if (obstacle_detected) {
    // Set the LED status to STOP
    digitalWrite(red_led, HIGH);  
    digitalWrite(blue_led, LOW);
  }
  else {
    // Set the LED status to MOVE
    digitalWrite(red_led, LOW);  
    digitalWrite(blue_led, HIGH);
  }
}

// Manages the sensor readings
float readSensor()
{
  // Generate a wave sound that will be bounced back by a near object (if thwere is any) 
  digitalWrite(sensor_trig, LOW);
  delay(2); // Wait for 2 millisenconds
  digitalWrite(sensor_trig, HIGH);
  delay(10); // Wait for 10 millisenconds
  digitalWrite(sensor_trig, LOW);
  time_ = pulseIn(sensor_echo, HIGH);  // Read the the pulse for a HIGH signal
  return (time_ * 0.034) / 2; // Calculate the distance in cm by the formula s=(t*v)/2
}

// Checks for obstacles
void obstacleDetection()
{
  current_distance = readSensor();  // Read the current distance

  // Check if there was an obstacle in 20cm or less detected and the current 
  // calculated distance is greater than 2cm (based on specs of the
  // HC-SR04 ultrasonic sensor).
  if (current_distance < 20 && current_distance > 2) {
    obstacle_detected = true; // Set the flag to true (i.e. obstacle detected)
  } else {
    obstacle_detected = false; // No obstacle detected
  }
}

// Drive forward
void driveForward()
{
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);

  for (int i=0; i<256; i++) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }
}

// Drive backward
void driveBackward()
{
  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH);
  digitalWrite(input3, LOW);
  digitalWrite(input4, HIGH);

  for (int i=0; i<256; i++) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }

  delay(800);  // Delay to stop the motors

  for (int i=255; i>=0; --i) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }
  // Turn motors off
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
}

// Turn to the left
void turnLeft(int grade)
{
  digitalWrite(input1, HIGH); // Left motor
  digitalWrite(input2, LOW);
  digitalWrite(input3, LOW); // Right motor
  digitalWrite(input4, HIGH);

  for (int i=0; i<256; i++) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }

  delay(grade);  // Delay to stop the motors

  for (int i=255; i>=0; --i) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }
  // Turn motors off
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
}

// Turn to the right
void turnRight(int grade)
{
  digitalWrite(input1, LOW); // Left motor
  digitalWrite(input2, HIGH);
  digitalWrite(input3, HIGH); // Right motor
  digitalWrite(input4, LOW);

  for (int i=0; i<256; i++) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }

  delay(grade);  // Delay to stop the motors

  for (int i=255; i>=0; --i) {
    analogWrite(enable1, i);
    analogWrite(enable2, i);
  }
  // Turn motors off
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
}

// Controls the orientation of the robot
void directionControl()
{
  // Vatriables to store the left and right distances 
  // gathered by the sensor.
  float left_dist = 0.0;
  float right_dist = 0.0;

  if (obstacle_detected) {
    /* If there was an obstacle detected, the following 
    algorithm gets executed step-by-step:
    1. Drive the robot some cm backwards and stop
    2. Turn the robot to the left 45 degrees
    3. Read the distance to the left and save it
    4. Turn the robot to the right 90 degrees
    5. Read the distance to the right and save it
    6. If the current distance (to the right) is greater,
    drive forward to this direction.
    7. Else, turn the robot 90 degrees to the left
    and drive forward in this direction.
    */
    indication();  // Manage the LED's
    driveBackward();
    delay(250);
    turnLeft(DEGREES_45);
    left_dist = readSensor();
    turnRight(2 * DEGREES_45);
    right_dist = readSensor();
    
    // Check to wich direction to turn the robot after calculating
    // the new left and right distances
    if (right_dist > left_dist) {
      driveForward();
    } else {
      turnLeft(2 * DEGREES_45);
      driveForward();
    }
    obstacle_detected = false;  // Reset the detection flag
    indication();  // Manage the LED's
  } else {
      // Drive forward
      digitalWrite(input1, HIGH);
      digitalWrite(input2, LOW);
      digitalWrite(input3, HIGH);
      digitalWrite(input4, LOW);
      indication();  // Manage the LED's
  }
}

// Controls the speed of the wheels
void speedControl()
{
  if (obstacle_detected) {
    // Stop the motors immediately (i.e. set the speed of the  
    // motors to zero to avoid collision)
    for (int i=255; i>=0; --i) {
      analogWrite(enable1, i);
      analogWrite(enable2, i);
    }
  }
  driveForward();
}

void loop() {
  obstacleDetection(); // Check for obstacles
  speedControl();
  directionControl(); // Calculate the new direction
  delay(100);
}
