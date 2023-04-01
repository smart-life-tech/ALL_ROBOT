#include <SoftwareSerial.h>
//#include "Servo.h" //servo library

// include the library code:
#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

#define RIGHT A0         // Right IR sensor connected to analog pin A2 of Arduino Uno:
#define LEFT A1          // Left IR sensor connected to analog pin A3 of Arduino Uno:
#define TRIGGER_PIN A3   // Trigger pin connected to analog pin A1 of Arduino Uno:
#define ECHO_PIN A2      // Echo pin connected to analog pin A0 of Arduino Uno:
#define MAX_DISTANCE 200 // Maximum ping distance:

//unsigned int distance = 0;    // Variable to store ultrasonic sensor distance:
unsigned int Right_Value = 0; // Variable to store Right IR sensor value:
unsigned int Left_Value = 0;  // Variable to store Left IR sensor value:

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance:

// create motor objects
AF_DCMotor Motor1(1, MOTOR12_1KHZ);
AF_DCMotor Motor2(2, MOTOR12_1KHZ);
AF_DCMotor Motor3(3, MOTOR34_1KHZ);
AF_DCMotor Motor4(4, MOTOR34_1KHZ);

// Servo myservo; //create servo object to control the servo:
int pos = 0; // variable to store the servo position:

Servo myservo;                  // create servo object to control servo
SoftwareSerial BT_Serial(2, 3); // RX, TX

#define enA 10 // Enable1 L298 Pin enA
#define in1 4  // Motor1  L298 Pin in1
#define in2 8  // Motor1  L298 Pin in1
#define in3 7  // Motor2  L298 Pin in1
#define in4 6  // Motor2  L298 Pin in1
#define enB 5  // Enable2 L298 Pin enB

#define servo 9

#define R_S A0 // ir sensor Right
#define L_S A1 // ir sensor Left

#define echo A2    // Echo pin
#define trigger A3 // Trigger pin
int rightDistance = 0, leftDistance = 0;
int distance_L, distance_F = 30, distance_R;
long distance;
int set = 20;

int bt_ir_data; // variable to receive data from the serial port and IRremote
int Speed = 130;
int mode = 0;
int IR_data;
// Ultrasonic distance measurement method

int Distance_test()
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigger, LOW);
  float Fdistance = pulseIn(echo, HIGH);
  Fdistance = Fdistance / 58;
  return (int)Fdistance;
}
void setup()
{ // put your setup code here, to run once

  pinMode(R_S, INPUT); // declare if sensor as input
  pinMode(L_S, INPUT); // declare ir sensor as input

  pinMode(echo, INPUT);     // declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output

  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB

  Serial.begin(9600); // start serial communication at 9600bps
  BT_Serial.begin(9600);
  myservo.attach(servo); // attach servo on pin a4 to servo object
                         /*pinMode(servo, OUTPUT);*/

  for (int angle = 70; angle <= 140; angle += 5)
  {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 0; angle -= 5)
  {
    servoPulse(servo, angle);
  }

  for (int angle = 0; angle <= 70; angle += 5)
  {
    servoPulse(servo, angle);
  }
  delay(500);
  mode = 0;
  bt_ir_data == 8;
}

void loop()
{

  if (BT_Serial.available() > 0)
  { // if some date is sent, reads it and saves in state
    String bt_data = BT_Serial.readStringUntil('\n');
    bt_ir_data = bt_data.toInt();
    Serial.println(bt_ir_data);
    if (bt_ir_data > 20 && bt_ir_data < 255)
    {
      Speed = bt_ir_data;
    }
  }

  if (bt_ir_data == 8)
  {
    Serial.println("manual mode 0 selected");
    mode = 0;
    Stop();
    bt_ir_data == 800;
  } // Manual Android Application
  else if (bt_ir_data == 0)
  {
    mode = 1;
    Serial.println("object follower mode 1 selected");
    Speed = 130;
    bt_ir_data == 800;
  } // Auto Line Follower Command
  else if (bt_ir_data == 10)
  {
    mode = 2;
    Serial.println("obstacle avoidance mode 2 selected");
    Speed = 255;
    bt_ir_data == 800;
  } // Auto Obstacle Avoiding Command

  analogWrite(enA, Speed); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed
  analogWrite(enB, Speed); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed

  if (mode == 0)
  {
    //===============================================================================
    //                          Key Control Command
    //===============================================================================
    if (bt_ir_data == 1)
    {
      forword();
    } // if the bt_data is '1' the DC motor will go forward
    else if (bt_ir_data == 2)
    {
      backword();
      Serial.println("backward");
    } // if the bt_data is '2' the motor will Reverse
    else if (bt_ir_data == 3)
    {
      turnLeft();
      Serial.println("turn left");
    } // if the bt_data is '3' the motor will turn left
    else if (bt_ir_data == 4)
    {
      turnRight();
      Serial.println("turn right");
    } // if the bt_data is '4' the motor will turn right
    else if (bt_ir_data == 5)
    {
      Stop();
      //  Serial.println("STOP");
    } // if the bt_data '5' the motor will Stop

    //===============================================================================
    //                          Voice Control Command
    //===============================================================================
    else if (bt_ir_data == 6)
    {
      turnLeft();
      delay(400);
      bt_ir_data = 5;
    }
    else if (bt_ir_data == 7)
    {
      turnRight();
      delay(400);
      bt_ir_data = 5;
    }
  }

  if (mode == 1)
  {
    //===============================================================================
    //                          auto  Follower Control
    //===============================================================================
    delay(50);                  // wait 50ms between pings:
    distance = sonar.ping_cm(); // send ping, get distance in cm and store it in 'distance' variable:
    Serial.print("distance");
    Serial.println(distance); // print the distance in serial monitor:

    Right_Value = digitalRead(RIGHT); // read the value from Right IR sensor:
    Left_Value = digitalRead(LEFT);   // read the value from Left IR sensor:

    Serial.print("RIGHT");
    Serial.println(Right_Value); // print the right IR sensor value in serial monitor:
    Serial.print("LEFT");
    Serial.println(Left_Value); // print the left IR sensor value in serial monitor:

    if ((distance > 1) && (distance < 15))
    { // check wheather the ultrasonic sensor's value stays between 1 to 15.
      // If the condition is 'true' then the statement below will execute:
      // Move Forward:
      Motor1.setSpeed(130); // define motor1 speed:
      Motor1.run(FORWARD);  // rotate motor1 clockwise:
      Motor2.setSpeed(130); // define motor2 speed:
      Motor2.run(FORWARD);  // rotate motor2 clockwise:
      Motor3.setSpeed(130); // define motor3 speed:
      Motor3.run(FORWARD);  // rotate motor3 clockwise:
      Motor4.setSpeed(130); // define motor4 speed:
      Motor4.run(FORWARD);  // rotate motor4 clockwise:
    }
    else if ((Right_Value == 0) && (Left_Value == 1))
    { // If the condition is 'true' then the statement below will execute:

      // Turn Left
      Motor1.setSpeed(150); // define motor1 speed:
      Motor1.run(FORWARD);  // rotate motor1 cloclwise:
      Motor2.setSpeed(150); // define motor2 speed:
      Motor2.run(FORWARD);  // rotate motor2 clockwise:
      Motor3.setSpeed(150); // define motor3 speed:
      Motor3.run(BACKWARD); // rotate motor3 anticlockwise:
      Motor4.setSpeed(150); // define motor4 speed:
      Motor4.run(BACKWARD); // rotate motor4 anticlockwise:
      delay(150);
    }
    else if ((Right_Value == 1) && (Left_Value == 0))
    { // If the condition is 'true' then the statement below will execute:

      // Turn Right
      Motor1.setSpeed(150); // define motor1 speed:
      Motor1.run(BACKWARD); // rotate motor1 anticlockwise:
      Motor2.setSpeed(150); // define motor2 speed:
      Motor2.run(BACKWARD); // rotate motor2 anticlockwise:
      Motor3.setSpeed(150); // define motor3 speed:
      Motor3.run(FORWARD);  // rotate motor3 clockwise:
      Motor4.setSpeed(150); // define motor4 speed:
      Motor4.run(FORWARD);  // rotate motor4 clockwise:
      delay(150);
    }
    else if (distance > 15)
    { // If the condition is 'true' then the statement below will execute:

      // Stop
      Motor1.setSpeed(0);  // define motor1 speed:
      Motor1.run(RELEASE); // stop motor1:
      Motor2.setSpeed(0);  // define motor2 speed:
      Motor2.run(RELEASE); // stop motor2:
      Motor3.setSpeed(0);  // define motor3 speed:
      Motor3.run(RELEASE); // stop motor3:
      Motor4.setSpeed(0);  // define motor4 speed:
      Motor4.run(RELEASE); // stop motor4:
    }
  }

  if (mode == 2)
  {
    //===============================================================================
    //                          Obstacle Avoiding Control
    //===============================================================================
    distance_F = Ultrasonic_read();
    Serial.print("S=");
    Serial.println(distance_F);
    if (distance_F > set)
    {
      forword();
    }
    else
    {
      Check_side();
    }
  }

  delay(10);
}

void servoPulse(int pin, int angle)
{
  // int pwm = (angle * 11) + 500; // Convert angle to microseconds
  // digitalWrite(pin, HIGH);
  // delayMicroseconds(pwm);
  // digitalWrite(pin, LOW);
  // delay(50); // Refresh cycle of servo
  myservo.write(angle);
}

//**********************Ultrasonic_read****************************
long Ultrasonic_read()
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  distance = pulseIn(echo, HIGH);
  return distance / 29 / 2;
}

void compareDistance()
{
  if (distance_L > distance_R)
  {
    turnLeft();
    delay(350);
  }
  else if (distance_R > distance_L)
  {
    turnRight();
    delay(350);
  }
  else
  {
    backword();
    delay(300);
    turnRight();
    delay(600);
  }
}

void Check_side()
{
  Stop();
  delay(100);
  for (int angle = 70; angle <= 140; angle += 5)
  {
    servoPulse(servo, angle);
  }
  delay(300);
  distance_L = Ultrasonic_read();
  delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)
  {
    servoPulse(servo, angle);
  }
  delay(500);
  distance_R = Ultrasonic_read();
  delay(100);
  for (int angle = 0; angle <= 70; angle += 5)
  {
    servoPulse(servo, angle);
  }
  delay(300);
  compareDistance();
}

void forword()
{                          // forword
  digitalWrite(in1, HIGH); // Right Motor forword Pin
  digitalWrite(in2, LOW);  // Right Motor backword Pin
  digitalWrite(in3, LOW);  // Left Motor backword Pin
  digitalWrite(in4, HIGH); // Left Motor forword Pin
}

void backword()
{ // backword

  digitalWrite(in1, LOW);  // Right Motor forword Pin
  digitalWrite(in2, HIGH); // Right Motor backword Pin
  digitalWrite(in3, HIGH); // Left Motor backword Pin
  digitalWrite(in4, LOW);  // Left Motor forword Pin
}

void turnRight()
{                          // turnRight
  digitalWrite(in1, LOW);  // Right Motor forword Pin
  digitalWrite(in2, HIGH); // Right Motor backword Pin
  digitalWrite(in3, LOW);  // Left Motor backword Pin
  digitalWrite(in4, HIGH); // Left Motor forword Pin
}

void turnLeft()
{ // turnLeft

  digitalWrite(in1, HIGH); // Right Motor forword Pin
  digitalWrite(in2, LOW);  // Right Motor backword Pin
  digitalWrite(in3, HIGH); // Left Motor backword Pin
  digitalWrite(in4, LOW);  // Left Motor forword Pin
}

void Stop()
{                         // stop
  digitalWrite(in1, LOW); // Right Motor forword Pin
  digitalWrite(in2, LOW); // Right Motor backword Pin
  digitalWrite(in3, LOW); // Left Motor backword Pin
  digitalWrite(in4, LOW); // Left Motor forword Pin
}