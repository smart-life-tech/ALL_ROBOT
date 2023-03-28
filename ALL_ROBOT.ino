#include <SoftwareSerial.h>
#include <Servo.h> //servo library
Servo myservo;     // create servo object to control servo

SoftwareSerial BT_Serial(2, 3); // RX, TX

#define enA 10 // Enable1 L298 Pin enA
#define in1 9  // Motor1  L298 Pin in1
#define in2 8  // Motor1  L298 Pin in1
#define in3 7  // Motor2  L298 Pin in1
#define in4 6  // Motor2  L298 Pin in1
#define enB 5  // Enable2 L298 Pin enB

#define servo A4

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
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(Trig, LOW);
    float Fdistance = pulseIn(Echo, HIGH);
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
                           /*pinMode(servo, OUTPUT);
                       
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
                            }*/
    delay(500);
}

void loop()
{

    if (BT_Serial.available() > 0)
    { // if some date is sent, reads it and saves in state
        bt_ir_data = BT_Serial.read();
        Serial.println(bt_ir_data);
        if (bt_ir_data > 20)
        {
            Speed = bt_ir_data;
        }
    }

    if (bt_ir_data == 8)
    {
        mode = 0;
        Stop();
    } // Manual Android Application and IR Remote Control Command
    else if (bt_ir_data == 9)
    {
        mode = 1;
        Speed = 130;
    } // Auto Line Follower Command
    else if (bt_ir_data == 10)
    {
        mode = 2;
        Speed = 255;
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
        } // if the bt_data is '2' the motor will Reverse
        else if (bt_ir_data == 3)
        {
            turnLeft();
        } // if the bt_data is '3' the motor will turn left
        else if (bt_ir_data == 4)
        {
            turnRight();
        } // if the bt_data is '4' the motor will turn right
        else if (bt_ir_data == 5)
        {
            Stop();
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
        myservo.write(60); // setservo position to right side
        delay(200);
        rightDistance = Distance_test();

        myservo.write(120); // setservo position to left side
        delay(200);
        leftDistance = Distance_test();

        if ((rightDistance > 70) && (leftDistance > 70))
        {
            stop();
        }
        else if ((rightDistance >= 20) && (leftDistance >= 20))
        {
            forward();
        }
        else if ((rightDistance <= 10) && (leftDistance <= 10))
        {
            backword();
            delay(100);
        }
        else if (rightDistance - 3 > leftDistance)
        {
            left();
            delay(100);
        }
        else if (rightDistance + 3 < leftDistance)
        {
            right();
            delay(100);
        }
        else
        {
            stop();
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
    int pwm = (angle * 11) + 500; // Convert angle to microseconds
    digitalWrite(pin, HIGH);
    delayMicroseconds(pwm);
    digitalWrite(pin, LOW);
    delay(50); // Refresh cycle of servo
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
{                            // forword
    digitalWrite(in1, HIGH); // Right Motor forword Pin
    digitalWrite(in2, LOW);  // Right Motor backword Pin
    digitalWrite(in3, LOW);  // Left Motor backword Pin
    digitalWrite(in4, HIGH); // Left Motor forword Pin
}

void backword()
{                            // backword
    digitalWrite(in1, LOW);  // Right Motor forword Pin
    digitalWrite(in2, HIGH); // Right Motor backword Pin
    digitalWrite(in3, HIGH); // Left Motor backword Pin
    digitalWrite(in4, LOW);  // Left Motor forword Pin
}

void turnRight()
{                            // turnRight
    digitalWrite(in1, LOW);  // Right Motor forword Pin
    digitalWrite(in2, HIGH); // Right Motor backword Pin
    digitalWrite(in3, LOW);  // Left Motor backword Pin
    digitalWrite(in4, HIGH); // Left Motor forword Pin
}

void turnLeft()
{                            // turnLeft
    digitalWrite(in1, HIGH); // Right Motor forword Pin
    digitalWrite(in2, LOW);  // Right Motor backword Pin
    digitalWrite(in3, HIGH); // Left Motor backword Pin
    digitalWrite(in4, LOW);  // Left Motor forword Pin
}

void Stop()
{                           // stop
    digitalWrite(in1, LOW); // Right Motor forword Pin
    digitalWrite(in2, LOW); // Right Motor backword Pin
    digitalWrite(in3, LOW); // Left Motor backword Pin
    digitalWrite(in4, LOW); // Left Motor forword Pin
}