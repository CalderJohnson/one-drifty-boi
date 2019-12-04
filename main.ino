/*
  Author: Calder Johnson
  This is the code that powers 'One Drifty Boi', our robot
  All code is original
*/

#include <Servo.h>
#include <IRremote.h>

const int IRpin = 2; //remote pin
const int servoPin = 3;
const int enableLeft = 5; //motor pins
const int in1 = 6;
const int in2 = 7;
const int in3 = 8;
const int in4 = 9;
const int enableRight = 10;
const int trigPin = 11; //ultrasonic senosor pins
const int echoPin = 12;
Servo servo;
IRrecv irrecv(IRpin);
decode_results results;

void driveForward(float feet) //drives forward, given a number of feet
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(feet * 845.0f);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void driveBack(float feet) //drives back, given a number of feet
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(feet * 845.0f);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnRight(int Degrees) //turns right, given a number of degrees
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(Degrees * 7.2);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnLeft(int Degrees) //turns left, given a number of degrees
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(Degrees * 7.2);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void checkDistanceForward(int& cm) //sets cm equal to the distance to the object in front of the robot
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  cm = pulseIn(echoPin, HIGH) / 58.0;
  cm = (int(cm * 100.0)) / 100.0;
}

void checkDistanceRight(int& cm) //turns sensor right, than checks distance
{
  servo.write(180);
  delay(500);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  cm = pulseIn(echoPin, HIGH) / 58.0;
  cm = (int(cm * 100.0)) / 100.0;
}

void checkDistanceLeft(int& cm) //turns sensor left, than checks distance
{
  servo.write(0);
  delay(500);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  cm = pulseIn(echoPin, HIGH) / 58.0;
  cm = (int(cm * 100.0)) / 100.0;
}

void setup() //sets up all pins, seeds random, initializes motors and ir reciever
{
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);
  servo.write(90); //points servo forward
  analogWrite(enableLeft, 1018); //enables motors, adjusting speeds to minimize drift
  digitalWrite(enableRight, HIGH);
  irrecv.enableIRIn();
  randomSeed(analogRead(0));
}

void loop()
{
  //static, so that they are not re-initialized with each iteration of the loop
  static bool automatic = false; //bool for automatic mode
  static int cmForward = 0; //distances left, right, and forward
  static int cmLeft = 0;
  static int cmRight = 0;
  static bool leftOrRight; //for random choices
  static bool forward = false, backward = false, left = false, right = false; //bools for movement/turning

  if (irrecv.decode(&results)) //get user input
  {
    switch (results.value)
    {
      case 0xFFA857: //gear button toggles automatic mode
        if (automatic)
          automatic = false;
        else
          automatic = true;
        forward = false; //so that he dosnt start doing stuff when coming out of automatic mode
        backward = false;
        left = false;
        right = false;
        break;
      case 0xFF02FD: //if forward arrow is pressed, drive until pressed again
        if (forward)
          forward = false;
        else
          forward = true;
        backward = false; //disables other bools so that he is not doing multiple things at once
        left = false;
        right = false;
        break;
      case 0xFFE01F: //if left arrow is pressed, turn left until pressed again
        if (left)
          left = false;
        else
          left = true;
        backward = false;
        forward = false;
        right = false;
        break;
      case 0xFF906F: //if right arrow is pressed, turn right until pressed again
        if (right)
          right = false;
        else
          right = true;
        backward = false;
        left = false;
        forward = false;
        break;
      case 0xFF9867: //if back arrow is pressed, drive back until pressed again
        if (backward)
          backward = false;
        else
          backward = true;
        forward = false;
        left = false;
        right = false;
        break;
      default: //if dosnt recognise command, shakes head
        servo.write(100);
        delay(200);
        servo.write(80);
        delay(200);
        servo.write(90);
        delay(200);
        break;
    }
    irrecv.resume();
  }
  if (automatic) //avoids obstacles
  {
    driveForward(0.2f);
    checkDistanceForward(cmForward);
    if (cmForward < 20 && cmForward > 0) //if theres an object, look left and right, than turn in the direction thats more open
    {
      checkDistanceLeft(cmLeft);
      checkDistanceRight(cmRight);
      servo.write(90);
      delay(500);
      if (cmLeft < 0 && cmRight < 0) //because negative indicates a distance further than the sensor can detect
      {
        leftOrRight = random(0, 2); //if both are very far, randomly pick a direction
        if (leftOrRight)
          turnRight(90);
        else
          turnLeft(90);
      }
      else if (cmRight < 0) //turns right or left, whichever is further
        turnRight(90);
      else if (cmLeft < 0)
        turnLeft(90);
      else if (cmRight > cmLeft)
        turnRight(90);
      else
        turnLeft(90);
    }
  }
  else //if not in automatic mode, act according to movement/turning bools
  {
    if (forward)
      driveForward(0.1);
    else if (backward)
      driveBack(0.1);
    else if (left)
      turnLeft(5);
    else if (right)
      turnRight(5);
  }
}
