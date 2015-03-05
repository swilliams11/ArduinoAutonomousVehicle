/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
 //Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontLeft = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rearLeft = AFMS.getMotor(2);
Adafruit_DCMotor *frontRight = AFMS.getMotor(3);
Adafruit_DCMotor *rearRight = AFMS.getMotor(4);
Servo servo1;
const int pingPin = 9;
const int xPin = 2;     // X output of the accelerometer
const int yPin = 1;     // Y output of the accelerometer
int pulseX, pulseY;

// variables to contain the resulting accelerations
int accelerationX, accelerationY;
//right turn training
int turnRightTime = 30;
double gammaRightTurn = .1;
long goalInchesFromWall = 1; //no more than 1 inch from the wall;

//left turn training
int turnLeftTime = 30;
double gammaLeftTurn = .1;
long goalInchesFromWallLeftTurn = 1; //no more than 1 inch from the wall;

long inches, cm;
const int STOPPED = 0;
const int FORWARDM = 1;

int state = STOPPED;

int motorSpeed = 70;
int motorSpeedRightTurn = 75;
int motorSpeedLeftTurn = 75;

int distanceToStop = 30; //inches - start stopping at this point
int goalStopDistanceToWall = 2; //inches should stop within one inch of object

double gammaStop = .1; //learning rate for stop training
boolean stopTrainingComplete = false;
boolean rightTurnTrainingComplete = false;
boolean leftTurnTrainingComplete = false;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.println("create motor sheild!");  
  AFMS.begin();  // OR with a different frequency, say 1KHz
  Serial.println("frequency set!");
  servo1.attach(10);
  servoFront();
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  Serial.println("setup complete");
  uint8_t i;  
  /*
  Serial.println("Training car to stop.");

  i = 0;
  while(!stopTrainingComplete){
    i++;
  //if(!stopTrainingComplete){
    trainForwardAndStop();
    if(i == 8){
      break;
    }
  }*/
  
  delay(10000);
  
  //right turn training
/*  i = 0;
  while(!rightTurnTrainingComplete){
    i++;
  //if(!rightTurnTrainingComplete && stopTrainingComplete){
    trainRightTurn();
    if(i == 8){
      break;
    }
  }*/
    
  delay(10000);
    
  //left turn training
  i = 0;
  while(!leftTurnTrainingComplete){
    i++;
  //if(!rightTurnTrainingComplete && stopTrainingComplete){
    trainLeftTurn();
    if(i == 8){
      break;
    }
  }
}

void loop() {
  uint8_t i;
  //stop training
  
  

  /*if(ping() >= distanceToStop && state == STOPPED){
  Serial.println("Forward!");
    servoFront();
    state = FORWARDM;
    forward();
  } else if (ping() < distanceToStop && state == FORWARDM) {
      Serial.println("Stop!");
    state = STOPPED;
    stop();  
    updateQStop();
  }*/
  
  
    
  //servoRight();
  //delay(1000);
  //servoLeft();
  //delay(1000);
  //servoFront();
  //delay(1000);
   /*if(calcRotation() == -24){
      turnRight();
   }*/
//  Serial.print("tick");
  //forward();
  //backward();
  //for(i = 0; i < 255; i++){
  //turnRight();
  //delay(5000);
 // }
  
  //Serial.print("tock");
  //Serial.print("tech");
  /*frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE);*/
  delay(250);
}

/*****************************************
  Right turn training functions.
*/
void trainRightTurn(){
  Serial.println("Right Turn Training");
  //Serial.println("Move forward");
  //moveCarForward();
  servoFront();
  delay(1000);
  Serial.println("Ping1!");
  long startDistance = ping();
  delay(1000);
  Serial.print("Start distance: ");
  Serial.print(startDistance);
  Serial.println();
  //Serial.println("Servo Right");
  //servoRight();
  
  //Serial.println("Ping Right distance!");
  //long dist = ping();
  //Serial.println("check the distance of Right Ping");
  //if(dist >= 12) {
    servoFront();
    Serial.println("turning right");
    turnRight();
    Serial.println("waiting");
    delay(1500);
    Serial.println("moving car forward");
    moveCarForward();
    delay(1000);
    Serial.println("Stopping car");
    stopCar();
    Serial.println("checking distance");
    servoLeft();
    delay(1000);
    long endDistance = ping();
    delay(1000);
    Serial.print("End distance: ");
    Serial.print( endDistance);
    Serial.println();
    
    long distApart = endDistance - startDistance;
    Serial.print("Distance apart: ");
    Serial.print(distApart);
    Serial.println();    
    updateQRight(distApart);
    setStopRightTurnTrainingComplete(distApart);
  //}
  delay(5000);
}

/*
Stops the right turn training session if the distApart if withing an acceptable range (<= 0)
Set the stopRightTurnTrainingComplete Flag
*/
void setStopRightTurnTrainingComplete(long distApart){
  long d = distanceToRightTurnGoal(distApart);
  if(d <= 0){
    rightTurnTrainingComplete = true;
  }
}

int updateQRight(long distance){
  int r = RRight(distance);
  long d = distanceToRightTurnGoal(distance);
  turnRightTime = turnRightTime + (r * gammaRightTurn * d);
  Serial.print("Turn right updated to ");
  Serial.print(turnRightTime);
  Serial.println();
}

int RRight(long distance){
  long d = distanceToRightTurnGoal(distance);
  if(d > goalInchesFromWall){
     return -1;
  } else {
    return 0;
  }
}

long distanceToRightTurnGoal(long distance){
  Serial.print("Distance To Goal: ");
  Serial.print(distance - goalInchesFromWall);
  Serial.println();
  return  distance - goalInchesFromWall;
}

/****************************************************
  Right turn functions
*/
void turnRight(){
  uint16_t i;
  frontRight->run(BACKWARD);
  rearRight->run(BACKWARD);  
  frontLeft->run(FORWARD);
  rearLeft->run(FORWARD);
  
  //while(calcRotation() < 24){
  for (i=0; i< turnRightTime; i++) {
    frontLeft->setSpeed(motorSpeedRightTurn);  
    rearLeft->setSpeed(motorSpeedRightTurn);  
    frontRight->setSpeed(motorSpeedRightTurn);  
    rearRight->setSpeed(motorSpeedRightTurn);  
    delay(30);
  }
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
}


/*****************************************
  Left turn training functions.
*/
void trainLeftTurn(){
  Serial.println("Left Turn Training");
  //Serial.println("Move forward");
  //moveCarForward();
  servoFront();
  delay(1000);
  Serial.println("Ping1!");
  long startDistance = ping();
  delay(1000);
  Serial.print("Start distance: ");
  Serial.print(startDistance);
  Serial.println();
  //Serial.println("Servo Right");
  //servoRight();
  
  //Serial.println("Ping Right distance!");
  //long dist = ping();
  //Serial.println("check the distance of Right Ping");
  //if(dist >= 12) {
    servoFront();
    Serial.println("turning left");
    turnLeft();
    Serial.println("waiting");
    delay(1500);
    Serial.println("moving car forward");
    moveCarForward();
    delay(1000);
    Serial.println("Stopping car");
    stopCar();
    Serial.println("checking distance");
    servoRight();
    delay(1000);
    long endDistance = ping();
    delay(1000);
    Serial.print("End distance: ");
    Serial.print( endDistance);
    Serial.println();
    
    long distApart = endDistance - startDistance;
    Serial.print("Distance apart: ");
    Serial.print(distApart);
    Serial.println();    
    updateQLeft(distApart);
    setStopLeftTurnTrainingComplete(distApart);
  //}
  delay(5000);
}

/*
Stops the left turn training session if the distApart if withing an acceptable range (<= 0)
Set the stopRightTurnTrainingComplete Flag
*/
void setStopLeftTurnTrainingComplete(long distApart){
  long d = distanceToLeftTurnGoal(distApart);
  if(d <= 0){
    leftTurnTrainingComplete = true;
  }
}

int updateQLeft(long distance){
  int r = RLeft(distance);
  long d = distanceToLeftTurnGoal(distance);
  turnLeftTime = turnLeftTime + (r * gammaLeftTurn * d);
  Serial.print("Turn left updated to ");
  Serial.print(turnLeftTime);
  Serial.println();
}

int RLeft(long distance){
  long d = distanceToRightTurnGoal(distance);
  if(d > goalInchesFromWall){
     return -1;
  } else {
    return 0;
  }
}

long distanceToLeftTurnGoal(long distance){
  Serial.print("Distance To Goal: ");
  Serial.print(distance - goalInchesFromWallLeftTurn);
  Serial.println();
  return  distance - goalInchesFromWallLeftTurn;
}

/****************************************************
  Left turn functions
*/
void turnLeft(){
  uint16_t i;
  frontRight->run(FORWARD);
  rearRight->run(FORWARD);  
  frontLeft->run(BACKWARD);
  rearLeft->run(BACKWARD);
  
  //while(calcRotation() < 24){
  for (i=0; i< turnLeftTime; i++) {
    frontLeft->setSpeed(motorSpeedLeftTurn);  
    rearLeft->setSpeed(motorSpeedLeftTurn);  
    frontRight->setSpeed(motorSpeedLeftTurn);  
    rearRight->setSpeed(motorSpeedLeftTurn);  
    delay(30);
  }
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
}

//Training functions for stop
void trainForwardAndStop(){
  if(canGoForward()){
      Serial.println("Forward!");
      goForward();
  } else if (mustStop()) {
      Serial.println("Stop!");
      stopCar();
      updateQStop();
  }
  setStopTrainingComplete();
}

void setStopTrainingComplete(){
  if(distanceToStop <= goalStopDistanceToWall){
    stopTrainingComplete = true;
  }
}

//forward
void moveCarForward(){
  if(canGoForward()){
      Serial.println("Forward!");
      goForward();
  } else if (mustStop()) {
      Serial.println("Stop!");
      stopCar();
  }
}

boolean canGoForward(){
  servoFront();
  if(ping() >= distanceToStop && state == STOPPED){
    return true;
  } else {
    return false;
  }  
}

boolean mustStop(){
  if (ping() < distanceToStop && state == FORWARDM){
    return true;
  } else {
     return false;
   }
}

void goForward(){
    servoFront();
    state = FORWARDM;
    forward();
}

void forward(){
  uint8_t i;
  frontLeft->run(FORWARD);
  rearLeft->run(FORWARD);
  frontRight->run(FORWARD);
  rearRight->run(FORWARD);
  for (i=0; i<motorSpeed; i++) {
    /*if(!canGoForward()){
      stop();
      break;
    }*/
    frontLeft->setSpeed(i);  
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }  
}

void stopCar(){
    state = STOPPED;
    stop();  
}

void stop(){
   uint8_t i;
  for (i=motorSpeed; i!=0; i--) {
   frontLeft->setSpeed(i);  
   rearLeft->setSpeed(i);  
   frontRight->setSpeed(i);  
   rearRight->setSpeed(i);  
   delay(2);
  }
}

int QStop(){
   return distanceToStop;
}

int distanceToStopGoal(){
  return distanceToStop - goalStopDistanceToWall;
}

int RStop(){
  int d = distanceToStopGoal();
  if(d > goalStopDistanceToWall){
     return -1;
  } else {
    1;
  }
}

int updateQStop(){
  int r = RStop();
  int d = distanceToStopGoal();
  distanceToStop = distanceToStop + (r * gammaStop * d);
}

void backward(){
    uint8_t i;
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  rearLeft->run(BACKWARD);
  rearRight->run(BACKWARD);
  for (i=0; i<255; i++) {
    frontLeft->setSpeed(i);  
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    frontLeft->setSpeed(i); 
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }
}

void checkDistance(){
      uint8_t i;
    
}

void servoRight(){
  servo1.write(0);
}
//SERVO
/*void servoRight(){
      uint8_t i;    
  for (i=255; i!=0; i--) {
    servo1.write(map(i, 0, 255, 0, 180)); 
    delay(3);
  }
}*/

void servoLeft(){
      uint8_t i;
      servo1.write(180);
  /*
  for (i=0; i<255; i++) {
     servo1.write(map(i, 0, 255, 0, 180)); 
     delay(3);
  }*/
}

void servoFront(){
  servo1.write(90);
}

//PING - DISTANCE SENSOR
long ping(){
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration; //, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
 
  Serial.print(inches);
  Serial.print("in, ");
  //Serial.print(cm);
  //Serial.print("cm");
  Serial.println();
 
//  delay(10);
  return inches;
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long calcRotation(){

 
  // read pulse from x- and y-axes:
  pulseX = pulseIn(xPin,HIGH);  
  pulseY = pulseIn(yPin,HIGH);
//  Serial.print(pulseX);
//    Serial.print("\t");
//   Serial.print(pulseY);
//     Serial.println();
  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.
  accelerationX = ((pulseX / 10) - 500) * 8;
  accelerationY = ((pulseY / 10) - 500) * 8;
  Serial.print("X acceleration: " );
    Serial.print(accelerationX );
  Serial.println();
  return accelerationX; 
}
