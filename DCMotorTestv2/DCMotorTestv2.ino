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
int turnLeftTime = 32; //was 30
double gammaLeftTurn = .1;
long goalInchesFromWallLeftTurn = 1; //no more than 1 inch from the wall;
double leftTurnDistApartLow = 2.0; //1.4
double leftTurnDistApartHigh = 2.5; //1.7
//long inches, cm;
const int NORTH = 0;
const int SOUTH = 1;
const int EAST = 2;
const int WEST = 3;
const int STOPPED = 4; //prior version 0
const int FORWARDM = 5;//prior version 1
const int LEFT = 6;
const int RIGHT = 7;

int state = STOPPED;

int motorSpeed = 70;
int motorSpeedRightTurn = 50; //was 75
int motorSpeedLeftTurn = 50;

int distanceToStop = 2; //30 inches - start stopping at this point
int goalStopDistanceToWall = 2; //inches should stop within one inch of object

double gammaStop = .1; //learning rate for stop training
boolean stopTrainingComplete = false;
boolean rightTurnTrainingComplete = false;
boolean leftTurnTrainingComplete = false;

int const r = 4, c = 4;
int Q[r][c];
int R[r][c];
int frequencyTable[r][c];
double gammaQLearning = .9;
int currentDirection = NORTH;

//holds the distances of left (0), forward(1), right(2)
double distArray [4];
long randomNumber;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.println("create motor sheild!");  
  randomSeed(analogRead(0)); //seed the random number generator
  AFMS.begin();  // OR with a different frequency, say 1KHz
  Serial.println("frequency set!");
  servo1.attach(10);
  servoFront();
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  Serial.println("setup complete");
  uint8_t i;  
  
  Serial.println("Training car to stop.");
/*
  i = 0;
  while(!stopTrainingComplete){
    i++;
    Serial.print("Stop training Iteration ");
    Serial.print(i);
    Serial.println();
  //if(!stopTrainingComplete){
    trainForwardAndStop();
    //delay(5000);
    if(i == 8){
      break;
    }
  }
  stopCar();  
  delay(5000);
  servoRight();
  delay(1000);
  servoFront();
  delay(1000);
  servoRight();
  delay(1000);
  servoFront();
  delay(5000);
  
  //right turn training
  i = 0;
  while(!rightTurnTrainingComplete){
    i++;
    Serial.print("***************Iteration ");
    Serial.print(i);
    Serial.print(" ***********************");
    Serial.println();
  //if(!rightTurnTrainingComplete && stopTrainingComplete){
    trainRightTurn();
    if(i == 8){
      break;
    }
  }
  
  servoFront();
  delay(1000);
  servoLeft();
  delay(1000);
  servoFront();
  delay(1000);
  servoLeft();
  delay(1000);
  servoFront();  
  delay(5000);
    
  //left turn training
  i = 0;
  while(!leftTurnTrainingComplete){
    i++;
    Serial.print("***************Iteration ");
    Serial.print(i);
    Serial.print(" ***********************");
    Serial.println();
  //if(!rightTurnTrainingComplete && stopTrainingComplete){
    trainLeftTurn();
    if(i == 8){
      break;
    }
  }
  */
  servoFront();
  initR();
  initQ(); 
  
  servoRight();
  delay(1000);
  servoLeft();
  delay(1000);
  servoRight();
  delay(1000);
  servoLeft();
  delay(5000);
  servoFront();
}

void loop() {
  uint8_t i;
  QLearningAgent(currentDirection);  
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
  double startDistance = ping();
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
    double endDistance = ping();
    delay(1000);
    Serial.print("End distance: ");
    Serial.print( endDistance);
    Serial.println();
    
    double distApart = endDistance - startDistance;
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
void setStopRightTurnTrainingComplete(double distApart){
  double d = distanceToRightTurnGoal(distApart);
  if(d <= 0){
    rightTurnTrainingComplete = true;
  }
}

int updateQRight(double distance){
  double r = RRight(distance);
  long d = distanceToRightTurnGoal(distance);
  turnRightTime = turnRightTime + (r * gammaRightTurn * d);
  Serial.print("Turn right updated to ");
  Serial.print(turnRightTime);
  Serial.println();
}

int RRight(double distance){
  double d = distanceToRightTurnGoal(distance);
  if(d > goalInchesFromWall){
     return -1;
  } else {
    return 0;
  }
}

double distanceToRightTurnGoal(double distance){
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
  double startDistance = ping();
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
    double endDistance = ping();
    delay(1000);
    Serial.print("End distance: ");
    Serial.print( endDistance);
    Serial.println();
    
    double distApart = endDistance - startDistance;
    Serial.print("Distance apart: ");
    Serial.print(distApart);
    Serial.println();    
    updateQLeft(startDistance, distApart);
    setStopLeftTurnTrainingComplete(startDistance, distApart);
  //}
  delay(5000);
}

/*
Stops the left turn training session if the distApart if withing an acceptable range (<= 0)
Set the stopRightTurnTrainingComplete Flag
*/
void setStopLeftTurnTrainingComplete(double startDistance, double endMinusStartDistance){
  //double d = distanceToLeftTurnGoal(distApart);
  Serial.print("Stop training when reward is 0:");
 // Serial.print("end distance / start distance <= .1");
 // Serial.print( endMinusStartDistance / startDistance);
   Serial.print("reward is :");
   
  int r = RLeft(startDistance, endMinusStartDistance);
  Serial.print(r);
  Serial.println();
  //if(endMinusStartDistance >= leftTurnDistApartLow && endMinusStartDistance <= leftTurnDistApartHigh){
  if(r == 0){
    leftTurnTrainingComplete = true;
  } else {
    leftTurnTrainingComplete = false;
  }
  /*if(abs(endMinusStartDistance/startDistance) <= .1){
    leftTurnTrainingComplete = true;
  } else {
    leftTurnTrainingComplete = false;
  }*/
}

/*
Utility function that updates the turnLeftTime variable.

*/
int updateQLeft(double startDistance, double endMinusStartDistance){
  int r = RLeft(startDistance, endMinusStartDistance);
  double currentTurnLeftTime = 0.0;
  Serial.print("Reward is ");
  Serial.print(r);
  Serial.println();
  //int multiplicationFactor = abs(endMinusStartDistance / startDistance);
  //long d = distanceToLeftTurnGoal(endMinusStartDistance);
  
  //endMinusStartDistance could be negative so we take the absolute value of it
  turnLeftTime = turnLeftTime + (r * gammaLeftTurn * abs(endMinusStartDistance));
  //Serial.print("turnLeftTime: ");
  //Serial.print(turnLeftTime);
  //Serial.print("+ (r * gammaLeftTurn * abs(endMinusStartDistance)): ");
  //Serial.print(r * gammaLeftTurn * abs(endMinusStartDistance));
  Serial.println();
  Serial.print("Turn left updated to ");
  Serial.print(turnLeftTime);
  Serial.println();
}

/*
Reward function that returns either -1 (reduce), 0 (do not change), or 1 (increase)
*/
int RLeft(double startDistance, double endMinusStartDistance){
  //double d = distanceToLeftTurnGoal(distance);
  double percent = endMinusStartDistance / startDistance;
  //if the goal is greater than goalInchesFromWall +- 10%
  //double checkD = goalInchesFromWallLeftTurn - (goalInchesFromWallLeftTurn * .1);
  /*if(percent > .2 ){ 
     return -1;     
  } else if (percent < -.1){
     return 20; 
  } else {
    return 0;
  }*/
  /*if(endMinusStartDistance > (startDistance + leftTurnDistApartHigh)){
    return -1;
  } else if (endMinusStartDistance < (startDistance + leftTurnDistApartLow)) {
    return 20;
  } else {
    return 0;
  }*/
  
  /*
  Serial.print("RLeft()-> ");
  Serial.print("endMinusStart + startDistance: ");
  Serial.print(endMinusStartDistance + startDistance);
  Serial.print(" startDistance + leftTurnDistApartHight:");
  Serial.print(startDistance + leftTurnDistApartHigh);
  Serial.println();
  
  Serial.print("RLeft()-> ");
  Serial.print(" startDistance + leftTurnDistApartLow:");
  Serial.print(startDistance + leftTurnDistApartLow);
  Serial.println();
  if((endMinusStartDistance + startDistance) > (startDistance + leftTurnDistApartHigh)){
    return -1;
  } else if ((endMinusStartDistance + startDistance) < (startDistance + leftTurnDistApartLow)) {
    return 20;
  } else {
    return 0;
  }*/
  

  double enddist = endMinusStartDistance + startDistance;
  double trueDist = enddist - leftTurnDistApartLow;
  double value = trueDist - startDistance;
  Serial.print("RLeft()-> ");
  Serial.print(" enddist: ");
  Serial.print(enddist);
  Serial.print(" truedist: ");
  Serial.print(trueDist);
  Serial.print(" value: ");
  Serial.print(value);
  Serial.println();
  if(value > 1){
    return -1;
  } else if (value < -1) {
    return 20;
  } else {
    return 0;
  }
    
}

double distanceToLeftTurnGoal(double distance){
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
  delay(1000);
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
double ping(){
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  double duration, inches; //, cm;

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
  //cm = microsecondsToCentimeters(duration);
 
  Serial.print(inches);
  Serial.print("in, ");
  //Serial.print(cm);
  //Serial.print("cm");
  Serial.println();
 
//  delay(10);
  return inches;
}

double microsecondsToInches(double microseconds)
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

/*
Initialize the Q model with 0s;
*/
void initQ(){
  uint8_t i, j;
  
  for(i=0; i < r; i++){
    for(j=0; j < c; j++){
      Q[i][j] = 0;
    }
  }
  for(i=0; i < r; i++){
    for(j=0; j < c; j++){
      frequencyTable[i][j] = 0;
    }
  }
}


/*
Initialize the Reward/transition matrix.
const int NORTH = 0;
const int SOUTH = 1;
const int EAST = 2;
const int WEST = 3;

-1 means the transition is not allowed.
e.g. North to South is not allowed; North to North is not allowed;
*/
void initR(){
    R[0][0] = -1; //N > N
    R[0][1] = -1; //N > S
    R[0][2] = 0;  //N > E
    R[0][3] = 0;  //N > W
    R[1][0] = -1; //S > N
    R[1][1] = -1; //S > S
    R[1][2] = 0;  //S > E
    R[1][3] = 0;  //S > W
    R[2][0] = 10; //E > N
    R[2][1] = -10;//E > S
    R[2][2] = -1; //E > E
    R[2][3] = -1; //E > W
    R[3][0] = 10; //W > N
    R[3][1] = -10;//W > S
    R[3][2] = -1; //W > E
    R[3][3] = -1; //W > W  
}


/*
returns an action given the current state
*/
int QLearningAgent(int currentState){
  selectAction(currentState);
  while(!goalState()){
    Serial.print("in loop");
    Serial.println();
    int action = selectAction(currentState);
    Serial.print("action selected: ");
    Serial.print(action);
    Serial.println();
    int newState = takeAction(currentState, action);
    int reward = R[currentState][action];
    int maxQ = getMaxQ(newState);
    Q[currentState][action] = Q[currentState][action] + (r + (gammaQLearning * maxQ));
    currentState = newState;
    delay(1000);
  }
  delay(10000);   
}

/*
Exploration function
*/
double f(){
}

void initDistanceArray(){
  
}

/*
Goal state is when the car cannot turn left, right or move forward.
*/
boolean goalState(){
  uint8_t i;
  for(i = 0; i < 4; i++){
    if(distArray[i] > 7){
      return false;
    }
  } 
  return true; 
}

/*
randomly chooses the next action
*/
/*int selectAction( int currentState){
  nextAction = random(0,4); //generate random number between 0 & 3
  //generate random number as long as R is -1 (not a valid transition)
  while((R[currentState][nextAction] == -1){    
    nextAction = random(0,4);
  }      
  return nextAction;
}*/

/*
selects an action based on the precept.
*/
int selectAction(int currentState){
  double temp;
  int i;
  for(i = 0; i < 4; i++){
    distArray[i] = 0;
  }
  servoLeft();
  delay(1000);
  double distance = ping();
  delay(1000);
  distArray[direction(currentState,LEFT)] = distance;
  
  servoFront();
  delay(1000);
  distance = ping();
  delay(1000);
  distArray[currentState] = distance;
    
  servoRight();
  delay(1000);
  distance = ping();
  delay(1000);
  distArray[direction(currentState,RIGHT)] = distance;

  temp = max(distArray[0], distArray[1]);    
  for (i = 2; i< 4; i++){
    temp = max(temp, distArray[i]);    
  }
  for(i = 0; i < 4; i++){
    if(distArray[i] == temp){
      return i;
    }
  }
}

int takeAction(int currentState, int action){
  if(currentState == NORTH && action == EAST){
    turnRight();
//    currentState = EAST;   
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }
    delay(2000);
    return EAST;
  } else if(currentState == NORTH && action == WEST){
    turnLeft();
    delay(2000);
//    currentState = WEST;
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return WEST;
  } else if(currentState == SOUTH && action == EAST){
    turnLeft();
//    currentState = EAST;
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return EAST;
  } else if(currentState == SOUTH && action == WEST){
    //currentState = WEST;
    turnRight();
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return WEST;
  } else if(currentState == EAST && action == NORTH){
//    currentState = NORTH;
    turnLeft();
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return NORTH;
  } else if(currentState == EAST && action == SOUTH){
//    currentState = SOUTH;
    turnRight();
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return SOUTH;
  } else if(currentState == WEST && action == NORTH){
//    currentState = NORTH;
    turnRight();
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return NORTH;
  } else if(currentState == WEST && action == SOUTH){
//    currentState = SOUTH;
    turnLeft();
    delay(2000);
    while(true){
      moveCarForward();
      if(state == STOPPED){
        break;
      }
      delay(50);
    }   
    delay(2000);
    return SOUTH;
  }  
}

int getMaxQ(int newState){  
  if(newState == NORTH){
    return max(Q[newState][EAST], Q[newState][WEST]);
  } else if(newState == SOUTH){
    return max(Q[newState][EAST], Q[newState][WEST]);
  } else if(newState == EAST){
    return max(Q[newState][NORTH], Q[newState][SOUTH]);
  } else if(newState == WEST){
    return max(Q[newState][NORTH], Q[newState][SOUTH]);
  }
}

int direction(int currentState, int servoDirection){
  if(currentState == NORTH && servoDirection == LEFT){
    return WEST;
  }else if (currentState == NORTH && servoDirection == RIGHT){
    return EAST;
  }else if (currentState == SOUTH && servoDirection == LEFT){
    return EAST;
  }else if (currentState == SOUTH && servoDirection == RIGHT){
    return WEST;
  }else if (currentState == EAST && servoDirection == LEFT){
    return NORTH;
  }else if (currentState == EAST && servoDirection == RIGHT) {
    return SOUTH;
  }else if (currentState == WEST && servoDirection == LEFT){
    return SOUTH;
  }else if (currentState == WEST && servoDirection == RIGHT) {
    return NORTH;
  }
}
int chooseDirection(){
  
}
