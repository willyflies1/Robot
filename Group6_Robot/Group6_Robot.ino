/*** INCLUDES ***/
#include <IRremote.h>
#include <Servo.h>

/*** SET PINS ***/

int RECV_PIN = 11;
char incomingByte = 0;

/*** ATTACH PINS ***/

IRrecv irrecv(RECV_PIN);

/*** VARIABLES ***/

decode_results results;
Servo myservo1;
Servo myservo2;

/*** SETUP ***/

void setup() {
  Serial.begin(9600); // Begin Serial Monitor
  
  // IR Sensor
  irrecv.enableIRIn(); // Start the receiver

  // Lift Servos
  myservo1.attach(9);
  myservo2.attach(10);
  myservo1.write(100);
  myservo2.write(95);

  //
}

/**********************
 *     FUNCTIONS      *
 **********************/

/*** IR ***/
// Read in the IR sensor
int readIR(){
}

// Display IR reading on 7 segment
void displaySevenSeg(){
}

/*** RGB SENSOR ***/
// Detect green LED
boolean detectGreen(){
}

/*** SCOOP ***/
// Lift the scoop up
void liftScoop(int degree){
  logging("ACTION", "Scoop Lifting");
  myservo1.write(degree);
  myservo2.write(degree);
}

// Lower the scoop down
void lowerScoop(int degree){
  logging("ACTION", "Scoop Lowering");
  myservo1.write(degree);
  myservo2.write(degree);
}

/*** NAVIGATION ***/
// Turn the robot a certain degree
void turn(int degree){
}

// Move the robot forward
void forward(int distance){
}

// Move the robot backwards
void backward(int distance){
}

// Check position
void checkPosition(){
}

/*** FLAG ***/
// Turn the flag
void turnFlag(int distance){
}

/*** OTHER ***/
// logging
void logging(String type, String message){
  Serial.print(type);
  Serial.print("=>");
  Serial.println(message);
}

/*** LOOP ***/
void loop() {
  // put your main code here, to run repeatedly:
}
