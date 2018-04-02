// SERVOS
#define lh_scoop  10
#define rh_scoop  9
#define flag_r    8
#define flag_p    7

// RGB SENSOR
/*
#define rgb_led   14
#define XXX
#define XXXX
#define XXXXX
#define XXXXXX
#define XXXXXXX
*/

// USRF SENSORS
#define fr_trig   23
#define fr_echo   25
#define rr_trig   27
#define rr_echo   29
#define lh_trig   37
#define lh_echo   39
#define rh_trig   47
#define rh_echo   49

// MTR_CTRL + ENCODERS
// RH MTR
#define in1_rh    30
#define in2_rh    28
#define enA_rh    3
#define chA_rh    53
#define chB_rh    51

// LH MTR
#define in3_lh    26
#define in4_lh    24
#define enB_lh    4
#define chA_lh    45  
#define chB_lh    41  

// GENERAL I/O
#define ir_in     32
#define button    34
#define led_out   58
#define bat_in    56

// 7_SEGMENT LED PINS
#define segA      38
#define segB      36
#define segC      48
#define segD      46
#define segE      44
#define segF      40
#define segG      42
#define segDp     50

/* VARIABLES */
#define LOOPTIME        10                     // PID loop time

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 75;                             // speed (Set Point)
int speed_act_rh = 0;                              // speed (actual value)
int speed_act_lh = 0;
int PWM_val_rh = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_lh = 0;

volatile long count_rh = 0;                        // rev counter
volatile long count_lh = 0;
volatile long count_tot_rh = 0;
volatile long count_tot_lh = 0;
float Kp =   .5;                                // PID proportional control Gain
float Kd =    .25;                                // PID Derivitave control gain

int one,two,three,four,five,average;

int button_state = 0;

void setup() {
  
  // BUTTON, LED, IR, & BATTERY INPUT
  pinMode(bat_in, INPUT);
  pinMode(ir_in, INPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(led_out, OUTPUT);
  
  // MTR_CTRL
  pinMode(in1_rh, OUTPUT);
  pinMode(in2_rh, OUTPUT);
  pinMode(enA_rh, OUTPUT);
  pinMode(in3_lh, OUTPUT);
  pinMode(in4_lh, OUTPUT);
  pinMode(enB_lh, OUTPUT);

  // ENCODERS
  pinMode(chA_rh, INPUT);
  pinMode(chB_rh, INPUT);
  pinMode(chA_lh, INPUT);
  pinMode(chB_lh, INPUT);

  /* MOTOR ENCODER PULL-UP SET */
  digitalWrite(chA_rh, HIGH);
  digitalWrite(chB_rh, HIGH);
  digitalWrite(chA_lh, HIGH);
  digitalWrite(chB_lh, HIGH);

  /* MOTOR ENCODER INTERRUPT DEFINITIONS */
  //attachInterrupt(chA_rh, RH_ENCODER, FALLING);
  //attachInterrupt(chA_lh, LH_ENCODER, FALLING);
  
  // USRF SENSORS
  pinMode(fr_trig, OUTPUT);
  pinMode(fr_echo, INPUT);
  pinMode(rr_trig, OUTPUT);
  pinMode(rr_echo, INPUT);
  pinMode(lh_trig, OUTPUT);
  pinMode(lh_echo, INPUT);
  pinMode(rh_trig, OUTPUT);
  pinMode(rh_echo, INPUT);

  // RGB SENSOR
  
  // 7_SEGMENT DISPLAY
  pinMode(segA, OUTPUT);
  pinMode(segB, OUTPUT);
  pinMode(segC, OUTPUT);
  pinMode(segD, OUTPUT);
  pinMode(segE, OUTPUT);
  pinMode(segF, OUTPUT);
  pinMode(segG, OUTPUT);
  pinMode(segDp, OUTPUT);

  // SERIAL DATA
  Serial.begin(115600);

  
  //forward_rh();                                   // Not sure the reason for initializing the motors to begin moving
  //forward_lh();                                   // before loop starts???
}



/**********************
 *     FUNCTIONS      *
 **********************/

/*** LOOP ***/
void loop() {
  // put your main code here, to run repeatedly:
  
}
//-----------------------------------------------------------------------
//-------------------------FUNCTION--DECLARATIONS------------------------
//-----------------------------------------------------------------------

//-------------------------INTERUPTS--FOR--MOTOR--ENCODERS---------------


void RH_ENCODER(){
  count_rh++;
  count_tot_rh++;
}

void LH_ENCODER(){
  count_lh++;
  count_tot_lh++;
}

//-----------------------------------------------------------------------
//-------------------------MOTOR--CONTROL--FUNCTIONS---------------------
//-----------------------------------------------------------------------

//------------------------------UPDATE--PID------------------------------


int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  int error=0;                                  
  static int last_error=0;                             
    
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

//----------------------------GET--MOTOR--DATA---------------------------

void getMotorData()  {                                                        // calculate speed
  static long countAnt_rh = 0;                                                   // last count
  static long countAnt_lh = 0;
  
  speed_act_rh = ((count_rh - countAnt_rh)*(60*(1000/LOOPTIME)))/(13*51);          // 13 pulses X 51 gear ratio = 663 counts per output shaft rev
  speed_act_lh = ((count_lh - countAnt_lh)*(60*(1000/LOOPTIME)))/(13*51);
  countAnt_rh = count_rh; 
  countAnt_lh = count_lh;                 
}

//------------------------------GET--PARAM--------------------------------

void getparam(){
  char param, cmd;

  if(!Serial.available()) return;
  delay(10);
  param = Serial.read();
  if(!Serial.available()) return;
  cmd = Serial.read();
  Serial.flush();
  switch(param)
  {
    case 'r':
      if(cmd == '+'){
        speed_req += 20;
        if(speed_req > 110) speed_req = 110;  
      }
      if(cmd == '-'){
        speed_req -= 20;
        if(speed_req < 0) speed_req = 0;
      }
      break;
    default:
      Serial.println("???");
  }
}
//--------------------------------FORWARD--------------------------------
/*
 *  Moves the robot forward for 'x' amount of distance in inches. 
 *  Takes that distance and turns it into ticks for the motor controller
 *  to work with
 */
void forward(long distance){
    long x = (distance) / (3.75 * 3.14);                                        // amount of ticks to travel for that distance

    while(count_tot_rh < x){
      lastMilli = millis();
      getMotorData();                                                         // calculate speed, volts and Amps
      PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh);            // compute PWM value
      PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh);
      
      analogWrite(enA_rh, PWM_val_rh); 
      analogWrite(enB_lh, PWM_val_lh);// send PWM to motor
    }
}


/*
 * Right motor forward
 */
void forward_rh(){
  digitalWrite(in1_rh,HIGH); 
  digitalWrite(in2_rh,LOW);  
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh,HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*
 * Left Motor forward
 */
void forward_lh(){
  digitalWrite(in3_lh, HIGH);
  digitalWrite(in4_lh, LOW);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

//--------------------------------REVERSE--------------------------------

/*
 * Left motor Reverse
 */
void reverse_lh(){
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, HIGH);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*
 * Right motor Reverse
 */
void reverse_rh(){
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, HIGH);
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

//----------------------------------BRAKE--------------------------------

/*
 * Left motor brake
 */
void brake_lh(){
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, LOW);
  digitalWrite(enB_lh, LOW);
}

/*
 * Right motor brake
 */
void brake_rh(){
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, LOW);
  digitalWrite(enA_rh, LOW);
}

//-----------------------------------------------------------------------
//-----------------------------IR/7SEG--FUCNTIONS------------------------
//-----------------------------------------------------------------------
/*** IR ***/
// Read in the IR sensor
int readIR(){
}

// Display IR reading on 7 segment
void displaySevenSeg(){
}
//-----------------------------------------------------------------------
//-------------------------------RGB--FUNCTIONS--------------------------
//-----------------------------------------------------------------------
// Detect green LED
boolean detectGreen(){
}

//-----------------------------------------------------------------------
//-----------------------------SCOOP--FUNCTIONS--------------------------
//-----------------------------------------------------------------------

// Lift the scoop up
void liftScoop(int degree){
  logging("ACTION", "Scoop Lifting");
//  myservo1.write(degree);                                               // No servo object created yet
//  myservo2.write(degree);
}

// Lower the scoop down
void lowerScoop(int degree){
  logging("ACTION", "Scoop Lowering");
  //myservo1.write(degree);                                               // No servo object created yet
  //myservo2.write(degree);
}

//-----------------------------------------------------------------------
//------------------------------FLAG--FUNCTIONS--------------------------
//-----------------------------------------------------------------------
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

//-----------------------------------------------------------------------
//------------------------ULTRASONIC--SENSOR--FUNCTIONS------------------
//-----------------------------------------------------------------------
/**
 * Gets distance from the HC-SR04
 * 
 * operating voltage:   5V DC
 * Operating current:   15mA
 * Measure Angle:       15(degrees)
 * Ranging Distance:    2cm - 4m
 */
int distanceInCm(int trigPin,int echoPin){
  int duration, answer;
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
  // reset the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // send sound wave, trigPin to HIGH
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //set echo pin to receive
  duration = pulseIn(echoPin, HIGH);
  answer = duration*0.034/2;

  return answer;
}

//-----------------------------------------------------------------------
/**
 * Gets distance from the HC-SR04
 * 
 * operating voltage:   5V DC
 * Operating current:   15mA
 * Measure Angle:       15(degrees)
 * Ranging Distance:    2cm - 4m
 */
int distanceInInches(int trigPin,int echoPin){
  int duration, answer;
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
  // reset the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // send sound wave, trigPin to HIGH
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //set echo pin to receive
  duration = pulseIn(echoPin, HIGH);
  answer = duration*0.034/2;
  answer = answer * 0.3937;

  return answer;
}

//-----------------------------------------------------------------------
/**
 * Displays the distance to the Serial Monitor
 * 
 * Display Speed @ 3cm    ~= 15ms         | Because of speed of
 * Display Speed @ 220cm  ~= 70ms         | sound 
 */
void displayDistance(int trigPin, int echoPin, String s){
  int answer;
  for(int i = 1; i < 6; i++){
    answer = distanceInInches(trigPin, echoPin);

    // average out 5 readings to get a more accurate read
    if( i == 1 ){
      one = answer; 
    } else if( i == 2 ){
      two = answer;
    } else if( i == 3 ){
      three = answer;
    } else if( i == 4 ){
      four = answer;
    } else{
      five = answer;
      average = (one+two+three+four+five)/5;    // average of last five pulses
      Serial.print(s);
      Serial.print(average);
      Serial.println(" in ");
    }
  }
}

//-----------------------------------------------------------------------
//-------------------------LINE--SEPERATOR--FOR--TESTING-----------------
//-----------------------------------------------------------------------
/**
 * prints a seperator of lines to the Serial Monitor
 */
void printSeperator(){
  Serial.println("--------------------------------------");
}

