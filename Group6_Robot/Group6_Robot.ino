#include <Servo.h>

// SERVOS
#define lh_scoop  10
#define rh_scoop  9
#define flag_r    8
#define flag_p    7

Servo rh_servo;
Servo lh_servo;
Servo flag_servo;
Servo pos_servo;

int rh_servo_pos[3] = {2, 85, 50};    // SCOOP INITIALIZED (UP[0] ==> RH: 2 LH: 174 
int lh_servo_pos[3] = {174, 91, 126}; // DWN[1] ==> RH: 85 LH: 91 BTN[2] ==> RH: 50 LH: 126)


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
#define LOOPTIME        10            // PID loop time

unsigned long lastMilli = 0;          // loop timing
int speed_req = 50;                   // speed (Set Point)
int speed_act_rh = 0;                 // speed (actual value)
int speed_act_lh = 0;
int PWM_val_rh = 0;                   // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val_lh = 0;

unsigned int pulse_count = 0;
volatile long count_rh = 0;           // pulse counters
volatile long count_lh = 0;
unsigned long count_tot_rh = 0;
unsigned long count_tot_lh = 0;
float Kp_rh = .9;                     // PID proportional control Gain
float Kd_rh = .15;                    // PID Derivitave control gain
float Kp_lh = .9;
float Kd_lh = .15;


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
int button_state = 0;
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
int one, two, three, four, five, average;

void setup() {

  // SERVOS
  rh_servo.attach(rh_scoop);
  lh_servo.attach(lh_scoop);
  rh_servo.write(rh_servo_pos[0]);
  lh_servo.write(lh_servo_pos[0]);

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
  pinMode(chA_rh, INPUT_PULLUP);
  pinMode(chB_rh, INPUT_PULLUP);
  pinMode(chA_lh, INPUT_PULLUP);
  pinMode(chB_lh, INPUT_PULLUP);

  // MOTOR ENCODER PULL-UP SET
  digitalWrite(chA_rh, HIGH);
  digitalWrite(chB_rh, HIGH);
  digitalWrite(chA_lh, HIGH);
  digitalWrite(chB_lh, HIGH);

  // MOTOR ENCODER INTERRUPT DEFINITIONS
  attachInterrupt(chA_rh, RH_ENCODER, FALLING);
  attachInterrupt(chA_lh, LH_ENCODER, FALLING);

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
}

/*** LOOP ***/
void loop() {

  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //DROP SCOOP
  rh_servo.write(rh_servo_pos[1]);
  lh_servo.write(lh_servo_pos[1]);

  //FWD 30"
  FWD_REV_distance(30, 0);



  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //REV 30"
  FWD_REV_distance(30, 1);

  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //TURN RIGHT 90*
  TURN(90, 0);

  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //TURN LEFT 90*
  TURN(90, 1);

  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //TURN LEFT 180*
  TURN(180, 1);

  /* WAIT FOR START BUTTON */
  while (button_state != HIGH) {
    button_state = digitalRead(button);
  }
  button_state = 0;

  //TURN RIGHT 180*
  TURN(180, 0);
}



//-----------------------------------------------------------------------
//-------------------------FUNCTION--DECLARATIONS------------------------
//-----------------------------------------------------------------------

//-------------------------INTERUPTS--FOR--MOTOR--ENCODERS---------------


void RH_ENCODER() {
  count_rh++;
  count_tot_rh++;
}

void LH_ENCODER() {
  count_lh++;
  count_tot_lh++;
}

//-----------------------------------------------------------------------
//-------------------------MOTOR--CONTROL--FUNCTIONS---------------------
//-----------------------------------------------------------------------

//------------------------------UPDATE--PID------------------------------

// compute PWM value
int updatePid(int command, int targetValue, int currentValue, float Kp, float Kd)   {             
  // PID correction
  float pidTerm = 0;                                                            
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

  speed_act_rh = ((count_rh - countAnt_rh) * (60 * (1000 / LOOPTIME))) / (13 * 51); // 13 pulses X 51 gear ratio = 663 counts per output shaft rev
  speed_act_lh = ((count_lh - countAnt_lh) * (60 * (1000 / LOOPTIME))) / (13 * 51);
  countAnt_rh = count_rh;
  countAnt_lh = count_lh;
}

//------------------------------GET--PARAM--------------------------------

void getparam() {
  char param, cmd;

  if (!Serial.available()) return;
  delay(10);
  param = Serial.read();
  if (!Serial.available()) return;
  cmd = Serial.read();
  Serial.flush();
  switch (param)
  {
    case 'r':
      if (cmd == '+') {
        speed_req += 20;
        if (speed_req > 110) speed_req = 110;
      }
      if (cmd == '-') {
        speed_req -= 20;
        if (speed_req < 0) speed_req = 0;
      }
      break;
    default:
      Serial.println("???");
  }
}
//--------------------------------FORWARD--------------------------------
/*
   Move forward until object is "inchesAway"
*/
void forwardUntilObject(int inchesAway) {
  brake_lh();
  brake_rh();
  delay(100);
  int distance = 0;                                                         // initialize distance to check
  forward_rh();                                                             // move robot forward
  forward_lh();

  while (distance < inchesAway) {
    distance = distanceInInches(fr_trig, fr_echo);                          // find distance
    lastMilli = millis();
    getMotorData();                                                         // calculate speed, volts and Amps
    PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh);            // compute PWM value
    PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh);

    analogWrite(enA_rh, PWM_val_rh);
    analogWrite(enB_lh, PWM_val_lh);// send PWM to motor
  }

  brake_lh();
  brake_rh();
  delay(100);
  count_tot_lh = 0;                                                         // reset the count for the motors
  count_tot_rh = 0;
}

//dist input ===> inches
//dir input ===> 0 = FWD & 1 = REV
void FWD_REV_distance(int dist, int dir)  {
  pulse_count = 0;

  pulse_count = (dist * 663) / 9.8; // 663 pulses/rev approx 9.8"/rev

  //set motor direction
  if (dir == 0) {
    forward_rh();
    forward_lh();
  }
  else if (dir == 1) {
    reverse_rh();
    reverse_lh();
  }

  // GO SET DISTANCE pi*d = circ = distance/revolution ==> 3.18" * pi = 9.99"
  // 2000/663 = 3.02 rev * 9.99" = 30" distance traveled
  while (count_tot_rh < pulse_count && count_tot_lh < pulse_count) {
    // enter tmed loop
    if ((millis() - lastMilli) >= LOOPTIME)   {
      lastMilli = millis();

      // calculate speed of motors
      getMotorData();
      // compute PWM value
      PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh, Kp_rh, Kd_rh);
      PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh, Kp_lh, Kd_lh);

      // send PWM to motor
      analogWrite(enA_rh, PWM_val_rh);
      analogWrite(enB_lh, PWM_val_lh);
    }
  }
  //stop motors
  brake_lh();
  brake_rh();

  //reset pulse counter
  count_tot_rh = 0;
  count_tot_lh = 0;
}

/*
   Right motor forward
*/
void forward_rh() {
  digitalWrite(in1_rh, HIGH);
  digitalWrite(in2_rh, LOW);
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh,HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*
   Left Motor forward
*/
void forward_lh() {
  digitalWrite(in3_lh, HIGH);
  digitalWrite(in4_lh, LOW);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

//--------------------------------REVERSE--------------------------------

/*
   Move backwards until "inchesAway" from object
*/
void reverseUntilObject(int inchesAway) {
  brake_lh();
  brake_rh();
  delay(100);
  int distance = 0;                                                         // initialize distance to check
  reverse_lh();                                                             // move robot forward
  reverse_rh();

  while (distance < inchesAway) {
    distance = distanceInInches(fr_trig, fr_echo);                          // find distance
    lastMilli = millis();
    getMotorData();                                                         // calculate speed, volts and Amps
    PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh);            // compute PWM value
    PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh);

    analogWrite(enA_rh, PWM_val_rh);
    analogWrite(enB_lh, PWM_val_lh);// send PWM to motor
  }

  brake_lh();
  brake_rh();
  delay(100);
  count_tot_lh = 0;                                                         // reset the count for the motors
  count_tot_rh = 0;
}

/*
   Left motor Reverse
*/
void reverse_lh() {
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, HIGH);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*
   Right motor Reverse
*/
void reverse_rh() {
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, HIGH);
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

//----------------------------------TURNING-----------------------------

/*
     turns the robot 90 degrees in the direction indicated
*/
//dir input ===> 0 = RIGHT & 1 = LEFT
//deg input ===> 180 or 90 input
void TURN(int deg, int dir)  {

  //set direction of motors for turn
  //TURN RIGHT
  if (dir == 0) {
    reverse_rh();
    forward_lh();
  }
  //TURN LEFT
  else if (dir == 1) {
    reverse_lh();
    forward_rh();
  }

  //set pulse count for turn
  if (deg == 90) {
    pulse_count = 683;
  }
  else if (deg == 180) {
    pulse_count = 1290;
  }

  while (count_tot_rh < pulse_count && count_tot_lh < pulse_count) {
    // enter tmed loop
    if ((millis() - lastMilli) >= LOOPTIME)   {
      lastMilli = millis();

      // calculate speed of motors
      getMotorData();
      // compute PWM value
      PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh, Kp_rh, Kd_rh);
      PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh, Kp_lh, Kd_lh);

      // send PWM to motor
      analogWrite(enA_rh, PWM_val_rh);
      analogWrite(enB_lh, PWM_val_lh);
    }
  }
  brake_lh();
  brake_rh();
  count_tot_rh = 0;
  count_tot_lh = 0;
}

//----------------------------------BRAKE--------------------------------

/*
   Left motor brake
*/
void brake_lh() {
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, LOW);
  digitalWrite(enB_lh, LOW);
}

/*
   Right motor brake
*/
void brake_rh() {
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, LOW);
  digitalWrite(enA_rh, LOW);
}

//-----------------------------------------------------------------------
//-----------------------------IR/7SEG--FUCNTIONS------------------------
//-----------------------------------------------------------------------
/*** IR ***/
// Read in the IR sensor
int readIR() {
}

// Display IR reading on 7 segment
void displaySevenSeg() {
}
//-----------------------------------------------------------------------
//-------------------------------RGB--FUNCTIONS--------------------------
//-----------------------------------------------------------------------
// Detect green LED
boolean detectGreen() {
}

//-----------------------------------------------------------------------
//-----------------------------SCOOP--FUNCTIONS--------------------------
//-----------------------------------------------------------------------

// Lift the scoop up
void liftScoop(int degree) {
  logging("ACTION", "Scoop Lifting");
  //  myservo1.write(degree);                                               // No servo object created yet
  //  myservo2.write(degree);
}

// Lower the scoop down
void lowerScoop(int degree) {
  logging("ACTION", "Scoop Lowering");
  //myservo1.write(degree);                                               // No servo object created yet
  //myservo2.write(degree);
}

//-----------------------------------------------------------------------
//------------------------------FLAG--FUNCTIONS--------------------------
//-----------------------------------------------------------------------
// Turn the flag
void turnFlag(int distance) {
}

/*** OTHER ***/
// logging
void logging(String type, String message) {
  Serial.print(type);
  Serial.print("=>");
  Serial.println(message);
}

//-----------------------------------------------------------------------
//------------------------ULTRASONIC--SENSOR--FUNCTIONS------------------
//-----------------------------------------------------------------------
/**
   Gets distance from the HC-SR04

   operating voltage:   5V DC
   Operating current:   15mA
   Measure Angle:       15(degrees)
   Ranging Distance:    2cm - 4m
*/
int distanceInCm(int trigPin, int echoPin) {
  int duration, answer;

  // reset the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // send sound wave, trigPin to HIGH
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //set echo pin to receive
  duration = pulseIn(echoPin, HIGH);
  answer = duration * 0.034 / 2;

  return answer;
}

//-----------------------------------------------------------------------
/**
   Gets distance from the HC-SR04

   operating voltage:   5V DC
   Operating current:   15mA
   Measure Angle:       15(degrees)
   Ranging Distance:    2cm - 4m
*/
int distanceInInches(int trigPin, int echoPin) {
  int duration, answer;
  // reset the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // send sound wave, trigPin to HIGH
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //set echo pin to receive
  duration = pulseIn(echoPin, HIGH);
  answer = duration * 0.034 / 2;
  answer = answer * 0.3937;

  return answer;
}

//-----------------------------------------------------------------------
/**
   Displays the distance to the Serial Monitor

   Display Speed @ 3cm    ~= 15ms         | Because of speed of
   Display Speed @ 220cm  ~= 70ms         | sound
*/
void displayDistance(int trigPin, int echoPin, String s) {
  int answer;
  for (int i = 1; i < 6; i++) {
    answer = distanceInInches(trigPin, echoPin);

    // average out 5 readings to get a more accurate read
    if ( i == 1 ) {
      one = answer;
    } else if ( i == 2 ) {
      two = answer;
    } else if ( i == 3 ) {
      three = answer;
    } else if ( i == 4 ) {
      four = answer;
    } else {
      five = answer;
      average = (one + two + three + four + five) / 5; // average of last five pulses
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
   prints a seperator of lines to the Serial Monitor
*/
void printSeperator() {
  Serial.println("--------------------------------------");
}

