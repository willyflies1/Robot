#include <Servo.h>
#include <Wire.h>

// SERVOS
#define lh_scoop  10
#define rh_scoop  9
#define flag_rot  8
#define flag_pos  7

Servo rh_servo;
Servo lh_servo;
Servo flag_servo;
Servo pos_servo;

int rh_servo_pos[3] = {16, 97, 60};   // SCOOP INITIALIZED (UP[0] ==> RH: 2 LH: 174 
int lh_servo_pos[3] = {169, 88, 125}; // DWN[1] ==> RH: 85 LH: 91 BTN[2] ==> RH: 50 LH: 126)
int pos_servo_pos[4] = {10, 180, 0, 37};        // FWD[0] BKWD[1] RH_FWD[2] LH_FWD[3]

// RGB SENSOR
#define rgb_led   14
#define rgb_s3    15
#define rgb_s2    16
#define rgb_s1    17
#define rgb_s0    18
#define rgb_out   19

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

#define ir_in     32
#define button    34
#define led_out   58
#define bat_in    56

int bat_val = 0;
int button_state = 0;
int bat_timer = 0;

// 7_SEGMENT LED PINS
#define segA      38
#define segB      36
#define segC      48
#define segD      46
#define segE      44
#define segF      40
#define segG      42
#define segDp     50

// NAVIGATION VARIABLES
int route[8][6] = {{1, 0, 1, 0, 0, 0},    // 1 L R L R R R
                   {1, 0, 1, 0, 0, 1},    // 2 L R L R R L
                   {1, 0, 0, 1, 0, 0},    // 3 L R R L R R
                   {1, 0, 0, 1, 0, 1},    // 4 L R R L R L
                   {0, 1, 1, 0, 0, 0},    // 5 R L L R R R
                   {0, 1, 1, 0, 0, 1},    // 6 R L L R R L
                   {0, 1, 0, 1, 0, 0},    // 7 R L R L R R
                   {0, 1, 0, 1, 0, 1}};   // 8 R L R L R L
int distance[8] = {13, 42, 8, 27, 3, 2, 50, 0};

long random_num = 0;
int turn_num = 0;

void setup() {
  
  // SERVOS
  rh_servo.attach(rh_scoop, 570, 2400);
  lh_servo.attach(lh_scoop, 570, 2400);
  pos_servo.attach(flag_pos, 575, 2460);
  rh_servo.write(rh_servo_pos[0]);
  lh_servo.write(lh_servo_pos[0]);
  pos_servo.write(pos_servo_pos[0]);
  
  // BUTTON, LED, IR, & BATTERY INPUT
  pinMode(bat_in, INPUT);
  //pinMode(ir_in, INPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(led_out, OUTPUT);
  analogReadResolution(12);
  
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

  randomSeed(analogRead(0));
}

void loop(){
  
  random_num = random(8);
  seven_seg_disp(random_num+1);
  
  /* WAIT FOR START BUTTON */
  while(button_state != HIGH){
    button_state = digitalRead(button);
  }

  // TURN 1 TOWARDS DEST A
  if(turn_num == 0){
    if(route[random_num][turn_num] == 0)
      TURN(90, 0);
    else
      TURN(90, 1);
      turn_num++;
      delay(10);
  }
  
  //BUTTON SCOOP
  rh_servo.write(rh_servo_pos[2]);
  lh_servo.write(lh_servo_pos[2]);
    
  //FWD TO DEST A
  FWD_REV_distance(distance[0], 0);
  delay(10);
  
  //REV TO START 
  FWD_REV_distance(distance[0], 1);
  delay(10);
  
  //TURN 2 TOWARDS RAMP
  if(turn_num == 1){
    if(route[random_num][turn_num] == 0)
      TURN(90, 0);
    else
      TURN(90, 1);
      turn_num++;
      delay(10);
  }

  //FWD TO DEST B PARALLEL
  FWD_REV_distance(distance[1], 0);
  delay(10);

  //TURN 3 TOWARDS DEST B
  if(turn_num == 2){
    if(route[random_num][turn_num] == 0)
      TURN(90, 0);
    else
      TURN(90, 1);
      turn_num++;
      delay(10);
  }

  //FWD TO DEST B
  FWD_REV_distance(distance[2], 0);
  delay(5);
  //DROP SCOOP
  rh_servo.write(rh_servo_pos[1]);
  lh_servo.write(lh_servo_pos[1]);
  delay(250);

  //UP SCOOP
  rh_servo.write(rh_servo_pos[0]);
  lh_servo.write(lh_servo_pos[0]);
  delay(100);
  
  //REV TO CENTER
  FWD_REV_distance(distance[2], 1);
  delay(10);

  //TURN 4 TOWARDS TREASURE
  if(turn_num == 3){
    if(route[random_num][turn_num] == 0)
      TURN(90, 0);
    else
      TURN(90, 1);
      turn_num++;
      delay(10);
  }

  //DROP SCOOP
  rh_servo.write(rh_servo_pos[1]);
  lh_servo.write(lh_servo_pos[1]);
  delay(100);
  //FWD TO FLAG SCOOP TREASURE
  FWD_REV_distance(distance[3], 0);

  //BUTTON SCOOP
  rh_servo.write(rh_servo_pos[2]);
  lh_servo.write(lh_servo_pos[2]);
  delay(100);
  
  //TURN 180*
  TURN(180, 0);

  //FWD TO START
  FWD_REV_distance(distance[6], 0);

  //TURN 90* TOWARDS DEST C
  if(turn_num == 5){
    if(route[random_num][turn_num] == 0)
      TURN(90, 0);
    else
      TURN(90, 1);
      turn_num++;
      delay(10);
  }
  button_state = 0;
}

/*****************************************************************************************************/
/*************************************BAT CHECK*******************************************************/
/*****************************************************************************************************/

void BAT_check(){
  
  //APPROX 12.1V BATTERY VOLTAGE
  if(analogRead(bat_in) < 2300){
    digitalWrite(led_out, HIGH);
  }
  else
    digitalWrite(led_out, LOW);
}

/*****************************************************************************************************/
/*********************************TURN DIRECTION & DEGREE*********************************************/
/*****************************************************************************************************/

//dir input ===> 0 = RIGHT & 1 = LEFT
//deg input ===> 180 or 90 input
void TURN(int deg, int dir)  {

  //set direction of motors for turn
  //TURN RIGHT
  if(dir == 0){
    reverse_rh();
    forward_lh();
  }
  //TURN LEFT
  else if(dir == 1){
    reverse_lh();
    forward_rh();
  }

  //set pulse count for turn
  if(deg == 90){
    pulse_count = 683;
  }
  else if(deg == 180){
    pulse_count = 1290;
  }
  
  while(count_tot_rh < pulse_count && count_tot_lh < pulse_count){
  // enter tmed loop
  if((millis()-lastMilli) >= LOOPTIME)   {                                    
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

/*****************************************************************************************************/
/***********************************FWD/REV X DISTANCE************************************************/
/*****************************************************************************************************/

//dist input ===> inches
//dir input ===> 0 = FWD & 1 = REV
void FWD_REV_distance(int dist, int dir)  {
  pulse_count = 0;
  
  pulse_count = (dist*663)/9.8; // 663 pulses/rev approx 9.8"/rev 

  //set motor direction
  if(dir == 0){
    forward_rh();
    forward_lh();
  }
  else if(dir == 1){
    reverse_rh();
    reverse_lh();
  }
  
  // GO SET DISTANCE pi*d = circ = distance/revolution ==> 3.18" * pi = 9.99" per rev
  // EXAMPLE 2000pulses / 663ppr = 3.02 rev * 9.99" = 30" distance traveled
  while(count_tot_rh < pulse_count && count_tot_lh < pulse_count){
  // enter tmed loop
  if((millis()-lastMilli) >= LOOPTIME)   {                                    
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
  
  //reset total pulse counter
  count_tot_rh = 0;
  count_tot_lh = 0;
}

/*****************************************************************************************************/
/***********************************UPDATE PID DATA***************************************************/
/*****************************************************************************************************/
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

/*****************************************************************************************************/
/***********************************MTR SPEED CALC****************************************************/
/*****************************************************************************************************/

void getMotorData()  {                    // calculate rpm
  static long countAnt_rh = 0;            // last count
  static long countAnt_lh = 0;

  // 13 pulses X 51:1 gear ratio = 663 counts per output shaft rev
  speed_act_rh = ((count_rh - countAnt_rh)*(60*(1000/LOOPTIME)))/(13*51);          
  speed_act_lh = ((count_lh - countAnt_lh)*(60*(1000/LOOPTIME)))/(13*51);
  
  countAnt_rh = count_rh; 
  countAnt_lh = count_lh;                 
}

/*****************************************************************************************************/
/***********************************SERIAL DATA INFO**************************************************/
/*****************************************************************************************************/

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

/*****************************************************************************************************/
/*************************************RH ENCODER INT**************************************************/
/*****************************************************************************************************/

void RH_ENCODER(){
  count_rh++;
  count_tot_rh++;
}

/*****************************************************************************************************/
/*************************************LH ENCODER INT**************************************************/
/*****************************************************************************************************/

void LH_ENCODER(){
  count_lh++;
  count_tot_lh++;
}

/*****************************************************************************************************/
/*************************************FWD MTR RH CTRL*************************************************/
/*****************************************************************************************************/

void forward_rh(){
  digitalWrite(in1_rh,HIGH); 
  digitalWrite(in2_rh,LOW);  
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh,HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*****************************************************************************************************/
/*************************************FWD MTR LH CTRL*************************************************/
/*****************************************************************************************************/

void forward_lh(){
  digitalWrite(in3_lh, HIGH);
  digitalWrite(in4_lh, LOW);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*****************************************************************************************************/
/*************************************REV MTR LH CTRL*************************************************/
/*****************************************************************************************************/

void reverse_lh(){
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, HIGH);
  //analogWrite(enB_lh, cmd);
  //digitalWrite(enB_lh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*****************************************************************************************************/
/*************************************REV MTR RH CTRL*************************************************/
/*****************************************************************************************************/

void reverse_rh(){
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, HIGH);
  //analogWrite(enA_rh, cmd);
  //digitalWrite(enA_rh, HIGH); //FULL SPEED @ HIGH==> use PWM for control
}

/*****************************************************************************************************/
/*************************************BRAKE MTR LH CTRL***********************************************/
/*****************************************************************************************************/

void brake_lh(){
  digitalWrite(in3_lh, LOW);
  digitalWrite(in4_lh, LOW);
  digitalWrite(enB_lh, LOW);
}

/*****************************************************************************************************/
/*************************************BRAKE MTR RH CTRL***********************************************/
/*****************************************************************************************************/

void brake_rh(){
  digitalWrite(in1_rh, LOW);
  digitalWrite(in2_rh, LOW);
  digitalWrite(enA_rh, LOW);
}

/*****************************************************************************************************/
/*************************************SEVEN SEGMENT OUTPUT********************************************/
/*****************************************************************************************************/
void seven_seg_disp(int map_code) {
  
  // Display 0-7 based on map code received
  switch (map_code)
  {   
    case 1:
      digitalWrite(segA, LOW);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, LOW);
      digitalWrite(segE, LOW);
      digitalWrite(segF, LOW);
      digitalWrite(segG, LOW);
      digitalWrite(segDp, HIGH);
      break;
                
    case 2:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, LOW);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, HIGH);
      digitalWrite(segF, LOW);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
                
    case 3:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, LOW);
      digitalWrite(segF, LOW);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
                
    case 4:
      digitalWrite(segA, LOW);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, LOW);
      digitalWrite(segE, LOW);
      digitalWrite(segF, HIGH);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
                
    case 5:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, LOW);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, LOW);
      digitalWrite(segF, HIGH);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
                
    case 6:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, LOW);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, HIGH);
      digitalWrite(segF, HIGH);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
    
    case 7:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, LOW);
      digitalWrite(segE, LOW);
      digitalWrite(segF, LOW);
      digitalWrite(segG, LOW);
      digitalWrite(segDp, HIGH);
      break;

    case 8:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, HIGH);
      digitalWrite(segF, HIGH);
      digitalWrite(segG, HIGH);
      digitalWrite(segDp, HIGH);
      break;
  }

}
