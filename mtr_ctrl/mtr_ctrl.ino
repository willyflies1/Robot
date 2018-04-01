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

  
  forward_rh();
  forward_lh();
}

void loop(){

  /* WAIT FOR START BUTTON */
  while(button_state != HIGH){
    button_state = digitalRead(button);
  }

  //getparam();

  // GO SET DISTANCE pi*d = circ = distance/revolution ==> 3.75" * pi = 11.77"
  while(count_tot_rh < 663){
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed, volts and Amps
    PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh);                        // compute PWM value
    PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh);
    
    analogWrite(enA_rh, PWM_val_rh); 
    analogWrite(enB_lh, PWM_val_lh);// send PWM to motor
  }
  }
  brake_lh();
  brake_rh();
  count_tot_rh = 0;
  count_tot_lh = 0;
  
  delay(1000);

  reverse_rh();
  forward_lh();
  
  // 180* turn
  while(count_tot_rh < 1320){
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed, volts and Amps
    PWM_val_rh = updatePid(PWM_val_rh, speed_req, speed_act_rh);                        // compute PWM value
    PWM_val_lh = updatePid(PWM_val_lh, speed_req, speed_act_lh);
    
    analogWrite(enA_rh, PWM_val_rh); 
    analogWrite(enB_lh, PWM_val_lh);// send PWM to motor
  }
  }
  brake_lh();
  brake_rh();
  count_tot_rh = 0;
  count_tot_lh = 0;
  
  delay(1000);

  forward_rh();
  forward_lh();
  
//  printMotorInfo();                                                           // display data
}

/*****************************************************************************************************/
/***********************************SERIAL DATA INFO**************************************************/
/*****************************************************************************************************/

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
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

void getMotorData()  {                                                        // calculate speed
  static long countAnt_rh = 0;                                                   // last count
  static long countAnt_lh = 0;
  
  speed_act_rh = ((count_rh - countAnt_rh)*(60*(1000/LOOPTIME)))/(13*51);          // 13 pulses X 51 gear ratio = 663 counts per output shaft rev
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
    case 0:
      digitalWrite(segA, HIGH);
      digitalWrite(segB, HIGH);
      digitalWrite(segC, HIGH);
      digitalWrite(segD, HIGH);
      digitalWrite(segE, HIGH);
      digitalWrite(segF, HIGH);
      digitalWrite(segG, LOW);
      digitalWrite(segDp, HIGH);
      break;
      
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
  }

}