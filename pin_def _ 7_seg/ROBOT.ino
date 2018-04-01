// SERVOS
#define lh_scoop  10
#define rh_scoop  9
#define flag_r    8
#define flag_p    7

// RGB SENSOR
#define rgb_led   14
#define XXX
#define XXXX
#define XXXXX
#define XXXXXX
#define XXXXXXX

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
#define in1_rh    28
#define in2_rh    30
#define enA_rh    3
#define chA_rh    
#define chB_rh    

// LH MTR
#define in3_lh    24
#define in4_lh    26
#define enB_lh    4
#define chA_lh    
#define chB_lh    

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
}

void loop(){
  /* LINE UP IR/BLINK LED */

  /* WAIT FOR START BUTTON */
  while(button_state != 1){
    button_state = digitalRead(button);
  }

  //If button pressed...
  if (buttonState == LOW) { 
    //...ones, turn led on!
    if ( flag == 0){
      digitalWrite(ledPin, HIGH);
      flag=1; //change flag variable
    }
    //...twice, turn led off!
    else if ( flag == 1){
      digitalWrite(ledPin, LOW);
      flag=0; //change flag variable again 
    }    
  }
  /* IR INPUT RECEIVE */
  /* DISPLAY CODE IN DECIMAL FORM */

  /* DETERMINE TURN DIRECTIONS */

  /* FORWARD TO EVEN WITH DESTINATION A/C */
  /* CHECK PROXIMITY REAR, LEFT, & RIGHT */

  /* TURN 90* TOWARDS DESTINATION A/C */
  /* CHECK PROXIMITY REAR, FRONT, & ONE SIDE */

  /* FORWARD TO DESTINATION A/C & LOWER SCOOP HALFWAY */
  /* CHECK PROXIMITY REAR, FRONT, & WALL SIDE */ 

  /* CHECK FOR COMPLETION FROM RGB SENSOR */

  /* REVERSE TO EVEN WITH RAMP */
  /* CHECK PROXIMITY REAR, FRONT, & WALL SIDE */ 
  
  /* TURN 90* TOWARDS DOWN RAMP */
  /* CHECK PROXIMITY REAR, LEFT, & RIGHT */
  
  /* FORWARD DOWN RAMP */
  /* CHECK PROXIMITY ALL */
  
  /* FORWARD TO EVEN WITH DESTINATION B */
  /* CHECK PROXIMITY ALL */
  
  /* TURN 90* TOWARDS DESTINATION B */\
  /* CHECK PROXIMITY ALL */
  
  /* FORWARD TO DESTINATION B */
  /* CHECK PROXIMITY ALL */

  /* LOWER SCOOP & CHECK FOR COMPLETION */

  /* REVERSE TO EVEN WITH TREASURE */
  /* CHECK PROXIMITY ALL */

  /* TURN 90* TOWARDS TREASEURE */
  /* CHECK PROXIMITY ALL */

  /* LOWER SCOOP COMPLETELY */

  /* FORWARD TOWARDS FLAG POLE */
  /* CHECK PROXIMITY ALL */

  /* LIFT SCOOP HALFWAY */

  /* TURN 180* BACK TOWARDS RAMP */
  /* CHECK PROXIMITY ALL */

  /* FORWARD TOWARDS RAMP */
  /* CHECK PROXIMITY ALL */

  /* FORWARD UP RAMP TO EVEN WITH A/C */
  /* CHECK PROXIMITY FRONT, LEFT & RIGHT */

  /* TURN 90* TOWARDS DESTINATION A/C */
  /* CHECK PROXIMITY REAR, FRONT, & WALL SIDE */ 

  /* FORWARD TO DESTINATION A/C */
  /* DANCE, DANCE, DANCE!!! */
  
}

void 7_seg_disp(map_code) {
  
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
