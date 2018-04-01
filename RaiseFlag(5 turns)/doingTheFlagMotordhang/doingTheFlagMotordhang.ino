/*Jarmon Browning
 * Moves HS-311 (Converted to continuous motor) to jog for 5 turns
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(8);  // attaches the servo on pin 8 to the servo object
}

void loop() {
  myservo.write(0);
  delay(4646);
  myservo.detach();
  delay(2000);
}

