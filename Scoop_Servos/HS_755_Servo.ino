#include <Servo.h>

Servo myservo1;
Servo myservo2;

char incomingByte = 0;

void setup()
{
  Serial.begin(9600);
  myservo1.attach(9);
  myservo2.attach(10);
  myservo1.write(100);
  myservo2.write(95);
}

void loop()
{
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("Reading: ");
    Serial.println(incomingByte);
    if (incomingByte == '1') {
      Serial.print("Position 1 \n");
      myservo1.write(155);
      myservo2.write(35);
    }
    if (incomingByte == '0') {
      Serial.print("Position 0\n");
      myservo1.write(100);
      myservo2.write(95);
    }
    Serial.println("End");
  }
}
