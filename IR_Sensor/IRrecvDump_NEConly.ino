/*
   IRremote: IRrecvDump - dump details of IR codes with IRrecv
   An IR detector/demodulator must be connected to the input RECV_PIN.
   Version 0.1 July, 2009
   Copyright 2009 Ken Shirriff
   http://arcfn.com
   JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
   LG added by Darryl Smith (based on the JVC protocol)
*/

#include <IRremote.h>

/*
   Default is Arduino pin D11.
   You can change this to another available Arduino Pin.
   Your IR receiver should be connected to the pin defined here
*/
int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("READY");
}


void dump(decode_results *results) {
  // Dumps out the decode_results structure.
  // Call this after IRrecv::decode()
  int count = results->rawlen;
  if (results->decode_type == NEC) {
    Serial.print("Decoded NEC: ");
    Serial.print(results->value, BIN);
    Serial.print("(");
    Serial.print(results->bits, DEC);
    Serial.println(" bits)");
    Serial.println();
  }
}

void loop() {
  if (irrecv.decode(&results)) {
    dump(&results);
    irrecv.resume(); // Receive the next value
  }
}
