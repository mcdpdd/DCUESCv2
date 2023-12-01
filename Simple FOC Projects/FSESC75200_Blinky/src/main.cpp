#include <Arduino.h>

#define LED1  PB_0 //GPIO DOUT
#define LED2  PB_1 //GPIO DOUT

void setup() {
  // put your setup code here, to run once:
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED1, !digitalRead(LED1));
  digitalWrite(LED2, !digitalRead(LED2));
  delay(500); 
}