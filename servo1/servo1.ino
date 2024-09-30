#include<Servo.h>
Servo s;
int s1=9;
void setup() {
  s.attach(s1);
  // put your setup code here, to run once:

}

void loop() {
  s.write(90);
  delay(1000);
  // put your main code here, to run repeatedly:

}
