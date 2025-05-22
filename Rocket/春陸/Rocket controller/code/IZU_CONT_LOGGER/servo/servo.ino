#include <Servo.h>
Servo myservo;
const int servo=5;
void setup() {
  myservo.attach(servo,500,2400);
  pinMode(4,INPUT);
  Serial.begin(9600);
  }

void loop() {
Serial.println(1234);
myservo.write(180);
delay(1000);
myservo.write(105);
delay(3000);
myservo.write(180);
delay(1000);

}
