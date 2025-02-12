#include <Servo.h>
Servo myservo;
const int servo=5;
void setup() {
  myservo.attach(servo,500,2400);
  }

void loop() {
delay(3000);
myservo.write(90);

}
