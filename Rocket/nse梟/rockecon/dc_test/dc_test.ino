AIN1=5;
AIN2=3;
void setup() {
pinMode(AIN1,OUTPUT);
pinMode(AIN2,OUTPUT);
}

void loop() {
digitalWrite(AIN2,HIGH);
digitalWrite(AIN1,LOW);
}
