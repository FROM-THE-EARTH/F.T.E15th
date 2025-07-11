int AIN1=5;
int AIN2=3;
void setup() {
pinMode(AIN1,OUTPUT);
pinMode(AIN2,OUTPUT);
digitalWrite(AIN1,LOW);
digitalWrite(AIN2,LOW);
}

void loop() {
delay(15000);
digitalWrite(AIN1,HIGH);
digitalWrite(AIN2,LOW);
}
