#define SAMPLENUM 20
#define settime 10600
int AIN1=5;
int AIN2=3;
int count=0;
int fpstate=0;
int Fpstate=1;
int fpStateArray[SAMPLENUM];
int state=0;
int alt_flag=0;
bool status;
int starttime,nowtime;
int Time;
int mode=0;

float calcMedian(void *array, int n, int type) {
  if (type == 0) { // If data type is int
    int *intArray = (int*) array;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (intArray[i] > intArray[j]) {
          int changer = intArray[j];
          intArray[j] = intArray[i];
          intArray[i] = changer;
        }
      }
    }
    if (n % 2 == 0) {
      return (float) (intArray[n / 2] + intArray[n / 2 - 1]) / 2;
    } else {
      return (float) intArray[n / 2];
    }
  } else if (type == 1) { // If data type is float
    float *floatArray = (float*) array;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (floatArray[i] > floatArray[j]) {
          float changer = floatArray[j];
          floatArray[j] = floatArray[i];
          floatArray[i] = changer;
        }
      }
    }
    if (n % 2 == 0) {
      return (floatArray[n / 2] + floatArray[n / 2 - 1]) / 2;
    } else {
      return floatArray[n / 2];
    }
  } else {
    // Error or unknown data type
    return 0.0;
  }
}

uint8_t fp (){
  int pinmode=0;
  int i = digitalRead(17);
  if(i==1){
    pinmode=0;
  }else if(i==0){
    pinmode=1;
  }
  return (uint8_t)pinmode;
}

int isLaunched(int FlighPinState) {
  fpStateArray[0] = FlighPinState;
	for (int i = (SAMPLENUM - 1); i > 0; i--) {
		fpStateArray[i] = fpStateArray[i - 1];
	}
  fpStateArray[0] = FlighPinState;
	if (calcMedian(fpStateArray, SAMPLENUM, 0) == 1) { //launched
		return 0;
	} else {
		return 1;
	}
}
void setup()
{ 
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(14,INPUT);
  pinMode(17,INPUT);
  pinMode(16,OUTPUT);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  Serial.begin(9600);

  pinMode(13,INPUT);
  pinMode(17,INPUT);
  
}

void loop()
{
fpstate = fp();
Fpstate = isLaunched(fpstate);
switch(mode){
case 0: 
if (Fpstate==0){
starttime=millis();
mode=1;
}
break;
case 1:
if (Fpstate==1){
mode = 0;
break;
} 
nowtime=millis();
Time=nowtime-starttime;
alt_flag=digitalRead(14);
if(alt_flag==1||Time>settime){
  Serial.print("open");
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(16,HIGH);
}
break;
}
}

