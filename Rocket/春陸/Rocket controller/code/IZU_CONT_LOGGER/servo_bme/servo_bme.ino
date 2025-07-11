#define SAMPLENUM 20
int AIN1=5;
int AIN2=3;
int count=0;
int fpstate=0;
int Fpstate=1;
int fpStateArray[SAMPLENUM];
int state=0;
int alt_flag=0;
bool status;

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
		return 1;
	} else {
		return 0;
	}
}
void setup()
{ 
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  Serial.begin(9600);

  pinMode(13,INPUT);
  pinMode(17,INPUT);
  
}

void loop()
{count++;
while(Fpstate==1){
fpstate = fp();
Fpstate = isLaunched(fpstate);
Serial.println("not launched");
}
Serial.println("launched");
alt_flag=digitalRead(13);
Serial.println(alt_ ASflag);

if(alt_flag==1){
  Serial.println("open");
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
}

delay(1000);
}


