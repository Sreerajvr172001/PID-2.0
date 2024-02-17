#include <EEPROM.h>
#define ENCA 2 
#define ENCB 3 

#define PWM 10
#define IN1 8
#define IN2 9

#define CPR 9360
#define FACTOR 6.5     
#define POT A0

float pos;
int deg2;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// PID constants
float kp = 1;
float ki = 0;
float kd = 0.09;

unsigned long previousMillis = 0;        // Store the last time the value was checked
const long interval = 100;             // Time interval in milliseconds (5 seconds here)
int variableToCheck = 0;                // The variable to monitor
int lastValue = -1;                     // Store the previous value for comparison

void setup() {
  Serial.begin(9600);
  pos=map(EEPROM.read(0),0,255,0,360) * FACTOR;
  Serial.println("     ");
  Serial.print("POS=  ");
  Serial.println(pos/FACTOR);  
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(POT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  
  PID(map(analogRead(A0),0,1023,0,360));


  /*
  while(Serial.available()!=0){

    deg2=Serial.parseInt();
    Serial.print("ANGLE=   ");
    Serial.println(deg2);
    if(deg2!=0){
      PID(deg2);
      delay(2000);
      Serial.println("Returned");
    }
  }
  Serial.print("Waiting ");
  Serial.println(deg2);
  */

  //int deg=map(analogRead(POT),0,1023,0,360);



}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

int PID(int deg){
  float target = deg*FACTOR;  
  

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;


  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }


  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  // store previous error
  eprev = e;
  //Serial.print("Target: ");

  Serial.print(target/FACTOR);
  Serial.print(" ");
  //Serial.print("Position: ");
  Serial.print(pos/FACTOR);
  Serial.println();
  EEPROM.update(0, map(pos/FACTOR,0,360,0,255));
  //Serial.print("EEPROM STORED=  ");
  //Serial.println(map(EEPROM.read(0),0,255,0,360));
  //Serial.println("DONE......................");

}

int constant_check(){
  unsigned long currentMillis = millis();  // Get the current time

  if (currentMillis - previousMillis >= interval) {  
    previousMillis = currentMillis;
    int currentValue = pos; //pos
    if (currentValue == lastValue) {    
      return 1;     
      
    } 
    lastValue = currentValue;
    return 0;
    //Serial.println("Value has changed");  
  } 

}