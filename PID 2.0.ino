#include <EEPROM.h> 
//Encoder pins
#define ENCA 2 
#define ENCB 3 
//Motor pins
#define PWM 10
#define IN1 8
#define IN2 9
//Joystick Pins
#define JOYSTICK A0

#define CPR 9360      //Counts Per Revolution 
#define PPR 9360/4    //Pulse Per Revolution
#define FACTOR 6.5    //Factor which is used to convert position from degree to PPR 


int deg ; //Target position in degree
long pos; //Current position in pulses

long prevT = 0;
float eprev = 0;
float eintegral = 0;

// PID constants
float kp = 1;
float ki = 0;
float kd = 0.09;

void setup() {

  Serial.begin(9600);

  //Read the previous position of the motor shaft from the EEPROM memory so that the reference position do not change (Absolute Servo)
  pos=map(EEPROM.read(0),0,255,0,360) * FACTOR; 
  Serial.println(" ");
  Serial.print("CURRENT POSITION =  ");
  Serial.println(pos/FACTOR);  
  deg=pos/FACTOR;
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  
  //Function for Clockwise Rotation
  if(analogRead(JOYSTICK)>800){ 
    deg+=10;    
  }
  //Function for Anti-Clockwise Rotation
  else if(analogRead(JOYSTICK)<300){
    deg-=10;        
  }
  //Upper limit = 360 degree
  if(deg>360)
    deg=360;
  //Lower limit =  0  degree
  if(deg<0)
    deg=0;

  PID(deg);  //PID Function

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
  //target position in terms of pulses
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

  // motor speed limiting
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // Sending signal to the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  // store previous error
  eprev = e;
  Serial.print("Target Angle = ");
  Serial.print(target/FACTOR); //Printing target position in degrees
  Serial.print("  ");
  Serial.print("Current Angle = ");
  Serial.print(pos/FACTOR);    //Printing current position in degrees
  Serial.println();

  EEPROM.update(0, map(pos/FACTOR,0,360,0,255)); //Store current position in EEPROM for reference incase of reset or power-off

}

