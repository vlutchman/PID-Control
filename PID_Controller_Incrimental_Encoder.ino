#define FULL_ROT 19*200
#define dir A0
#define mot A1
#define clk_pin 7
#define DT_pin 6
#define degStep FULL_ROT/360
#include <Servo.h>
#include <math.h>
#include <PinChangeInterrupt.h>

void biStep(int dirPin,int motPin,int dir, int steps);

Servo gripper;
Servo base ;

int test = 0;
int timer2 =400;
volatile int CLK = 0;
volatile int DT= 0;
int prevState =0;
volatile int counter = 0;
int direction =0;
int position =0; 
int degSteps = 0;
int degCounter = 0;
int pos =0;
int mot_steps = 0;
int startTime = 0;
int endTime = 0;
volatile double dt = 0;
double Ki = 0.1;
double Kp =1;
double Kd = 0.01;

double error= 0;
double prevError =0;
double P =0;
double I = 0;
double D = 0;

double control = 0;

void setup() { 
  Serial.begin(9600);
  pinMode(clk_pin,INPUT);
  pinMode(DT_pin,INPUT);
  pinMode(dir,OUTPUT);
  pinMode(mot,OUTPUT);

  attachPinChangeInterrupt(digitalPinToPCINT(clk_pin),ISR_1,CHANGE);
  prevState = digitalRead(7);

}

void ISR_1(){
  startTime = micros();
  CLK = digitalRead(clk_pin);
  DT = digitalRead(DT_pin);
  if(CLK != prevState){
    if(DT == CLK){
      counter--;
    }else{
      counter++;
    }
  } 
  prevState = CLK;
  endTime = micros();
  dt = (endTime-startTime);
}


void loop() {
  //biStep(dir,mot,1,10);
  if(Serial.available() > 0){
    pos = Serial.parseInt();
     while (Serial.available() > 0) {
        Serial.read();  
    }
  }

  dt = dt*0.000001;
  mot_steps =map(pos,0,360,0,FULL_ROT);
  degCounter = map(counter,0,139,0,360);
  error = pos - degCounter;
  P = error*Kp;
  I += Ki*error*dt;
  D = Kd*(error-prevError)/dt; 

  Serial.println(error);
  Serial.println(counter);
  Serial.println(control);

  if(abs(error) > 2){
    control = P + I ;
    if(abs(error) <3){
      control = P + I + D;
    }
    if(control > 0){
      biStep(dir,mot,-1,ceil(control*degStep));
    }else if(control < 0){
      biStep(dir,mot,1,ceil(control*degStep));
    }else{
      biStep(dir,mot,1,0);
    }
  }
  

}


void biStep(int dirPin,int motPin,int dir, int steps){
  if(steps < 0){
    steps = -1*steps;
  }
  if(dir == 1){
    digitalWrite(dirPin,HIGH);
  }else if(dir == -1){
    digitalWrite(dirPin,LOW);
  }
  for(int i =0;i<=steps;i++){
  digitalWrite(motPin,HIGH);
  delayMicroseconds(timer2);
  digitalWrite(motPin,LOW);
  delayMicroseconds(timer2);
 }
 return;
}
