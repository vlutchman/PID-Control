#define FULL_ROT 19*200
#define dir A0
#define mot A1
#define pot_pin A2
#define degStep FULL_ROT/360
#include <Servo.h>
#include <math.h>

void biStep(int dirPin,int motPin,int dir, int steps);

Servo gripper;
Servo base ;

int timer2 =400;
int prevState =0;
int direction =0;
int position =0; 
int degSteps = 0;
int potPos = 0;
int potDeg =0;
int mot_steps = 0;
int pos =90;
double Ki = 0.005;
double Kp =0.85;
double Kd = 0.000001;

int potValues[10];
int averagePotValue = 0;
double dt = 0;
double error= 0;
double prevError =0;
double P =0;
double I = 0;
double D = 0;

double control = 0;

void setup() { 
  Serial.begin(9600);
  pinMode(pot_pin,INPUT);
  pinMode(dir,OUTPUT);
  pinMode(mot,OUTPUT);
   
}


void loop() {
  //biStep(dir,mot,1,10);
  if(Serial.available() > 0){
    pos = Serial.parseInt();
     while (Serial.available() > 0) {
        Serial.read();  
    }
  }


  for (int i = 0; i < 10; i++) {
    potValues[i] = analogRead(pot_pin);
    delay(5);  
  }
  
  averagePotValue = average(potValues, 10);
  
  potDeg = map(averagePotValue,6,1010,0,300);
  
  error = pos-potDeg;
  Serial.print("Set Position : ");
  Serial.println(pos);
  Serial.print("Potentiometer angle : ");
  Serial.println(potDeg);
  Serial.print("Error : ");
  Serial.println(error);

  
  //delay(1000);
  
  dt = timer2*0.000001;
  mot_steps =map(pos,0,360,0,FULL_ROT);
  error = pos - potDeg;
  P = error*Kp;
  I += Ki*error*dt;
  D = Kd*(error-prevError)/dt; 


  if(abs(error) > 3){
    control = P + I ;
    if(abs(error) <4){
      control = P + I + D;
    }
    if(control > 0){
      biStep(dir,mot,1,ceil(control*degStep));
    }else if(control < 0){
      biStep(dir,mot,-1,ceil(control*degStep));
    }else{
      biStep(dir,mot,1,0);
    }
  }
  delay(100);
  
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


int average(int values[], int numValues) {
    long sum = 0;// Use 'long' to avoid overflow for larger arrays
    for (int i = 0; i < numValues; i++) {
        sum += values[i];
    }
    return sum / numValues; 
}
