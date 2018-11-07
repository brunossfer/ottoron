#include <Servo.h> 
#include <Oscillator.h>
#include <US.h>
#include <Otto.h>
#include <EEPROM.h>
#include <LedMatrix.h>
#include <BatReader.h>
#include <SoftwareSerial.h> // bluetooth  
#define N_SERVOS 4
//This is Otto!
//---------------------------------------------------------
//-- First step: Make sure the pins for servos are in the right position
/*
         --------------- 
        |     O   O     |
        |---------------|
YR 3==> |               | <== YL 2
         --------------- 
            ||     ||
RR 5==>   -----   ------  <== RL 4
         |-----   ------|
*/
#define EEPROM_TRIM false 

//OTTO.setTrims(-7,-4,-4,7);

  #define PIN_YL 2 //servo[2]
  #define PIN_YR 3 //servo[3]
  #define PIN_RL 4 //servo[4]
  #define PIN_RR 5 //servo[5]
  
  #define RX 7
  #define TX 6
  
  #define INTERVALTIME 10.0 

Oscillator servo[N_SERVOS];

void drunk (int tempo);
void noGravity(int tempo);

void run(int steps, int T=500);
void walk(int steps, int T=1000);
void backyard(int steps, int T=3000);
void backyardSlow(int steps, int T=5000);
void moonWalkLeft(int steps, int T=1000);
void moonWalkRight(int steps, int T=1000);
void crusaito(int steps, int T=1000);
void upDown(int steps, int T=1000);
void flapping(int steps, int T=1000);
void dance();
  SoftwareSerial bluetooth(RX,TX);
  Otto Otto;
  
/* 
pinos A2 signal
      d12 ground
*/
#define EEPROM_TRIM false 

bool obstacleDetected = false;
bool btS = false; // bt = true, serial = false
int distance = 0;
char c,b;
const int buzzer = A2;
///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){
  pinMode(12,OUTPUT); // ground gambiarra
  digitalWrite(12,LOW);
  delay(50);
  
  //Set the servo pins
  Otto.init(PIN_YL,PIN_YR,PIN_RL,PIN_RR,true);
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  Otto.home();
  delay(50);
}

int t=495;
double pause=0;
///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
//--Uncomment lines or add you own-------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
          //OBSTACLE MODE ON!!!! 
          obstacleMode();
}
///////////////////////////////////////////////////////////////////
//-- Function to avoid obstacles
void obstacleMode(){
   if(Otto.getDistance()<15){
      while(Otto.getDistance()<15){
        Serial.println(Otto.getDistance());
        Otto.home();
        delay(1000); 
      }
   }
   else{
     while(Otto.getDistance()>=15){
       if(Serial.available()){
         c = Serial.read();
         btS = false;
       }       
       if(bluetooth.available()){
         b = bluetooth.read();
         btS = true;
       }                  
      if(btS){ // se bluetooth         
         if(b == 'f'){
            Otto.walk(2,1000,1);
         }        
         else if(b =='t'){
           Otto.walk(2,1000,-1);
           b = 'h';
         }
         else if(b == 'h'){
           Otto.home();
         }
         else if(b=='e'){
           Otto.turn(2,1000,LEFT);
           b = 'h';
         }
         else if(b=='d'){
         Otto.turn(2,1000,RIGHT);
         b = 'h';
         }
         else if(b=='n'){
         dance();
         b = 'h';
         }
         //delay(10);         
       }
       else{  //se serial
         if(c =='w'){
            Otto.walk(2,1000,1);
         }      
         else if(c =='s'){
           Otto.walk(2,1000,-1);
           b = 'h';
         }
         else if(c == 'h'){
           Otto.home();           
         }
          else if(c=='e'){
           Otto.turn(2,1000,LEFT);
           c= 'h';
         }
         else if(c=='d'){
         Otto.turn(2,1000,RIGHT);
         c='h';
         }
         else if(c=='m'){
         dance();
         c= 'h';
         }        
       } 
    }
  }
}

void dance(){
  //Serial.println("aqui");
  int t = 495;
  double pause=0;
  primera_parte();
  segunda_parte();
  Otto.moonwalker(4,t*2,25,LEFT);
  Otto.moonwalker(4,t*2,25,RIGHT);
  Otto.moonwalker(4,t*2,25,LEFT);
  Otto.moonwalker(4,t*2,25,RIGHT);
  primera_parte(); 
  crusaito(1,t*8);
  crusaito(1,t*7);

  for (int i=0; i<16; i++){
    flapping(1,t/4);
    delay(3*t/4);
  } 
  Otto.moonwalker(4,t*2,25,RIGHT);
  Otto.moonwalker(4,t*2,25,LEFT);
  Otto.moonwalker(4,t*2,25,RIGHT);
  Otto.moonwalker(4,t*2,25,LEFT);

  drunk(t*4);
  drunk(t*4);
  drunk(t*4);
  drunk(t*4);
  drunk(t*8);
  drunk(t*4);
  drunk(t/2);
  delay(t*4); 

  drunk(t/2);

  delay(t*4); 
  walk(2,t*2);
  backyard(2,t*2);
  noGravity(t*2);
  crusaito(1,t*2);
  crusaito(1,t*8);
  crusaito(1,t*2);
  crusaito(1,t*8);
  crusaito(1,t*2);
  crusaito(1,t*3);
  delay(t);
  primera_parte();
    for (int i=0; i<32; i++){
    flapping(1,t/2);
    delay(t/2);
  }
  for(int i=0;i<4;i++) servo[i].SetPosition(90);
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////FUNCIONES DE CONTROL//////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

unsigned long final_time;
unsigned long interval_time;
int oneTime;
int iteration;
float increment[N_SERVOS]; 
int oldPosition[]={90,90,90,90};

//////////////////////////////////////////////////////////////////////////////
////////////////////////////////PASSOS DE BAILE////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void primera_parte(){  
  int move1[4] = {60,120,90,90};
  int move2[4] = {90,90,90,90};
  int move3[4] = {40,140,90,90};
  
  for(int x=0; x<3; x++){  
    pause=millis();
    for(int i=0;i<4;i++) servo[i].SetPosition(90);
    Otto._moveServos(t*0.4,move1);
    Otto._moveServos(t*0.4,move2);
    while(millis()<(pause+t*2));
  }
    
  pause=millis();
  for(int i=0;i<4;i++) servo[i].SetPosition(90);
  crusaito(1,t*1.4);
  Otto._moveServos(t*1,move3);
  for(int i=0;i<4;i++) servo[i].SetPosition(90);
  while(millis()<(pause+t*4));
}

void segunda_parte(){
  int move1[4] = {90,90,80,100};
  int move2[4] = {90,90,100,80};
  int move3[4] = {90,90,80,100};
  int move4[4] = {90,90,100,80};
    
  int move5[4] = {40,140,80,100};
  int move6[4] = {40,140,100,80};
  int move7[4] = {90,90,80,100};
  int move8[4] = {90,90,100,80};
       
  int move9[4] = {40,140,80,100};
  int move10[4] = {40,140,100,80};
  int move11[4] = {90,90,80,100};
  int move12[4] = {90,90,100,80};
  
  for(int x=0; x<7; x++){ 
    for(int i=0; i<3; i++){
      pause=millis();
      Otto._moveServos(t*0.15,move1);
      Otto._moveServos(t*0.15,move2);
      Otto._moveServos(t*0.15,move3);
      Otto._moveServos(t*0.15,move4);
      while(millis()<(pause+t));
    }
    pause=millis();
    Otto._moveServos(t*0.15,move5);
    Otto._moveServos(t*0.15,move6);
    Otto._moveServos(t*0.15,move7);
    Otto._moveServos(t*0.15,move8);
    while(millis()<(pause+t));
  }
 
  for(int i=0; i<3; i++){
    pause=millis();
    Otto._moveServos(t*0.15,move9);
    Otto._moveServos(t*0.15,move10);
    Otto._moveServos(t*0.15,move11);
    Otto._moveServos(t*0.15,move12);
    while(millis()<(pause+t));
  }
}

void drunk (int tempo){  
  pause=millis();
  
  int move1[] = {60,70,90,90};
  int move2[] = {110,120,90,90};
  int move3[] = {60,70,90,90};
  int move4[] = {110,120,90,90};
  
  Otto._moveServos(tempo*0.235,move1);
  Otto._moveServos(tempo*0.235,move2);
  Otto._moveServos(tempo*0.235,move3);
  Otto._moveServos(tempo*0.235,move4);
  while(millis()<(pause+tempo));
}

void noGravity(int tempo){  
  int move1[4] = {120,140,90,90};
  int move2[4] = {140,140,90,90};
  int move3[4] = {120,140,90,90};
  int move4[4] = {90,90,90,90};  
  
  for(int i=0;i<4;i++) servo[i].SetPosition(90);
  for(int i=0;i<N_SERVOS;i++) oldPosition[i]=90;
  Otto._moveServos(tempo*2,move1);
  Otto._moveServos(tempo*2,move2);
  delay(tempo*2);
  Otto._moveServos(tempo*2,move3);
  Otto._moveServos(tempo*2,move4);
}

void walk(int steps, int T){
Otto.walk(steps, T, 1);
}

void run(int steps, int T){
Otto.walk(steps, T/2, 1);
}

void backyard(int steps, int T){
Otto.walk(steps, T/2, -1);
}

void backyardSlow(int steps, int T){
Otto.walk(steps, T, -1);
}

void crusaito(int steps, int T){
Otto.crusaito(steps,T);
}

void upDown(int steps, int T){
Otto.updown(steps, T);
}

void flapping(int steps, int T){
Otto.flapping(steps, T);
}

