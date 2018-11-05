#include <Servo.h> 
#include <Oscillator.h>
#include <US.h>
#include <Otto.h>
#include <EEPROM.h>
#include <LedMatrix.h>
#include <BatReader.h>
#include <SoftwareSerial.h> // bluetooth  
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
  #define PIN_YL 2 //servo[2]
  #define PIN_YR 3 //servo[3]
  #define PIN_RL 4 //servo[4]
  #define PIN_RR 5 //servo[5]
  #define RX 7
  #define TX 6
  
  SoftwareSerial bluetooth(RX,TX);
  Otto Otto;
  
/* 
pinos d11 signal
      d12 ground
*/
bool obstacleDetected = false;
bool btS = false; // bt = true, serial = false
int distance = 0;
char c,b;
const int buzzer = 11;
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
         }      //delay(10);         
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
       } 
    }
  }
}
