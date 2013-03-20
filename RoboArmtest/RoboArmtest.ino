#include <RoboArm.h>
#include <Component.h>
#include <Servo.h>
#include <array>


Servo elbo2,wrist2,grip2,wtwist2,should1,should0, base2;
Component *compList; 
int rota;
bool r = true;

void setup(){
  Serial.begin(9600);
  elbo2.attach(6);
  wtwist2.attach(3);
  should1.attach(7);
  should0.attach(8);
  base2.attach(9);
  wrist2.attach(4);
  grip2.attach(2);
  
}

void loop(){
   rota = 0;
  //Component comp3(should,10.2,1);
  //Component comp2(elbo,3,30,3.1,2);

  Component gripx(grip2,22,102,3.5,4,0,0.705,2.4,44.6,38.8);
  Component wtwistx(wtwist2,11,171,0.25,3,0,0.705,0.25,44.6,38.8);
  Component wristx(wrist2,10,170,0.5,2,0,0.705,0.25,44.6,38.8);
  Component elbox(elbo2,10,170,3.5,1,120,1.446,0.388,90.22,72.2);
  Component shouldx(should1,5,175,5.75,0,0,1.446,0.63,90.22,72.2);
  Component basex(base2,0,175,0,-1,0,0,0,0,0);
  Component shouldx0(should0,5,175,0,0);
  
 RoboArm a(shouldx,elbox,wristx,wtwistx,gripx,basex,true,false,true,shouldx0);
 //RoboArm a(wristx,gripx);
 
/*
if(r){
if(wtwist2.read() > 30){
  rota = a.spinTrue(20);
//rota = a.rotateTrue(10, *a.wtwist);
//wtwist2.write(25);
}
else{
r=false;
delay(1000);
}
}
else if(!r){
if(wtwist2.read() < 160){
  rota = a.spinTrue(165);
  //rota = a.rotateTrue(150, *a.wtwist);
//wtwist2.write(95);
}
else{
r=true;
delay(1000);
}
}
*/


  //rota = a.rotate(-1, *a.elbo);
  //Serial.println(rota);
  //Serial.println(rota);
  int valS = 20;
  int valElb = 0;
  
  if(valElb >170){
   valElb = 170; 
  }
  
  
  a.getState();
  
  //test rArm function
  //float dista = a.rArm(0);
  //Serial.println(dista);
  
  
  a.grasp(35);
  //grip2.write(30);
  wtwist2.write(173); //90 is flat servo on top, looking down the arm towards the grip a value under 90 or decreasing is a couter-clockwise rotation,
  wrist2.write(0); //DO NOT WRITE OVER 170
  elbo2.write(180); //straight in line with shoulder joint at 85, 0 for 90' back
  base2.write(0);
  should1.write(valS); //write 80 to this servo for vertical placement, 170 for striaght out front
  should0.write(181-valS); //write 101 to this servo for vertical placement, 11 for straight
  
  
  
  

    delay(20);
  

}


