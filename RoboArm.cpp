#include <array>
#include <Component.h>
#include <Servo.h>
#include <RoboArm.h>
#include <Arduino.h>


RoboArm::RoboArm(Component shouldx, Component elbox, Component wristx,Component wtwistx, Component gripx, Component basex, bool should2x, bool rangefinder, bool twistFunc, Component shouldx2){
Component compL[7];
compL[0] = shouldx;
compL[1] = elbox;
compL[2] = wristx;
compL[3] = wtwistx;
compL[4] = gripx;
compL[5] = basex;
compL[6] = shouldx2;
base = 		&compL[5];
should = 	&compL[0];
elbo = 		&compL[1];
wrist = 	&compL[2];
wtwist = 	&compL[3];
grip =		&compL[4];
should0 = 	&compL[6];
compNum=5;
should2=should2x;
endPointR = 0;
endPointT = 0;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
/*
RoboArm::RoboArm(Component shouldx, Component elbox, Component wristx, Component gripx, Component basex, bool shouldx2){
Component compL[5];
compL[0] = shouldx;
compL[1] = elbox;
compL[2] = wristx;
compL[3] = gripx;
compL[4] = basex;
compNum=4;
should2 = shouldx2;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
RoboArm::RoboArm(Component shouldx, Component elbox, Component wristx, Component gripx){
Component compL[4];
compL[0] = shouldx;
compL[1] = elbox;
compL[2] = wristx;
compL[3] = gripx;
compNum=4;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
RoboArm::RoboArm(Component elbox, Component wristx, Component gripx){
Component compL[3];
compL[0] = elbox;
compL[1] = wristx;
compL[2] = gripx;
compNum=3;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
*/
RoboArm::RoboArm(Component shouldx, Component gripx){

Component compL[2];
compL[0] = shouldx;
compL[1] = gripx;
should = &compL[0];
grip = &compL[1];
compNum=2;
should2=false;
endPointR = 0;
endPointT = 0;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
RoboArm::RoboArm(){
Component compL[4];
compNum = 0;
compList = compL;
endPointR = 0;
endPointT = 0;
}



 float RoboArm::maxLift(int x){ //assumes joint that you want is at the pointer of compList
	float sumLeng = 0;
	float sumTorq = 0;
	float g = 9.81;
	for(int i=x;i<compNum;i++){ //gets size of compL array that CompList is pointing to/ size of a component to get number of items in the array
		compList += i;
        Component compx = (*compList);
        float torq = (((compx.leng / 2) + sumLeng) * compx.massLink) + (sumLeng * compx.massServo); 	// (L*Ml*g)+(l*Ms*g) = (cm * g * m/s^2) + (cm * g * m/s^2 ) = gramforce*cm  
		Serial.println(compx.leng / 2);
		Serial.println(compx.massLink);
		Serial.println("+");
		Serial.println(compx.massServo);
		Serial.println(sumLeng);
		Serial.println(torq);
		sumLeng += compx.leng;
		sumTorq += torq;
        compList -= i;
	}
	sumTorq = sumTorq / 1000; 	//convert gf*cm to kgf*cm 
	compList += x;
	float t = (*compList).torqMax;
	compList -= x;
	Serial.println("torque");
	Serial.println(t);
	Serial.println(sumTorq);
	float to;
	if(should2 && x==0){
	to = (t * 2) - sumTorq;
	return to;
	}
	else{ 
	to = t - sumTorq;
	return to;
	}
	}
	
	
 float RoboArm::maxLiftC(){ //assumes joint that you want is at the pointer of compList
	float sumTorq = 0;
	float sumX = 0;
	float sumY = 0;
	float sumDeg = 90; //offset for first angle being true to the x,y axis
	float *tempx;
	float *tempy;
	tempx = new float;
	tempy = new float;
	float *tempr;
	float *tempt;
	tempr = new float;
	tempt = new float;
	
	for(int i=0;i<compNum;i++){ //gets size of compL array that CompList is pointing to/ size of a component to get number of items in the array
		compList += i;
        Component compx = (*compList);
		sumDeg+= compx.currDeg - 90;
		polarToCart(compx.leng / 2, sumDeg, tempx, tempy);
		sumX += (*tempx) / 2;
		sumY += (*tempy) / 2; //only add half to get to center of mass of link

		cartToPolar(sumX, sumY, tempr, tempt); // get distance from origonal servo directly to COM
		float torq1 = (*tempr * compx.massLink * cos(sumDeg * 0.0174532925)); //calculate torq of link
		sumX += (*tempx) / 2;
		sumY += (*tempy) / 2;
		cartToPolar(sumX, sumY, tempr, tempt);
		float torq2 = (*tempr * compx.massServo * cos(sumDeg * 0.0174532925));
	
		Serial.println(sumX);
		Serial.println(sumY);
		Serial.println(" = ");		
		Serial.println(*tempr);
        Serial.println(sumDeg);
		Serial.println(compx.currDeg);
		Serial.println("+");
		Serial.println(compx.massServo);
		Serial.println(torq1);
		Serial.println(torq2);

		sumTorq += torq1 +torq2;
        compList -= i;
	}
	sumTorq = sumTorq / 1000; 	//convert gf*cm to kgf*cm 
	float t = (*compList).torqMax;
	Serial.println("torque");
	Serial.println(t);
	Serial.println(sumTorq);
	float to;
	
	//memory mangement
	 delete tempx;
	 delete tempy;
	 delete tempr;
	 delete tempt;
	 
	//return values
	if(should2){
	to = (t * 2) - sumTorq;
	return to;
	}
	else{ 
	to = t - sumTorq;
	return to;
	}
	}
  
  //make sure to check for two shoulder setup flag
  
 
 
 int RoboArm::rotate(int val, Component cpnt){
   int flag=0;
   int writeVal;
   int currentVal = cpnt.serv.read();
   int servMax = cpnt.maxR;
   int servMin = cpnt.minR;
   if ((currentVal + val <= servMax) && (currentVal + val >= servMin)){
   writeVal = currentVal + val;
   flag = 0;
   }
   else if (currentVal + val > servMax){
   writeVal = servMax;
   flag = 1;
   }
   else if (currentVal + val < servMin){
   writeVal = servMin;
   flag = 1;
   }
   else{
     flag = 1;}
     
   cpnt.serv.write(writeVal);
   return flag;
 }
 int RoboArm::rotateTrue(int val, Component cpnt){
   if((val >= cpnt.minR) && (val <= cpnt.maxR)){
   cpnt.serv.write(val);
   return 0;
   }
   else
   return 1;
 }
 
 int RoboArm::spinTrue(int x){
  if((*wtwist).serv.attached())
    return rotateTrue(x, *wtwist);
}
 int RoboArm::spin(int x){
  if((*wtwist).serv.attached())
    return rotate(x, *wtwist);
}
bool RoboArm::gripClosed(){
if((*grip).serv.read() > 101)
	return true;
else
	return false;
}
bool RoboArm::gripOpen(){
if((*grip).serv.read() < 23)
	return true;
else
	return false;
}
int RoboArm::releaseGrip(){
  int val= (*grip).minR;
  return rotateTrue(val, *grip);
}
int RoboArm::grasp(){
  int valu = (*grip).maxR;
  return rotateTrue(valu, *grip);
}
int RoboArm::grasp(int width){
  //is correct claw should go from 0 to 2" which is about 50 mm
  //grip has 5mm of play when stationary
 int val = map(width,0,55,(*grip).maxR,(*grip).minR);
  return rotateTrue(val, *grip);
}

int RoboArm::grasp(int height, int width){
   releaseGrip();
   //raiseTrue(height); 							//NEED TO IMPLEMENT THIS FUNCTION
   return grasp(width);
}


void RoboArm::getState(){
(*base).currDeg = (*base).serv.read();
(*should).currDeg = (*should).serv.read() + 10; //adjustment for servo being 90' at val 80
(*should0).currDeg = (*should0).serv.read() - 10; //adjustment for servo being 90' at val 101
(*elbo).currDeg = (*elbo).serv.read() + 5; //90' at val 85
(*wrist).currDeg = (*wrist).serv.read(); //ADD ADJUSTMENT
(*wtwist).currDeg = 0; //(*wtwist).serv.read(); //no adjustment needed, 90' is flat, rotates 90' either CC or CW
(*grip).currDeg = 0; //(*grip).serv.read();
//wrist and grip 0 so that the rArm function works
}

float RoboArm::rArm(int x){
	float sumX = 0;
	float sumY = 0;
	int sumTheta = 0;
	float *r;
	float *theta;
	for(int i=x;i<compNum;i++){ //gets size of compL array that CompList is pointing to/ size of a component to get number of items in the array
		float *sx;
		float *sy;
		compList += i;
        Component compx = (*compList);
        polarToCart(compx.leng, compx.currDeg, sx, sy); //FIX WILL TREAT WRIST ROTATION AS A NORMAL THETA, NAD GRIPPER THETA
		sumX += *sx;
		sumY += *sy;
		sumTheta += compx.currDeg;
        compList -= i;
	}
	cartToPolar(sumX, sumY, r, theta);
	endPointR = *r;
	endPointT = *theta;
	return *r;
	
}
void RoboArm::polarToCart(float r, int theta, float *sx, float *sy){ //ignores base value, just updates values at sx and sy
	float rad = theta * 0.017453;
	double cr = cos(rad);
	double sr = sin(rad);
	float x = r * cr;
	float y = r * sr;
   *sx = x;
   *sy = y;
}

void RoboArm::cartToPolar(float x, float y, float *r, float *theta){
	int sqrttemp = (int) (x * x + y * y);
	double rd = pow(sqrttemp, 0.5); 
   double t =  atan( x / y ); //supposedly atan is supported????????????????
   t = t / 0.017453; 	//convert radians to degrees
   *r = rd;
   *theta = t ;
}

/*
int[] currentP(){
  //some fancy math with sperical coordinates nneds to give r, theta, phi
  coordinates = [0,0,0];
  coordinates[2] = base.serv.read() * 3.14159 / 180;
  coordinates[1] = 0; //dont quite know where to start here yet
  float r = 
  coordinates[0] = r; 
  return val;
}
raise(int val){
   int currentPosition[] = currentP();
   float tempPos[] = polarToCart(currentPosition);
   tempPos[1] += val;
   float newPos[] = cartToPolar(tempPos);
   reachTo(newPos);
}
raiseTrue(int val){
  //steal stuff from raise but dont subtract the arrays
}

int reachTo(float np[]){
 //fancy math to reach and move the arm 
 return 0;
}
*/
int RoboArm::scan(){
	//use rangefinder code for this
}
