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
X = new float;
Y = new float;
R = new float;
T = new float;
P = new float;
compNum=5;
should2=should2x;
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
RoboArm::~RoboArm(){
delete X;
delete Y;
delete R;
delete T;
delete P;
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
compList = compL;  //compList is a pointer of type Component set now to the start of the compL array
}
RoboArm::RoboArm(){
Component compL[4];
compNum = 0;
compList = compL;
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
		polarToCart(compx.leng, sumDeg, tempx, tempy);
		sumX += (*tempx) / 2;
		sumY += (*tempy) / 2; //only add half to get to center of mass of link

		cartToPolar(sumX, sumY, tempr, tempt); // get distance from origonal servo directly to COM
		float torq1 = (*tempr * compx.massLink * cos(sumDeg * 0.0174532925)); //calculate torq of link
		sumX += (*tempx) / 2;
		sumY += (*tempy) / 2;
		cartToPolar(sumX, sumY, tempr, tempt);
		float torq2 = (*tempr * compx.massServo * cos(sumDeg * 0.0174532925));
		
		/*
		Serial.println(compx.leng);
		Serial.println("polar");
		Serial.println(sumDeg);
		Serial.println(sumX);
		Serial.println("sums");
		Serial.println(sumY);
		Serial.println(*tempx);
		Serial.println("temps");
		Serial.println(*tempy);
		Serial.println(" = ");		
		Serial.println(*tempr);
		Serial.println(*tempt);
        Serial.println(sumDeg);
		Serial.println(compx.currDeg);
		Serial.println("+");
		Serial.println(compx.massServo);
		Serial.println(torq1);
		Serial.println(torq2);
		*/
		sumTorq += torq1 +torq2;
        compList -= i;
	}
	sumTorq = sumTorq / 1000; 	//convert gf*cm to kgf*cm 
	float t = (*compList).torqMax;
	//Serial.println("torque");
	//Serial.println(sumTorq);
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
*P = (*base).serv.read();
(*should).currDeg = (*should).serv.read() + 10; //adjustment for servo being 90' at val 80
(*should0).currDeg = (*should0).serv.read() - 10; //adjustment for servo being 90' at val 101
(*elbo).currDeg = (*elbo).serv.read() + 5; //90' at val 85
(*wrist).currDeg = (*wrist).serv.read(); //ADD ADJUSTMENT
(*wtwist).currDeg = 90; //(*wtwist).serv.read(); //no adjustment needed, 90' is flat, rotates 90' either CC or CW
(*grip).currDeg = 90; //(*grip).serv.read();
//wrist and grip 90 so that the rArm function works
}

float RoboArm::rArm(float *x, float *y, float *r, float *t){
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
		polarToCart(compx.leng, sumDeg, tempx, tempy);
		sumX += (*tempx);
		sumY += (*tempy);
		cartToPolar(sumX, sumY, tempr, tempt);
        compList -= i;
	}
	
	*x = sumX;
	*y = sumY;
	*r = *tempr;
	*t = *tempt;
	/*
	*X = sumX;
	*Y = sumY;
	*R = *tempr;
	*T = *tempt;
	*/
	//memory mangement
	 delete tempx;
	 delete tempy;
	 delete tempr;
	 delete tempt;
	 
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
	//was x / y for torque calculations.....?????
   double t =  atan( y / x ); 
    t = t / 0.017453;
   if( (y > 0) and (x < 0)){
		t = abs(t);
		t = t + 90;
   }
   else if( (y < 0) and (x < 0)){
		t = t - 180;
   }
   
   Serial.println(t);
   	//convert radians to degrees
   *r = rd;
   *theta = t ;
}


int RoboArm::raise(int val){
   //getState(); //IMPLEMENT ONCE OUT OF TESTING
    float a = rArm(X,Y,R,T);
    float *tempr;
	float *tempt;
	tempr = new float;
	tempt = new float;
   cartToPolar(*X,((*Y) + val),tempr,tempt);
   reachTo(*tempr,*tempt,*P); //IMMPLEMEMT ONCE REACHTO IS COMPLETE
   Serial.println(*X);
   Serial.println(*Y);
   Serial.println(*tempr);
   Serial.println(*tempt);
   
   
   delete tempr;
   delete tempt;
}



float RoboArm::srminf(){
	float sqelb = sq((*elbo).leng);
	float sqwri = sq(((*wrist).leng + (*wtwist).leng + (*grip).leng));
	float hyp = sqrt( sqelb + sqwri);
	//float srmin = acos((*should).leng / hyp) / 0.017453; //srmin is the cutoff degree at the base
	return hyp;
}



void RoboArm::rs(float tr, float tt, float *a, float *b){
	float *tempx;
	float *tempy;
	float *tempr;
	float *tempt;
	tempx = new float;
	tempy = new float;
	tempr = new float;
	tempt = new float;
	polarToCart(tr,tt,tempx,tempy);
	float x = *tempx;
	Serial.println("xy from input");
	Serial.println(x);
	float y = *tempy;
	Serial.println(y);
	polarToCart((*should).leng,(*should).currDeg,tempx,tempy);
	
	float xd = abs(x - *tempx) ;
	Serial.println("xy from shoulder");
	Serial.println(*tempx);
	float yd = abs(y - *tempy);
	Serial.println(*tempy);
	Serial.println("xy difference");
	Serial.println(xd);
	Serial.println(yd);
	cartToPolar(xd,yd,tempr,tempt);
	float ts = *tempt;
	if( (y < *tempy) and (x < *tempx)){
		Serial.println(y); 
		Serial.println(*tempy);
		
		Serial.println("why are you running in here?");
		ts = ts + 180;
	}
	else if( (y > *tempy) and (x > *tempx)){
		ts = ts;
	}
	else if( (y > *tempy) and (x < *tempx)){
		ts = 180 - ts;
	}
	else if( (y < *tempy) and (x > *tempx)){
		ts = -ts;
	}
	else if(y == *tempy){
		if(x < *tempx){
		ts = 180;
		}
		else{
		ts = 0;
		}
	}
	else if(x == *tempx){
		if(y < *tempy){
		ts = -90;
		}
		else{
		ts = 90;
		}
	}
	*a = *tempr;
	*b = ts;

	//memory mangement
	 delete tempx;
	 delete tempy;
	 delete tempr;
	 delete tempt;

}
int RoboArm::findElbow(float rs,float ts,int wr){
	float *tempx;
	float *tempy;
	float *tempr;
	float *tempt;
	tempx = new float;
	tempy = new float;
	tempr = new float;
	tempt = new float;
	
	float L2 = (*wrist).leng + (*wtwist).leng + (*grip).leng;
	float L1 = (*elbo).leng;
	polarToCart(L2,wr,tempx,tempy);
	float Lnew = *tempy + L1;
	cartToPolar(*tempx,Lnew,tempr, tempt);

	
	
	//memory mangement
	 delete tempx;
	 delete tempy;
	 delete tempr;
	 delete tempt;
	 
	return (*tempt - ts);
}
 float RoboArm::cosLaw(float a,float b, float c){
	float val = ( square(a) + square(b) - square(c) ) / (2 * a * b); 
	float theta = acos(val);
	return theta / 0.017453;
 }




int RoboArm::reachTo(float tr,float tt, float tp){
	int sh = 90;
	int el = 90;
	int wr = 90;
	float *tempa;
	float *tempb;
	tempa = new float;
	tempb = new float;
	 //fancy math to reach and move the arm 
	 //try shoulder at 90 and see if it can reach
	float srm = (*elbo).leng + (*wrist).leng + (*wtwist).leng + (*grip).leng; //shoulder reach max max length arm can reach from shoulder joint
	float srmin = srminf();
	Serial.println(srm);
	Serial.println(srmin);
	rs(tr,tt,tempa,tempb);
	float rs = *tempa;
	float ts = *tempb;
	Serial.println(rs);
	Serial.println(ts);
	if((rs < srm) && (rs > srmin)){
		sh = 90;
		 Serial.println("Shoulder can be at 90'");

			 if((ts < 180) && (ts > 0)){
			 Serial.println("in full band region");
			 wr = cosLaw((*elbo).leng,(*wrist).leng + (*wtwist).leng + (*grip).leng, rs);
			 //srm-srmin compared to tr-srmin should give angle of wrist, experimental values tested and mapped to curve
			float ratr = (rs - srmin) / (srm - srmin);
			int out[] = {0,18,34,45,60,90};
			int in[]  = { 0,30,52,76,92,100};
			int ratri = ratr * 100;
			Serial.println(ratri);
			wr = multiMap(ratri,in,out,6); // faster at the beginning( when tr-srmin is small) and slower towards the top (when tr is gettign close to srm)
			Serial.println(wr);
			if(tt > 90){
				wr = 180 - wr;
			}
			Serial.println(wr);
			
			el = findElbow(rs,ts,wr);
			//only have to calculate elbow value now
			
			
			
			
			
		}
		 else{ //else statement for if reach is not in the constant band region
			//srm is not a constant in this region
			//IMPLEMENT SOME WAY TO TELL IF ITS STILL UNDER THE MAX
			Serial.println("not in full band region");
		 }
 
 
 
	}
	else{ //else stateemnt for if shoulder cannot be at 90'
	Serial.println("shoulder cant be at 90'");
 
	}
		//if it cant lower shoulder by 10 degrees and see if it can reach
 
 
 
 
	 rotateTrue(tp,*base);
	 rotateTrue(sh,*should);
	 if(should2){
		rotateTrue((181 - sh), *should0);
		}	
	 rotateTrue(el,*elbo);
	 rotateTrue(wr,*wrist);
	 Serial.print("shoulder value:    ");
	 Serial.println(sh);
	 Serial.print("elbow value:    ");
	 Serial.println(el);
	 Serial.print("wrist value:    ");
	 Serial.println(wr);
	 
	 
	 
	 delete tempa;
	 delete tempb;
	 
	 return 0;
}


int RoboArm::multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return map(val, _in[pos-1], _in[pos], _out[pos-1], _out[pos]);
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
raiseTrue(int val){
  //steal stuff from raise but dont subtract the arrays
}


*/
int RoboArm::scan(){
	//use rangefinder code for this
}
