/*
  RoboArm.h - Library for moving a robotic arm
  Created by Tim Hurd, February 12, 2013.
  
*/



#ifndef RoboArm_H
#define RoboArm_H

#include <Arduino.h>
#include <Servo.h>
#include <Component.h>
#include <array>

//const int wtwist = 5; //NEEDS TO BE CHANGED TEMP FIX FOR FOR MY ARM SETUP
//const int grip = 6;   //HAVE NAMES BE POINTERS TO THE COMPONENTS LIKE ROBOARM.COMPL[1]




class RoboArm {
public:

// variables
Component* grip;	//a pointer that will be set to the grip Component of the CompList array
Component* wtwist;	//a pointer that will be set to the wtwist Component of the CompList array
Component* wrist;	//a pointer that will be set to the wrist Component of the CompList array
Component* elbo;	//a pointer that will be set to the elbo Component of the CompList array
Component* should;	//a pointer that will be set to the should Component of the CompList array
Component* base;	//a pointer that will be set to the base Component of the CompList array
Component* should0;	//a pointer that will be set to the second shoulder Component of the CompList array
Component *compList; //a pointer that will be set to the start of the list of all the components
int compNum; //stores how many components are passed into the constructor
float endPointR;	//stores the distance value of the end point of the arm
float endPointT;	//stores the theta value of the end point of the arm


// constructors
RoboArm();
RoboArm(Component shouldx, Component gripx);
RoboArm(Component shouldx, Component elbox, Component wristx,Component wtwistx, Component gripx, Component basex, bool should2x, bool rangefinder, bool twistFunc, Component shouldx2);
//RoboArm(Component elbox, Component wristx, Component gripx);
//RoboArm(Component shouldx, Component elbox, Component wristx, Component gripx);


// functions
//tier one
float getRange(); //this function will simply get the current value from the rangefinder
int rotate(int val, Component cpnt); //will specifically rotate a single servo a set number of degrees from its current location.
int rotateTrue(int val, Component cpnt); //rotate a servo to specified location no matter what the current orientation is.
int grasp(); //close the gripper all the way at max speed
int releaseGrip(); //opens grip at maximum speed
bool gripClosed(); //returns true if grip is closed
bool gripOpen(); //returns true if grip completely open
void getState(); //this function will return an array of the state of each servo in the arm.
float maxLift(int x); //if user input all parameters in setup this function will calculate the maximum wieght the arm can lift.
void polarToCart(float r, int theta, float *sx, float *sy); //converts an array of three values to cartsian, IGNORES PHI
void cartToPolar(float x, float y, float *r, float *theta); //converts cartesian coordinates to polar, IGNORES PHI


//tier two     (need tier one functions to work)
int grasp(int width);
int spin(int x); // calls rotate but specifies whatever servo the wrist rotate is specified at
int spinTrue(int x); //rotate wrist to specified angle, regardless of current position
float maxLiftC();// will calculate the max lift of the arm at its current location.
float rArm(int x); //returns r coord of end of arm position, but changes the endpoint[r,theta] as well

//tier three  (uses two or more tier one functions or at least a tier two)
int scan(); //returns the location data of the closest object it finds
int reachTo(int r, int theta, int phi); //reaches arm so that gripper is right in front of location


//tier four    (requires tier three functions to work)
int raise(int val); //will move the shoulder and elbow and wrists to raise the object without changing it orientation to the ground other than the vertical distance
int scanMult(); //returns coordinate data of multiple objects in an array
int grasp(int height, int width); //grasp an object directly in front of the gripper at specified height and firmly at set width without crushing it
int graspAt(int x, int y, int z, int height, int width, int twist); //grasp an object at a given location height width and angle of grasping, angle is from looking straight at it and change is yaw like a barel roll
int graspLight(int height, int width); //grasps slowly and certain width and height

//tier five
int throwTo( int x, int y, int z, int height, int width); //throws 
int throwObject(int base, int should, int elbow, int wrist, int x, int y, int z); //throws the object to a specified location releasing at a specified orientation. this may or may not actually work, need to play around and test capabilities of arm 




private:
bool should2; //will be true if there are two shoulder servos acting together
bool twistFunc; //will be true if there is a gripper twist capability on the arm.
bool rangefinder; // will be true if there is a rangefinder component
int coordinates[3]; //stores coordinates of arm end position in spherical coordinates




void rangeSetup(int x, int y, int z); //sets position of rangefinder relative to base of arm. x is the distance forward from the base, y is any offset side to side (with right being positive)instead of being directly in line with the arm, z is the vertical difference from the base servo(positive is up).



};

#endif