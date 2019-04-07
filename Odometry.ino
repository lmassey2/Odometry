/*Keeps track of forward kinematics for a robot with two motor wheels.
 * Two methods are covered; the analytical and matrices versions.
 * Differences between the two have comments before them stating what version
 * the following variables and methods are used for.
 */

#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <math.h>
#include <time.h>

//pin definitions
#define IRPIN_L 10
#define IRPIN_R 11

//constants
#define BASE_L 80   //distance between wheels(mm)
#define WHEEL_R 20  //radius of wheels(mm)
#define TICKS 2     //number of rising edge ticks of encoder
#define GEAR_RATIO 75.81    //gear ratio of motor
#define DELTA_THETA_R ((2*PI/TICKS)/GEAR_RATIO)   //radians wheel travels per tick
#define PT (WHEEL_R*DELTA_THETA_R)    //distance wheel travels per tick
#define PHI atan2(PT, BASE_L)   //angle change of robot per tick (radians)
//FOR ANALYTICAL METHOD
#define DELTA_X ((BASE_L/2)*sin(PHI))   //change in x-axis of robot per tick
#define DELTA_Y ((BASE_L/2)-((BASE_L/2)*cos(PHI)))    //change in y-axis of robot per tick
//FOR MATRIX METHOD
const Matrix<4,4> id_matrix = {   //identity matrix
   1, 0, 0, 0,
   0, 1, 0, 0,
   0, 0, 1, 0,i
   0, 0, 0, 1};
const Matrix<4,4> right_angle_matrix = {    //angle matrix for right wheel
   cos(PHI), -sin(PHI), 0, 0,
   sin(PHI), cos(PHI), 0, BASE_L/2,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> right_delta_matrix = {    //delta matrix for right wheel
   1, 0, 0, 0,
   0, 1, 0, -BASE_L,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> right_full_matrix = right_angle_matrix * right_delta_matrix;    //combination of both right wheel matrices

const Matrix<4,4> left_angle_matrix = {   //angle matrix for left wheel
   cos(-PHI), -sin(-PHI), 0, 0,
   sin(-PHI), cos(-PHI), 0, -BASE_L/2,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> left_delta_matrix = {   //delta matrix for left wheel
   1, 0, 0, 0,
   0, 1, 0, BASE_L,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> left_full_matrix = left_angle_matrix * left_delta_matrix;   //combination of both left wheel matrices


//global variables
//ANALYTICAL METHOD
volatile float x_global = 0;    //global frame x
volatile float y_global = 0;    //global frame x
volatile float phi_global = 0;    //global frame z angle (radians)
volatile float delta_x_prime = 0;   //total change of x
volatile float delta_y_prime = 0;   //total change of y
//MATRIX METHOD
Matrix<4,4> TGR = id_matrix;    //matrix for robot's position and angle (global to robot)
volatile double matrix_delta_x = 0;   //delta x prime found using the matrix method (TGR[0][4])
volatile double matrix_delta_y = 0;   //delta y prime found using the matrix method (TGR[1][4])
volatile double matrix_z_angle = 0;   //current z angle after using inverse trig of a value in the rotation matrix of TGR (radians)



void setup() {
  //setup IR sensor pins for encoder wheels, each with their own interrupt routines
  pinMode(IRPIN_R, INPUT);
  pinMode(IRPIN_L, INPUT);
  attachInterrupt(IRPIN_R, rightISR, RISING);
  attachInterrupt(IRPIN_L, leftISR, RISING);
}

void loop() {
  //ANALYTICAL METHOD
  //constantly keeping the delta prime x and y values up to date
  delta_x_prime = ((DELTA_X*cos(phi_global)) + (DELTA_Y*sin(phi_global)));
  delta_y_prime = ((DELTA_Y*cos(phi_global)) + (DELTA_X*sin(phi_global)));

  //MATRIX METHOD
  //constantly keeping position and angle up to date base on the TGR matrix
  matrix_delta_x = TGR(0,4);
  matrix_delta_y = TGR(1,4);
  matrix_z_angle = acos(TGR(0,0));
}

//interrupt routine for the right wheel
//updates current position and angle every right wheel tick based on predefined constants
void rightISR(){
  //ANALYTICAL METHOD
  x_global = x_global + DELTA_X;
  y_global = y_global + DELTA_Y;
  phi_global = phi_global + PHI;

  //MATRIX METHOD
  TGR = TGR * right_full_matrix;
}

//interrupt routine for the left wheel
//updates current position and angle every rleft wheel tick based on predefined constants
void leftISR(){
  //ANALYTICAL METHOD
  x_global = x_global + DELTA_X;
  y_global = y_global - DELTA_Y;
  phi_global = phi_global - PHI;

  //MATRIX METHOD
  TGR = TGR * left_full_matrix;
}
