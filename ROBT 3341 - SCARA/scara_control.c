/*|SCARA Controller|-----------------------------------------------------------
#
# Project: ROBT 3341 - SCARA Control
# Program: scara_control.c
#
# Description:
#	This program contains <Your Names>'s solution to ROBT 3341 - SCARA.
#
# Week 1 Tasks:
#	1. define constants
#	2. scaraMenu
#	3. resetCount
#	4. displayPosition
#
# Week 2 Tasks:
#	1. moveJoint
#	2. moveTCP
#	3. limpUnlimp
#	4. homeRobot
#	5. moveJoint + moveTCP only using setScaraSpeed
#
# Author: <Your Names>
# Last Modified: <Today>
# -----------------------------------------------------------------------------*/
#include "scara_control.h"
#include <stdbool.h>


/*|Global Variables|-----------------------------------------------------------*/
extern struct BRobot robot;


/*|displayPosition|------------------------------------------------------------
#
# Decription:
#	This function displays the encoder counts, joint angles, and coordinates.
#
# ---------------------------------------------------------------------------*/
void displayPosition(void) {
	printf("Display the Position");

	// Initialize Variables
	long countone = 0;
	long counttwo = 0;

	// Update and Display Position
	while (!_kbhit()) {
		// Capture Encoder Data
		countone = robot.encoder0;
		counttwo = robot.encoder1;

		// Display Results
		printf("Encoder Count 1: %ld", countone);
		printf("\nEncoder Count 2: %ld", counttwo);

		Sleep(100);
		system("CLS");
	}
}


/*|homeRobot|------------------------------------------------------------------
#
# Decription:
#	This function will autohome the robot.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void homeRobot(void) {

}


/*|moveJoint|------------------------------------------------------------------
#
# Decription:
#	This function prompts the user for a Joint 1 and Joint 2 position, then moves there
#
# ---------------------------------------------------------------------------*/
void moveJoint(void) {
	double joint1, joint2, x, y;

	printf("Please enter the joint angles you would like to move to (theta1,theta2)");
	scanf("%lf,%lf", &joint1, &joint2);

	setScaraPosition(
		joint1*JOINT1_RESOLUTION, 
		joint2*JOINT2_RESOLUTION, 
		JOINT1_MAX_SPEED, 
		JOINT2_MAX_SPEED
	);
	forwardKinematics(joint1, joint2, &x, &y);

	printf("Moving to X:%lf , Y:%lf\n", x, y);
}


/*|moveTCP|--------------------------------------------------------------------
#
# Decription:
#	This function prompts the user for a specific x,y coordinate and left or 
# right arm configuration.
#
# ---------------------------------------------------------------------------*/
void moveTCP(void) {
	double joint1 = 0;
	double joint2 = 0;
	double x = 0;
	double y = 0;
	char armSolution = 0;

	printf("Please enter the joint angles you would like to move to (X,Y,armSolution)\
		   \n(0 or 1 for arm solution, being left or right)");
	scanf("%lf,%lf, %c", &x, &y, &armSolution);

	inverseKinematics(x, y, &joint1, &joint2, armSolution);
	setScaraPosition(
		joint1*JOINT1_RESOLUTION, 
		joint2*JOINT2_RESOLUTION, 
		JOINT1_MAX_SPEED, 
		JOINT2_MAX_SPEED
	);

	printf("Moving to X:%lf , Y:%lf\n", x, y);	
}


/*|limpUnlimp|-----------------------------------------------------------------
#
# Decription:
#	This function prompts the user to make the robot:
#		- limp (able to move by hand) or 
#		- unlimp (moves to a set position constantly).
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void limpUnlimp(void) {
//Mine
}


/*|moveRobot|------------------------------------------------------------------
#
# Decription:
#	This function performs a PID feedback control loop. The setpoint is set by
# the motor0SetAngle and the motor1SetAngle in the robot variable. The feedback
# is received from the encoder0 and encoder1 in the robot variable. The output
# will drive the speed of both motors. For proper functionality, this function
# should be called on a set period.
#
# Note: Values for the PID constants and the gain were found experimentally.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void moveRobot(void) {

}


/*|forwardKinematics|-----------------------------------------------------------
#
# Decription:
#	This function calculates the x,y coordinates for given joint angles.
#
# Inputs:
#	joint1 - Joint 1 angle in degrees.
#	joint2 - Joint 2 angle in degrees.
#
# Outputs:
#	x - x-coordinate in inches.
#	y - y-coordiante in inches.
#
# ---------------------------------------------------------------------------*/
void forwardKinematics(double joint1, double joint2, double* x, double* y) {
	double joint1_Radians = (joint1*180)/M_PI;
	double joint2_Radians = (joint2*180)/M_PI;
	if (((fabs(joint1)) <= JOINT1_MAX) && ((fabs(joint2)) <= JOINT2_MAX)){
		*x = JOINT1_LENGTH * cos(joint1_Radians) + JOINT2_LENGTH * cos(joint1_Radians + joint2_Radians);
		*y = JOINT1_LENGTH * sin (joint1_Radians) + JOINT2_LENGTH * cos(joint1_Radians + joint2_Radians);
	}
	else{
		*x = 0;
		*y = 0;
		printf("SOMETHING WENT VERY WRONG (forwardKinematics function)\n");
	}
}


/*|inverseKinematics|----------------------------------------------------------
#
# Decription:
#	This function calcuates the joint angles for a given x,y coordinates.
#
# Inputs:
#	x - x-coordinate in inches.
#	y - y-coordinate in inches.
#	arm - LEFT or RIGHT arm configuration.
#
# Outputs:
#	joint1 - Joint 1 angle in degrees.
#	joint2 - Joint 2 angle in degrees.
#
# ---------------------------------------------------------------------------*/
void inverseKinematics(double x, double y, double* joint1, double* joint2, char arm) {
 
	double L = sqrt((x*x) + (y*y));
	double beta = atan2(y,x);
	double alpha = acos(((JOINT2_LENGTH*JOINT2_LENGTH)-(L*L)-JOINT1_LENGTH*JOINT1_LENGTH)/(-2*L*JOINT1_LENGTH));
 
	if(arm == LEFT){
	   *joint1 = beta + alpha;
	}
	else if (arm == RIGHT){
	   *joint1 = beta - alpha;
	}
	else{
	   *joint1 = 1000; //to make it out of range cuz arm solution isnt valid
	}

	if(*joint1 < -M_PI){
		*joint1+= 2*M_PI;
	 }
	 else if (*joint1 > M_PI){
		*joint1-= 2*M_PI;
	 }

	*joint2 = atan2((y-(JOINT1_LENGTH*sin(*joint1))),(x - (JOINT1_LENGTH * cos(*joint1))))-(*joint1);
 
	if(*joint2 < -M_PI){
		*joint2+= 2*M_PI;
	 }
	 else if (*joint2 > M_PI){
		*joint2-= 2*M_PI;
	 }

	*joint1 = (*joint1*180)/M_PI;
	*joint2 = (*joint2*180)/M_PI;
 
	if ((fabs(*joint1) >= JOINT1_MAX) || (fabs(*joint2) >= JOINT2_MAX)){
		*joint1 = 0;
		*joint2 = 0;
		printf("SOMETHING WENT VERY WRONG (inverseKinematics function)\n");
	}
}

/*|udpate|---------------------------------------------------------------------
#
# Decription:
#	This function is executed every 100 ms. The encoder counts stored in robot 
# are updated. Motors will be activated or deactivated based on the limp 
# variable. If the motors are active, moveRobot is called which calculates the
# required motor speed based on the desired angle.
#
# ---------------------------------------------------------------------------*/
void update(void) {
	// Update the Encoder Counts
	getScaraEncoderCount();
}
