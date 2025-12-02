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


/*|Global Variables|-----------------------------------------------------------*/
extern struct BRobot robot;


/*|scaraMenu|------------------------------------------------------------------
#
# Decription:
#	This function contains the SCARA menu program.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void scaraMenu(void) {

}


/*|displayPosition|------------------------------------------------------------
#
# Decription:
#	This function displays the encoder counts, joint angles, and coordinates.
#
# Last Modified: September 30, 2025
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

// Need to move to 0,0

setScaraPosition(0,0,800,800);

}


/*|moveJoint|------------------------------------------------------------------
#
# Decription:
#	This function prompts the user for a Joint 1 and Joint 2 position.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void moveJoint(void) {

}


/*|moveTCP|--------------------------------------------------------------------
#
# Decription:
#	This function prompts the user for a specific x,y coordinate and left or 
# right arm configuration.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void moveTCP(void) {

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

int limp, J1, J2;

printf("Enter a 1 to put the Scara into Limp Mode. Or any other key to proceed: \n");
scanf("%d" &limp);

if(limp)
{
	disableScaraMotors();
	return;
}
else
{
	moveJoint();
	return;
}

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


/*|setScaraPosition|-----------------------------------------------------------
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
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void forwardKinematics(double joint1, double joint2, double* x, double* y) {

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
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void inverseKinematics(double x, double y, double* joint1, double* joint2, char arm) {

}


/*|udpate|---------------------------------------------------------------------
#
# Decription:
#	This function is executed every 100 ms. The encoder counts stored in robot 
# are updated. Motors will be activated or deactivated based on the limp 
# variable. If the motors are active, moveRobot is called which calculates the
# required motor speed based on the desired angle.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void update(void) {
	// Update the Encoder Counts
	getScaraEncoderCount();
}