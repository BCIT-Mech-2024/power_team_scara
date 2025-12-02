/*|SCARA Controller|-----------------------------------------------------------
#
# Project: ROBT 3341 - SCARA Control
# Program: scara_control.h
#
# Description:
#	This program contains Peter, Chris & Tavin's solution to ROBT 3341 - SCARA.
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
# Authors: Peter, Tavin, Chris
# Last Modified: December 02, 2025
# -----------------------------------------------------------------------------*/

/*|HOW TO TEST THIS PROGAM|------------------------------------------------------
# 1: run & debug to make sure program runs
# 2: 
# -----------------------------------------------------------------------------*/


#ifndef SCARA_CONTROL_H_
#define SCARA_CONTROL_H_

/*|Includes|-------------------------------------------------------------------*/
#include "scara_interface.h"
#include <conio.h>		// _kbhit
#include <windows.h>	// system, Sleep
#include <stdio.h>
#include <math.h>

/*|Constants|------------------------------------------------------------------*/
#define M_PI    3.14159265358979323846
#define LEFT 	1
#define RIGHT 	0

#define JOINT1_RESOLUTION	800.0			// Pulses/Degree
#define JOINT2_RESOLUTION	800.0			// Pulses/Degree

#define JOINT1_LENGTH		10.0			// Inches
#define JOINT2_LENGTH		10.0			// Inches

#define JOINT1_MAX			95				// Degrees (from Horizontal)
#define JOINT2_MAX			150				// Degrees (from Joint 1)

#define JOINT1_MAX_SPEED 	800				// pulses/sec
#define JOINT2_MAX_SPEED 	800				// pulses/sec


/*|Enumerations|---------------------------------------------------------------*/
typedef enum {
	HOME = 1, DISPLAY, JOINTS, TOOL, LIMP, RESET, QUIT
} menu;

/*|Function Declarations|------------------------------------------------------*/
void scaraMenu(void);
void displayPosition(void);
void homeRobot(void);
void moveJoint(void);
void moveTCP(void);
void limpUnlimp(void);
void moveRobot(void);

void forwardKinematics(double joint1, double joint2, double* x, double* y);
void inverseKinematics(double x, double y, double* joint1, double* joint2, char arm);

void update(void);

#endif