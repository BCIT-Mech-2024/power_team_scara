/*|ROBT 3341: SCARA Interface|-------------------------------------------------
#
# Project: ROBT 3341 - SCARA Control
# Program: scara_interface.h
#
# Description:
#   This module provides an interface to the SCARA.
#
# Other Information:
#	Axis 0-first arm segment (Shoulder)
#	Axis 1-second arm segment (Elbow)
#
# This program contains code developed by Bryn Rissling and modified by
# Isaiah Regacho for ROBT 3341.
#
# History:
#	v2025-03-22 (BR): Created for ROBT 4491 Capstone Project.
#	v2025-09-30 (IR): Modified for ROBT 3341.
#
# Last Modified: September 30, 2025
# -----------------------------------------------------------------------------*/

// #pragma once
#ifndef SCARA_INTERFACE_H_
#define SCARA_INTERFACE_H_

/*|Includes|-------------------------------------------------------------------*/
#include "stdio.h"	// printf, scanf_s, getchar, sprintf_s
#include "gclibo.h"	// Open Source Galil C Library

/*|Constants|------------------------------------------------------------------*/
#define RECORD_RATE 9600
#define MOVE_BIT 0x8000

#define INDEX_SPEED_FAST 10000
#define INDEX_SPEED_SLOW 500

/*|Structures|-----------------------------------------------------------------*/
struct BRobot {
	GCon connection;
	long encoder0;
	long encoder1;
	int motor0Speed;
	int motor1Speed;
	double motor0SetAngle;
	double motor1SetAngle;
	char limp;
	char motorsActive;
};

/*|Function Declarations|------------------------------------------------------*/
int initializeSCARA(GCStringIn address);
void setScaraSpeed(int motor0Speed, int motor1Speed);
void getScaraEncoderCount(void);
void findScaraIndex(void);
void setScaraPosition(int m0Pos, int m1Pos, int m0Speed, int m1Speed);
void defineScaraPosition(int motor0count, int motor1count);
void enableScaraMotors(void);
void disableScaraMotors(void);
void powerDownScara(void);


#endif /* SCARA_INTERFACE_H_ */