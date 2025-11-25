/*|SCARA Controller|-----------------------------------------------------------
#
# Project: ROBT 3341 - SCARA Control
# Program: scara_main.c
#
# Description:
#	This program controls the SCARA Robot which was a repurposed CRS 5-DoF
# Robot arm. By default, the program should connect to the SCARA.
#
# This program contains code developed by Bryn Rissling and modified by
# Isaiah Regacho.
#
# Author: <Your Name>
# Date Created: March 22, 2025 by Bryn Rissling
#
# History:
#	v2025-03-22 (BR): Created for ROBT 4491
#	v2025-06-17 (IR): Modified for ROBT 3341
#
# Last Modified: <Today>
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>
#include "scara_interface.h"
#include "scara_control.h"
#include "tic_generator.h"


/*|Global Variables|-----------------------------------------------------------*/
extern struct BRobot robot;


/*|Function Declarations|------------------------------------------------------*/
void scaraTestProcedure(void);


void main(void){
	// Customize Output
	system("COLOR 0A");
	system("CLS");

	robot.limp = 1;
	robot.motorsActive = 1;

	// Initializes SCARA Connection
	// 
	// Using Ethernet:
	//		Use GalilTools to find the assigned address of the SCARA.
	//		Update the initialization function call to "initializeSCARA("NEW IP ADDRESS").
	// 
	// Alternative Connection USB:
	//		Go to Device Manager > Ports (COM & LPT)
	//		Look for a device call "USB Serial Port".
	//		Changed the initialization function call from "initializeSCARA("142.232.112.183")" to
	//		"initializeSCARA("COM4")"
	if (!initializeSCARA("141.232.112.209")) {
		exit(0);
	}

	// Use tic to repeatedly call a function.
	tic_setup(update, 100000);		//setup a function to execute every 100ms.
	tic_enable(1);					//start the timer. The function called "update" will be called every 100ms

	disableScaraMotors();
	displayPosition();
	
	scaraTestProcedure(); // Remove this line if not testing SCARA operation.
	
	// Full Program
	// scaraMenu();
	
	tic_enable(0);	// Stop the timer
	powerDownScara();
}

/******************************************************************************
* Function: scaraTestProcedure
* 
*	This function will validate the operation of the following functions:
*	- findScaraIndex
*	- setScaraPosition
*	- defineScaraPosition
*	- setScaraSpeed
*	- getScaraEncoderCount
* 
* Last Modified: September 08, 2025 by Isaiah Regacho
******************************************************************************/
void scaraTestProcedure(void) {
	// Move to Index
	printf("\nSearching for indexes...\n");
	findScaraIndex();
	printf("...index found.\n");
	printf("Press ENTER to continue...\n");
	getchar();

	// Move SCARA based on Index offset from 0 degrees.
	setScaraPosition(-1354, -1040, 20000, 20000);
	printf("\nReseting Encoder Counts to 0, 0...\n");
	defineScaraPosition(0, 0);

	// Reset both encoders to 0
	getScaraEncoderCount();
	printf("Encoder 0: %d\n", robot.encoder0);
	printf("Encoder 1: %d\n", robot.encoder1);
	printf("Press ENTER to continue...\n");
	getchar();

	// Demonstrate Speed Control
	printf("\nTesting Velocity Control...\n");
	setScaraSpeed(1000, 1000);

	while (robot.encoder0 < 5000 && robot.encoder1 < 5000) {
		getScaraEncoderCount();
		printf("Encoder 0: %d\n", robot.encoder0);
		printf("Encoder 1: %d\n", robot.encoder1);
	}

	printf("...motion successful.\n");
	printf("Press ENTER to continue...\n");
	setScaraSpeed(0, 0);
	getchar();

	// Demonstrate Position Control
	printf("\nTesting Position Control...\n");
	setScaraPosition(-4000, -4000, 20000, 20000);
	printf("...motion succesful.\n");
	printf("Press ENTER to continue...\n");
	getchar();

	// End of Test Procedure
	printf("\nBeginning SCARA shutdown sequenece...\n");
	printf("Press ENTER to continue...\n");
	getchar();
}

