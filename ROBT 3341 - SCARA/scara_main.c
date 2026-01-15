/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>
#include "scara_interface.h"
#include "scara_control.h"
#include "scara_menu.h"
#include "tic_generator.h"
#include <string.h>


/*|Global Variables|-----------------------------------------------------------*/
extern struct BRobot robot;


/*|Function Declarations|------------------------------------------------------*/
void scaraTestProcedure(void);


void main(void){
	// Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. 
	// Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. 
	// Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. 
	// Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.
	system("COLOR 0A");
	system("CLS");

	robot.limp = 1;
	robot.motorsActive = 1;
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
	scaraMenu();
	
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

