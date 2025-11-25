/*|ROBT 3341: SCARA Interface|-------------------------------------------------
#
# Project: ROBT 3341 - SCARA Control
# Program: scara_interface.c
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

#include "scara_interface.h"

/*|Global Variables|-----------------------------------------------------------*/
struct BRobot robot;


/*|initializeSCARA|------------------------------------------------------------
#
# Decription:
#	This function will initialize the SCARA by establishing a connection with
# the Galil Motion Controller.
#
# Note: The address of the controller may change unexpectedly. The correct
# address can be found using GalilTools. As a last resort, the USB connection
# can also be used.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
int initializeSCARA(GCStringIn address) {

	GCStringIn brushedSetup = "BR 1,1";
	GCStringIn abortEnable = "OE 1,1";
	GCStringIn startHereCmd = "SH";

	//Assumes an IP address for the galil.
	GAssign(address, "00:50:4C:38:7C:22");

	//open a connection to the galil controller
	printf("Connecting to %s...\n", address);
	GOpen(address, &robot.connection);

	//checks to see if globalGVariable for filled, and only runs if it did so.
	if (robot.connection == 0) {
		printf("...connection failed.\n");
		printf("Check to make sure that Galil is powered on and connected.\n");
		printf("If using ethernet, ensure that LINK/ACT light is either on or blinking (should take a 5-10 seconds).\n");

		return 0;

	} else {
		printf("...connection established, running code.\n");

		//issue brushed servo motor setup
		GCmd(robot.connection, brushedSetup);

		//issue command to enable the abort input
		GCmd(robot.connection, abortEnable);

		//issue a start here command
		GCmd(robot.connection, startHereCmd);

		//start recording stream
		GRecordRate(robot.connection, RECORD_RATE);

		return 1;
	}
}


/*|setScaraSpeed|--------------------------------------------------------------
#
# Decription:
#	This function uses the JG command to set the speed of the motors.	
#
# Inputs:
#	motor0Speed - Motor 0 Speed in counts/s.
#	motor1Speed - Motor 1 Speed in counts/s.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void setScaraSpeed(int motor0Speed, int motor1Speed) {
	char command[G_SMALL_BUFFER] = "";
	//This function assumes motor0 is axis A, and motor1 is axis B. 

	// TO DO: Validate Speeds
	robot.motor0Speed = motor0Speed;
	robot.motor1Speed = motor1Speed;

	// Send the Command to Galil Controller
	sprintf_s(command, G_SMALL_BUFFER, "JG %d,%d", robot.motor0Speed, robot.motor1Speed);
	GCmd(robot.connection, command);
	GCmd(robot.connection, "BG XY");

}


/*|getScaraEncoderCount|-------------------------------------------------------
#
# Decription:
#	This function retrieves the encoder counts from the Galil Motion 
# Controller. The values are saved in the global robot variable.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void getScaraEncoderCount(void) {

	//GDataRecord struct to aquire the relevant data
	union GDataRecord dataRecord;

	//use Grecord to populate Data
	GRecord(robot.connection, &dataRecord, G_QR);

	robot.encoder0 = dataRecord.dmc4000.axis_a_motor_position;
	robot.encoder1 = dataRecord.dmc4000.axis_b_motor_position;
}


/*|findScaraIndex|-------------------------------------------------------------
#
# Decription:
#	This function uses the FI command to find the index pulse for each motor.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void findScaraIndex(void) {

	//variables
	unsigned char command[G_SMALL_BUFFER] = "";
	union GDataRecord dataRecord = { dataRecord.dmc4000.axis_a_status = 0, dataRecord.dmc4000.axis_b_status = 0 };

	//issue stop command
	GCmd(robot.connection, "ST");
	
	sprintf_s(command, G_SMALL_BUFFER, "JG %d,%d", INDEX_SPEED_FAST, INDEX_SPEED_FAST);
	GCmd(robot.connection, command);

	robot.motor0Speed = INDEX_SPEED_FAST;
	robot.motor1Speed = INDEX_SPEED_FAST;

	sprintf_s(command, G_SMALL_BUFFER, "HV %d,%d", INDEX_SPEED_SLOW, INDEX_SPEED_SLOW);
	GCmd(robot.connection, command);

	GCmd(robot.connection, "FI AB");
	GCmd(robot.connection, "BG AB");

	//Wait for it to stop moving
	do {

		GRecord(robot.connection, &dataRecord, G_QR);

	} while (((dataRecord.dmc4000.axis_a_status & MOVE_BIT) | (dataRecord.dmc4000.axis_b_status & MOVE_BIT)));

	//issue SH command
	sprintf_s(command, G_SMALL_BUFFER, "SH");
	GCmd(robot.connection, command);
	
}


/*|setScaraPosition|-----------------------------------------------------------
#
# Decription:
#	This function uses the PA command to send the SCARA to a new setpoint.
#
# Inputs:
#	m0Pos - Motor 0 Position in counts.
#	m1Pos - Motor 1 Position in counts.
#	m0Speed - Motor 0 Speed in counts/s.
#	m1Speed - Motor 1 Speed in counts/s.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void setScaraPosition(int m0Pos, int m1Pos, int m0Speed, int m1Speed) {

	unsigned char command[G_SMALL_BUFFER] = "";

	union GDataRecord dataRecord;
	
	//motor speeds that where used in the previous function call
	static int m0SpeedPrev = 0;
	static int m1SpeedPrev = 0;

	//the new speeds that will be set
	int usableM0Speed = m0Speed;
	int usableM1Speed = m1Speed;

	//done bits
	int m0Done = 0;
	int m1Done = 0;

	//to make sure that the motor isn't moving when the code begins to be executed, sends stop, then start command
	GCmd(robot.connection, "ST");

	GRecord(robot.connection, &dataRecord, G_QR);
	GCmd(robot.connection, "SH");
	GRecord(robot.connection, &dataRecord, G_QR);

	//if the old speed is different from the new speed, update the speed
	if ((robot.motor0Speed != m0Speed) | (robot.motor1Speed != m1Speed)) {
		robot.motor0Speed = m0Speed;
		robot.motor1Speed = m1Speed;

		sprintf_s(command, G_SMALL_BUFFER, "SP %d,%d", m0Speed, m1Speed);
		GCmd(robot.connection, command);
	}

	//command the motor to move
	sprintf_s(command, G_SMALL_BUFFER, "PA %d,%d", m0Pos, m1Pos);
	GCmd(robot.connection, command);

	sprintf_s(command, G_SMALL_BUFFER, "BG XY");
	GCmd(robot.connection, command);

	//wait for it to arrive
	do {

		GRecord(robot.connection, &dataRecord, G_QR);

	} while (((dataRecord.dmc4000.axis_a_status & MOVE_BIT) | (dataRecord.dmc4000.axis_b_status & MOVE_BIT)));



}


/*|defineScaraPosition|--------------------------------------------------------
#
# Decription:
#	This function uses the DP command to overwrite the encoder counts of the
# motors.
#
# Inputs:
#	motor0count - Motor 1 Position in counts.
#	motor1count - Motor 2 Position in counts.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void defineScaraPosition(int motor0count, int motor1count) {
	unsigned char command[G_SMALL_BUFFER] = "";

	sprintf_s(command, G_SMALL_BUFFER, "DP %d, %d", motor0count, motor1count);
	GCmd(robot.connection, command);
}


/*|enableScaraMotors|----------------------------------------------------------
#
# Decription:
#	This function enables both motors.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void enableScaraMotors(void) {

	GCmd(robot.connection, "SH XY");	// Servo Here

}


/*|disableScaraMotors|---------------------------------------------------------
#
# Decription:
#	This function disables both motors.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void disableScaraMotors(void) {

	GCmd(robot.connection, "ST XY");	// Stop
	GCmd(robot.connection, "AB 1,1");	// Abort
	GCmd(robot.connection, "MO");		// Motors Off

}


/*|powerDownScara|-------------------------------------------------------------
#
# Decription:
#	This function powers down and disables both motors then closes the 
# connection to the Galil Motion Controller.
#
# Last Modified: September 24, 2025
# ---------------------------------------------------------------------------*/
void powerDownScara(void) {
	//Set the motors to stop
	disableScaraMotors();

	//anything else that needs to happen to safely stop the SCARA goes here.

	//close communications
	GClose(robot.connection);
	
}