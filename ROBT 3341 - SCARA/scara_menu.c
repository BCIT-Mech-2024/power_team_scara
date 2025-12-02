#include "scara_menu.h"
#include <stdbool.h>

extern struct BRobot robot;

MenuOption readInput();
void displayMenu();

/*|scaraMenu|------------------------------------------------------------------
#
# Decription:
#	This function contains the SCARA menu program.
#
# Last Modified: September 30, 2025
# ---------------------------------------------------------------------------*/
void scaraMenu(void) {
	while(true) {
		displayMenu();
		switch (readInput()) {
			case Home:
				system("CLS");
				printf("\tHoming ... ");
				homeRobot();
				break;

			case Display:
				system("CLS");
				displayPosition();
				break;

			case Joints:
				moveJoint();
				break;

			case Tool:
				moveTCP();
				break;

			case Limp:
				system("CLS");
				printf("Include message about limp/unlimp\n");
				limpUnlimp();
				break;

			case Reset:
				system("CLS");
				printf("Resetting motor\n");
				disableScaraMotors();
				robot.encoder0 = 0;
				robot.encoder1 = 0;
				break;

			case Quit:
				system("CLS");
				printf("Exiting\n");
				return;
				break;
		}
	}
}

MenuOption readInput() {
	while (true) {
		if (_kbhit()) {
			char key = getch();
			switch (key) {
				case "1":
					return Home;
				case "2":
					return Display;
				case "3":
					return Joints;
				case "4":
					return Tool;
				case "5":
					return Limp;
				case "6":
					return Reset;
				case "7":
					return Quit;
			}
		}
	}
}

void displayMenu() {
	printf(
"  -- Menu --\n
\t 1:\t Home\n
\t 2:\t Display Position\n
\t 3:\t Move Joint\n
\t 4:\t Move Tool\n
\t 5:\t Limp/Unlimp\n
\t 6:\t Reset\n
\t 7:\t Quit\n
");
}

