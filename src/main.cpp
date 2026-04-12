#include "main.h"
#include "globals.hpp"
#include "lib/chassis.hpp"
#include "lib/pid.hpp"
#include "pros/misc.h"
#include "autonomous.hpp"
#include "pros/motors.h"
#include <string>
using namespace std;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// parkMech.toggle();
	// initializeScreen();
	// chassis.reset();
	// chassis.setInputScale(Chassis::SINSQUARED);
	wingMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// matchAuto();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	bool parkMechPossible = true;
	
	
	while (true) {
		int leftY = controller.get_analog(ANALOG_LEFT_Y);
		int rightX = controller.get_analog(ANALOG_RIGHT_X);

		chassis.arcade(leftY,rightX);
		//controller.set_text(0,0, (chassis.getPose().to_string()));
		//controller.set_text(0,0, std::to_string((chassis.getPose().radToDeg(chassis.getPose().getTheta()))));
		controller.set_text(0,0,to_string(wingMech.get_current_draw()));

		//intakePeriodic();

		if (controller.get_digital_new_press(DIGITAL_L1)) {
			wingMech.move(-60);
		}
		else if (controller.get_digital_new_press(DIGITAL_L2)) {
			wingMech.move(60);
		}
		else if (controller.get_digital_new_release(DIGITAL_L1) || controller.get_digital_new_release(DIGITAL_L2)) {
			wingMech.move(0);
		}

		if (controller.get_digital_new_press(DIGITAL_DOWN)){
			wingMech.move(127);
		}
		else if (controller.get_digital_new_press(DIGITAL_UP)){
			wingMech.move(-127);
		}
		else if (controller.get_digital_new_release(DIGITAL_DOWN) || controller.get_digital_new_release(DIGITAL_UP)){
			wingMech.move(0);
		}

		//Remove if wingmech is stopping early
		if (wingMech.get_current_draw() > 2500) {
			wingMech.move(0);
		}

		
		// if (controller.get_digital_new_press(DIGITAL_X)) {
		// 	intakeRBWheel.move(127);
		// 	intakeTopFront.move(127);
		// 	intakeMain.move(127);
		// }
		// else if (controller.get_digital_new_press(DIGITAL_B)) {
		// 	intakeRBWheel.move(-67);
		// 	intakeTopFront.move(-67);
		// 	intakeMain.move(-67);
		// }
		// else if (controller.get_digital_new_release(DIGITAL_X) || controller.get_digital_new_release(DIGITAL_B)) {
		// 	intakeRBWheel.move(0);
		// 	intakeTopFront.move(0);
		// 	intakeMain.move(0);
		// }

		//Intake
		if (controller.get_digital_new_press(DIGITAL_R1)){
			intakeRBWheel.move(-127);
			intakeTopFront.move(-127);
			intakeMain.move(-127);
		}
		//Outtake
		else if (controller.get_digital_new_press(DIGITAL_R2)) {
			intakeRBWheel.move(127);
			intakeTopFront.move(127);
			intakeMain.move(127);
		}
		//Intake without scoring
		else if (controller.get_digital_new_press(DIGITAL_X)) {
			intakeMain.move(-127);
			intakeTopFront.move(0);
			intakeRBWheel.move(0);
		}
		else if (controller.get_digital_new_release(DIGITAL_R1) 
		|| controller.get_digital_new_release(DIGITAL_R2)
		|| controller.get_digital_new_release(DIGITAL_X)) {
			intakeRBWheel.move(0);
			intakeTopFront.move(0);
			intakeMain.move(0);
		}

		pros::delay(20);
	}
}