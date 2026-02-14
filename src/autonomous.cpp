
#include "globals.hpp"
#include "lib/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "util/pose.hpp"
#include <numbers>

void hump(double ms) {
    leftMotors.move(-127);
    rightMotors.move(-127);
    pros::delay(ms/6);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(ms/3);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(ms/2);
}

void matchAuto() {
	chassis.moveToPose(Pose(0, 24, 0), 1500, 50);
}