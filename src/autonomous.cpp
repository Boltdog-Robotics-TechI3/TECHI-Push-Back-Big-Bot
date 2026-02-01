
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
	chassis.startTracking();
    chassis.moveDistance(26,3000);
    //intake
    chassis.turnAngle(90);
    chassis.moveDistance(13,2000);
    intakeOnAll();
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    pros::delay(100);
    intakeOffAll();
    chassis.moveDistance(-6, 3000);
    chassis.turnAngle(45);
    outtake();
    pros::delay(600);
    intakeOffAll();

    //Score
    chassis.turnAngle(270);
    intakeOnAll();
    chassis.moveDistance(12,2000);
    intakeOffAll();
    chassis.turnAngle(280);
    intakeTopScore();
    pros::delay(2500);
    // hump(150);
    // hump(150);
    // hump(150);
    // pros::delay(1250);
    chassis.turnAngle(270);
    intakeOffAll();
    chassis.moveDistance(-6);
    chassis.turnAngle(90);
    intakeOnAll();
    chassis.moveDistance(18, 2000);

    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);

    // //dump off
    // chassis.moveDistance(-6);
    // chassis.turnAngle(45);
    // intakeTopScore();
    // pros::delay(5000);
    // intakeOffAll();
    // chassis.turnAngle(90);
    // intakeOnAll();
    // chassis.moveDistance(8,1500);
    
    // hump(300);
    // hump(300);
    // hump(300);
    // hump(300);
    // hump(300);
    // hump(300);
    // hump(300);

    chassis.moveDistance(-12, 3000);
    chassis.turnAngle(270);
    intakeOffAll();
    chassis.moveDistance(7,1000);
    chassis.turnAngle(285);
    intakeTopScore();
    pros::delay(4000);
    intakeOffAll();
    chassis.turnAngle(270);
}