
#include "globals.hpp"
#include "lib/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "util/pose.hpp"
#include <numbers>
#include "pros/misc.h"

void hump(double ms) {
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(ms/6);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(ms/3);
    leftMotors.move(-127);
    rightMotors.move(-127);
    pros::delay(ms/2);
}

void loaderDown(){
    loaderMech.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    loaderMech.move(-50);
    pros::delay(1500);
    loaderMech.move(0);
}

void loaderUp(){
    loaderMech.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    loaderMech.move(50);
    pros::delay(1500);
    loaderMech.move(0);
}

void matchAuto() {
	//chassis.moveToPose(Pose(0,-32,0),1500,50);
    //chassis.turnToAngle(90,1500);
    /*intake.move(127);
    chassis.moveToPose(Pose(-8,-32,0),1500,50);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    hump(300);
    
    chassis.moveToPose(Pose(-20,-32,0),1500,70);
    middleScore.move(-127);
    topScore.move(100);
    pros::delay(1500);
    //intake.move(0);
    middleScore.move(0);
    topScore.move(0); 
    chassis.moveToPose(Pose(0,-32,0),1500,50);
    chassis.turnToAngle(0,1500);
    chassis.moveToPose(Pose(0,-40,0),1500,50);
    chassis.turnToAngle(90,1500);
    chassis.moveToPose(Pose(-20 ,-45,0),1500,50); */

    /*chassis.moveDistance(45, 1500);
	chassis.turnToAngle(90,1500);
	//chassis.moveDistance(-35,2000);
    topScore.move(100);
    middleScore.move(127);
    pros::delay(1500);
    topScore.move(0);
    middleScore.move(0);*/

}