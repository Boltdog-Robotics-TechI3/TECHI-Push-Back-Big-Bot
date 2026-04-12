#pragma once

// Libraries
#include "api.h"
#include "lib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"
#include <cstdint>

// Bot measurements
inline double wheel_diameter = 2.75;
inline double track_width = 12;
inline double gear_ratio = 1;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
inline pros::MotorGroup rightMotors({3, 4});
inline pros::MotorGroup leftMotors({-1, -2});
inline pros::MotorGroup intakeRBWheel({7}); //Rubber Band Wheel
inline pros::MotorGroup intakeTopFront({9});
inline pros::MotorGroup intakeMain({5});
inline pros::MotorGroup wingMech({12});
// inline pros::Motor middleScore( -18);
// inline pros::Motor motor19(19);
// inline pros::Motor motor11(11);
// inline pros::Motor motor20(20);
// inline pros::Motor topScore( -16);
// inline pros::Motor lowerTopScore(-17);
// inline pros::adi::Pneumatics leftDescore((uint8_t)'A', true);
// inline pros::adi::Pneumatics parkMech((uint8_t)'B', true);

// Chassis PID Controllers
inline PIDController lateral(6, 0.0001 ,0.1); 
inline PIDController turn(90, 0.0075, 0.1);
inline PIDController align(45, 0, 0);

// Drivetrain
inline TankDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
inline pros::IMU imu(16);
inline TrackingWheel horizontalTrackingWheel(3, 3.25, 0.25, WheelPosition::HORIZONTAL);
inline TrackingWheel verticalTrackingWheel(-2, 3.25, -1.0, WheelPosition::VERTICAL);

// Odometry
inline OdomSensors odometry(&verticalTrackingWheel, &horizontalTrackingWheel, &imu);

// Chassis
inline TankChassis chassis(&drivetrain, &odometry, &lateral, &turn, &align);

// Comp Specifications
inline bool skills = false;
inline bool match = true;
inline bool elim = false;
inline bool qual = true;
inline bool redAlliance = true;
inline bool blueAlliance = false;
inline int autoSelection = 0; 
