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
inline double wheel_diameter = 3.25;
inline double track_width = 10.75;
inline double gear_ratio = 3.0/4.0;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
inline pros::MotorGroup rightMotors({8, -9, 7, 10});
inline pros::MotorGroup leftMotors({-4, -2, 3, -1});
inline pros::MotorGroup intake({-14,15});
inline pros::Motor middleScore( -18);
inline pros::Motor motor19(19);
inline pros::Motor motor11(11);
inline pros::Motor motor20(20);
inline pros::Motor topScore( -16);
inline pros::Motor lowerTopScore(-17);
inline pros::adi::Pneumatics leftDescore((uint8_t)'A', true);
inline pros::adi::Pneumatics parkMech((uint8_t)'B', true);

//Color Sensor
inline pros::Optical colorSensor(21);

// Chassis PID Controllers
inline PIDController lateral(8, 0, 0.1); 
inline PIDController turn(60, 0.2, 3);
inline PIDController align(30, 0, 0);

// Drivetrain
inline DifferentialDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
inline pros::IMU imu(6);
inline TrackingWheel horizontalTrackingWheel(-12, 2.08, -6, WheelPosition::BACK);
inline TrackingWheel verticalTrackingWheel(-13, 2.08, 0.75, WheelPosition::LEFT);

// Odometry
inline Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

// Chassis
inline DifferentialChassis chassis(&drivetrain, &odometry, &lateral, &turn, &align);


// Comp Specifications
inline bool skills = false;
inline bool match = true;
inline bool elim = false;
inline bool qual = true;
inline bool redAlliance = true;
inline bool blueAlliance = false;
inline int autoSelection = 0; 
