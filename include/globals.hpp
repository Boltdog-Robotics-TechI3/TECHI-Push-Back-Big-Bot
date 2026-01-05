#pragma once

// Libraries
#include "api.h"
#include "lib/api.hpp"

// Bot measurements
inline double wheel_diameter = 3.25;
inline double track_width = 10.75;
inline double gear_ratio = 3.0/4.0;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
inline pros::MotorGroup rightMotors({18, 19, 20});
inline pros::MotorGroup leftMotors({-11, -12, -13});
inline pros::MotorGroup intake({-1, 2, 8});
inline pros::Motor lift(3); 
inline pros::Motor low_sorter(4); 
inline pros::Motor ejector(9); 
inline pros::Motor high_sorter(10); 
// Drivetrain
inline DifferentialDrivetrain drivetrain(&leftMotors, &rightMotors, wheel_diameter, track_width, gear_ratio);

// Tracking Wheel
inline pros::IMU imu(7);
inline TrackingWheel horizontalTrackingWheel(5, 2.08, 0, WheelPosition::BACK);
inline TrackingWheel verticalTrackingWheel(-4, 2.08, 0.25, WheelPosition::LEFT);

// Odometry
inline Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &imu);

// Chassis
inline DifferentialChassis chassis(&drivetrain, &odometry);
