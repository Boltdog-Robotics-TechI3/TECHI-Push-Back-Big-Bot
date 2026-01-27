#pragma once

// Libraries
#include "api.h"
#include "lib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
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
inline pros::MotorGroup topScore({-16, -17});
//inline pros::Motor lift(3); 
//inline pros::Motor low_sorter(4); 
//inline pros::Motor ejector(9); 
//inline pros::Motor high_sorter(10); 
inline pros::adi::Pneumatics leftDescore((uint8_t)'A', true);
//inline pros::adi::Pneumatics rightDescore((uint8_t)'B', true);
inline pros::adi::Pneumatics parkMech((uint8_t)'B', true);
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
