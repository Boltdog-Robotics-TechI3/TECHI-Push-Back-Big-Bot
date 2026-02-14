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
inline pros::MotorGroup rightMotors({18, 19, -20, 17});
inline pros::MotorGroup leftMotors({-12, -14, 13, -15});
inline pros::MotorGroup intake({-9, 8});
inline pros::Motor middleScore(-10);
inline pros::Motor topScore(-7);
inline pros::Motor loaderMech(5);

// Pneumatics
inline pros::adi::Pneumatics leftDescore((uint8_t)'A', true);
inline pros::adi::Pneumatics parkMech((uint8_t)'B', true);

// Chassis PID Controllers
inline PIDController lateral(8, 0, 0.1); 
inline PIDController turn(60, 0.2, 3);
inline PIDController align(30, 0, 0);

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
