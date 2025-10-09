#pragma once

#include "main.h"

#include "lemlib/api.hpp"

namespace config {
    // Upload info
    inline const std::string version = "Version 1.0.0";
    inline const std::string upload_message = "dt";

    // Drivetrain
    inline const double dt_track_width = 13;
    inline const double dt_wheel_diameter = 2.75;

    inline int dt_rpm = 360;
    inline uint8_t dt_horizontal_drift = 2;

    inline const std::initializer_list<int8_t> right_motor_ports = {8, -9, 10};
    inline const std::initializer_list<int8_t> left_motor_ports = {-5, -19, 20};

    // Intake
    inline const std::initializer_list<int8_t> intake_motor_ports = {-13, -14};
    inline const char indexer_ADI = 'B';

    // Color sort
    inline const char color_sort_ADI = 'A';
    inline const int8_t color_sensor_port = 8;

    // Imu
    inline const int8_t imu_port = 10;
    inline const double imu_wheel_diameter = 2.75;
    inline const double imu_wheel_distance = 0;

    // Encoders
    inline const char verticalEncoder_port = 'C';

    // Lateral PID
    inline const double lateral_kP = 10;
    inline const double lateral_kI = 0;
    inline const double lateral_kD = 3;

    inline const double lateral_anti_windup = 3;
    inline const double lateral_small_error_range = 1;
    inline const double lateral_small_error_range_timeout = 100;
    inline const double lateral_large_error_range = 3;
    inline const double lateral_large_error_range_timeout = 500;
    inline const double lateral_slew = 20;

     // Angular PID
    inline const double angular_kP = 3;
    inline const double angular_kI = 0.1;
    inline const double angular_kD = 18.0;

    inline const double angular_anti_windup = 3;
    inline const double angular_small_error_range = 1;
    inline const double angular_small_error_range_timeout = 100;
    inline const double angular_large_error_range = 3;
    inline const double angular_large_error_range_timeout = 500;
    inline const double angular_slew = 20;

} // namespace config

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
inline pros::MotorGroup leftMotorGroup(config::left_motor_ports, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotorGroup(config::right_motor_ports, pros::MotorGearset::blue);
inline pros::MotorGroup intake(config::intake_motor_ports);

// Drive train settings
inline lemlib::Drivetrain driveTrain(&leftMotorGroup, &rightMotorGroup, config::dt_track_width, config::dt_wheel_diameter, config::dt_rpm, config::dt_horizontal_drift);

// Pneumatics
inline pros::ADIDigitalOut indexer(config::indexer_ADI);
inline pros::ADIDigitalOut color_sort(config::color_sort_ADI);

// Imu
inline pros::Imu imu(config::imu_port);

// Encoders
inline pros::adi::Encoder verticalEncoder(config::verticalEncoder_port, true); // true --> reversed

// Tracking Wheels
inline lemlib::TrackingWheel verticalTrackingWheel(&verticalEncoder, config::imu_wheel_diameter, config::imu_wheel_distance);

// Sensors
inline lemlib::OdomSensors sensors(nullptr, nullptr, &verticalTrackingWheel, nullptr, &imu);
inline pros::Optical color_sensor(config::color_sensor_port);

// PID
inline lemlib::ControllerSettings lateral_controller(config::lateral_kP, config::lateral_kI, config::lateral_kD, config::lateral_anti_windup, config::lateral_small_error_range, config::lateral_small_error_range_timeout, config::lateral_large_error_range, config::lateral_large_error_range_timeout, config::lateral_slew);
inline lemlib::ControllerSettings angular_controller(config::angular_kP, config::angular_kI, config::angular_kD, config::angular_anti_windup, config::angular_small_error_range, config::angular_small_error_range_timeout, config::angular_large_error_range, config::angular_large_error_range_timeout, config::angular_slew);

// Drive curves
inline lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband (out of 127)
                                            10, // minimum output for movement (out of 127)
                                            1.02 // curve gain (a - value)
);
inline lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband (out of 127)
                                        10, // minimum output for movement (out of 127)
                                        1.016 // curve gain (a - value)
);

// Chassis
inline lemlib::Chassis chassis(driveTrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);