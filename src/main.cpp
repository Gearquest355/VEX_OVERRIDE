#include "main.h"
#include "lemlib/api.hpp"

// left motor group
MotorGroup left_motor_group({-20, -10}, pros::MotorGears::blue);

// right motor group
MotorGroup right_motor_group({11, 1}, pros::MotorGears::blue);


// drivetrain settings
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    10,                              // track width
    lemlib::Omniwheel::NEW_275,      // 2.75 inch wheels
    450,                             // blue motor RPM
    2                                // horizontal drift
);


// IMU
pros::Imu imu(-9);


// odometry settings
// No tracking wheels
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    0,//previously 10
    0,
    0,//previously 3
    0,
    0,
    0,
    0,
    0,
    0
);


// angular PID controller
lemlib::ControllerSettings angular_controller(
    3,//previously 2
    0,
    14,//previously 10
    0,
    0,
    0,
    0,
    0,
    0
);


// create chassis
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// initialize
void initialize() {

    pros::lcd::initialize();

    chassis.calibrate();

    pros::Task screen_task([&]() {
        while (true) {

            pros::lcd::print(
                0,
                "X: %f",
                chassis.getPose().x
            );

            pros::lcd::print(
                1,
                "Y: %f",
                chassis.getPose().y
            );

            pros::lcd::print(
                2,
                "Heading: %f",
                chassis.getPose().theta
            );

            pros::delay(100);
        }
    });
}


// driver control
void opcontrol() {

    while (true) {

        int leftY = controller.get_analog(
            pros::E_CONTROLLER_ANALOG_LEFT_Y
        );

        int rightY = controller.get_analog(
            pros::E_CONTROLLER_ANALOG_RIGHT_Y
        );

        // Limit speed to 60%
        leftY = leftY * 60 / 127;
        rightY = rightY * 60 / 127;

        chassis.tank(rightY, leftY);

        pros::delay(25);
    }
}


// autonomous
void autonomous() {

    chassis.setPose(0, 0, 0);

    // //chassis.moveToPoint(
    //     0, 48, 2000,{ .forwards = true, .maxSpeed = 80}
    // //);

   chassis.turnToHeading(90, 20000);

}
