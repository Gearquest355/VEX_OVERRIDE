#include "main.h"
#include "lemlib/api.hpp"

// left motor group
MotorGroup left_motor_group({-20, -10}, pros::MotorGears::blue);

// right motor group
MotorGroup right_motor_group({11, 1}, pros::MotorGears::blue);


//intake motor 
pros::Motor intake(5, pros::v5::MotorGears::green); 

//cascade motors
pros::MotorGroup cascade({-6, 7}, pros::v5::MotorGears::green);

//spin outtake
pros::Motor spin(8, pros::v5::MotorGears::green);

//move outtake
pros::Motor move(9, pros::v5::MotorGears::green);


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
pros::Imu imu(17);


// odometry settings
// No tracking wheels
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateral_controller(
    6.4,    // kP-used to be 10
    0.0025, // kI
    24.0,   // kD-used to be 3
    0,      // Anti-windup
    0.2,    // Small error range (0.2 inches)
    100,    // Small error timeout
    1.0,    // Large error range
    500,    // Large error timeout
    0       // Slew rate
);

lemlib::ControllerSettings angular_controller(
    2.8,   // kP-used to be 3
    0.005, // kI
    24.0,  // kD-used to be 18.5
    0,     // Anti-windup
    0.25,  // Small error range (degrees)
    100,   // Small error timeout
    1.0,   // Large error range
    500,   // Large error timeout
    0      // Slew rate
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
        leftY = leftY; 
        rightY = rightY;

        chassis.tank(rightY, leftY);

        pros::delay(25);

        // Intake controls
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);   // Intake / spin forward
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);  // Outtake / reverse
        } else {
            intake.move(0);     // Stop intake
        }

        pros::delay(25);
   
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            cascade.move(127);   // Move up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            cascade.move(-127);  // Move down
        } else {
            cascade.move(0);     // Stop
        }

        pros::delay(25);

         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            spin.move(127);   // Move up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            spin.move(-127);  // Move down
        } else {
            spin.move(0);     // Stop
        }


         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            move.move(127);   // Move up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            move.move(-127);  // Move down
        } else {
            move.move(0);     // Stop
        }

        pros::delay(25);

    }
}

    



// autonomous
void autonomous() {
/*
    chassis.setPose(0, 0, 0);

    // //chassis.moveToPoint(
    //     0, 48, 2000,{ .forwards = true, .maxSpeed = 80}
    // //);

   chassis.turnToHeading(90, 20000);
   */

chassis.turnToHeading(90, 3000, {.maxSpeed = 70});
chassis.waitUntilDone();



// Drive 48 inches straight forward at moderate speed
    //chassis.moveToPoint(0, 48, 4000, {.forwards = true, .maxSpeed = 70});
   // chassis.waitUntilDone();
}





