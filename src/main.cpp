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
    10.1,                              // track width 
    lemlib::Omniwheel::NEW_2,      // 2.75 inch wheels
    450,                             // blue motor RPM
    2                                // horizontal drift
);


// IMU
pros::Imu imu(17);
// rotation sensor vertical 
pros::Rotation rotation_sensor(3); //come back here if change 1 -> -1

//change the port of the rotation sensor ! !! 

//defining tracking wheel-ODOMETRY 
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_2, -0.5); //make sure the -2.5 inch here is changed

// odometry settings
// No tracking wheels
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, 
    nullptr,
    nullptr,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateral_controller(
    10,    // kP-used to be 10 // between 3.1 to 6
    0.00, // kI
    10,   // kD-used to be 3
    0,      // Anti-windup
    0.2,    // Small error range (0.2 inches)
    100,    // Small error timeout
    1.0,    // Large error range
    500,    // Large error timeout
    0       // Slew rate
);

lemlib::ControllerSettings angular_controller(
    2.8,   // kP-used to be 3
    0.000, // kI
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


        // Intake controls
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);   // Intake / spin forward
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);  // Outtake / reverse
        } else {
            intake.move(0);     // Stop intake
        }

   
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            cascade.move(127);   // Move up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            cascade.move(-127);  // Move down
        } else {
            cascade.move(0);     // Stop
        }


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






void redSideAuto() {
/*
    //EXAMPLE CODE WITHOUT ODOM OR PID
    // Drive backward at 40% power for 1 second
    left_motor_group.move(-40);
    right_motor_group.move(-40);
    pros::delay(1000);

    // Stop the drivetrain
    left_motor_group.move(0);
    right_motor_group.move(0);

    pros::delay(250);
    */
}
// autonomous
void autonomous() {
    move.tare_position();
    //redSideAuto();
    //runnning the intake slowly to keep the pin 
    spin.move(-5);

    chassis.setPose(0, 0, -47);
    //change toggle 
    chassis.moveToPoint(0, 6, 1000, {.minSpeed = 127}); 
    //change toggle
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -2, 1000,{.forwards = false});
    //change toggle
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 6, 1000,{.minSpeed = 127}); 
    //change toggle
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -2, 1000,{.forwards = false});
    chassis.waitUntilDone();

    move.move_absolute(1100, 200); 
    //this is to move the intake arm 

    
    chassis.moveToPoint(13, -6, 1500, {.forwards = false});
    chassis.waitUntilDone();
    //turning to the red goal, position robot in front of the goal(our color)

    spin.move(127);
    pros::delay(200);
    spin.move(0);
    //drop pin in goal(our color)
    pros::delay(300);

    chassis.moveToPoint(0, -6, 1500, {.forwards = true}); //moving back 
    chassis.waitUntilDone();

    chassis.turnToHeading(-36, 1000);
    //turn towards pin on the right of the goal
    chassis.waitUntilDone();
    //lift intake arm
    move.move_absolute(1700, 200); 

    chassis.moveToPoint(17.5, -25, 1500, {.forwards = false, .maxSpeed = 60} );
    //move towards the pin exact position

    //move the intake down, also spin the intake 
    chassis.waitUntilDone();
    move.move_absolute(900, -200);

   spin.move(100);
   pros::delay(400);
   spin.move(0);
  
    
}






