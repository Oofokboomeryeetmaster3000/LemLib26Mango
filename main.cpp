#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, -3, -11},pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-9, 10, 19},pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::Distance Distance(4);

pros::MotorGroup Intake({-20,12}, pros::MotorGearset::blue);
pros::adi::DigitalOut MatchLoader ('D');
pros::adi::DigitalOut Descore ('C');
pros::adi::DigitalOut DoublePark ('A');
pros::adi::DigitalOut RedB ('H');
pros::adi::DigitalOut RedT ('G');

pros::Rotation rotation_sensor(-13);

lemlib::TrackingWheel hwheel(&rotation_sensor, lemlib::Omniwheel::NEW_2, -5.1);


void HoldBalls1() {
    RedB.set_value(false);
    RedT.set_value(true);
}

void ScoreTop1() {
    RedB.set_value(false);
    RedT.set_value(false);
}

void ScoreMid1() {
    RedB.set_value(true);
    RedT.set_value(true);
}

void reverseIntakeUntilLoaded() {
	
    // 1. Start the intake in reverse (e.g., -127 for full speed)
	MatchLoader.set_value(true);
    //Intake.move(-50);

    // 2. Loop while the distance is GREATER than 100mm
    // .get() returns millimeters by default/*
    //while (Distance.get() > 100) {
        // Optional: Print distance for debugging
        //pros::lcd::print(1, "Intaking... Dist: %d", Distance.get());
        
        // Small delay to allow other tasks to run
      //  pros::delay(20);
    //}

    // 3. Stop the motor once the loop condition is met (< 100mm)
	//Intake.move(0);

	Intake.move(0);

	pros::delay(600);

	Intake.move(0);
	Intake.brake();
	DoublePark.set_value(true);
/*
    // 2. Loop while the distance is GREATER than 100mm
    // .get() returns millimeters by default
    while (Distance.get() < 100) {
        // Optional: Print distance for debugging
        //pros::lcd::print(1, "Intaking... Dist: %d", Distance.get());
        
        // Small delay to allow other tasks to run
        pros::delay(20);
    }
	Intake.move(0);
	Intake.brake();

	
    pros::lcd::print(1, "Intaking... Dist: %d", Distance.get());
	*/
}



// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(16);

lemlib::OdomSensors sensors(
    nullptr, // vertical wheel 1
    nullptr, // vertical wheel 2
    &hwheel, // horizontal wheel 1
    nullptr, // horizontal wheel 2
    &imu     // inertial sensor (REQUIRED)
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(23, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    chassis.calibrate();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPose(0, 24, 0, 99999);
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

int robotdeg;
void opcontrol() {
	bool mlext = false;
	bool rext = false;
	bool dpext = false;
    rotation_sensor.reset_position();
	while (true) {
        
        robotdeg = (rotation_sensor.get_position()/400);
        
        pros::lcd::print(1, "Rotation Sensor: %i", robotdeg);

		int distance = Distance.get();
		//pros::lcd::print(0, "Distance: %d mm", distance);
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, leftX);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			ScoreMid1();
			Intake.move(100);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			Intake.move(100);
			ScoreTop1();
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			Intake.move(-100);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			HoldBalls1();
			Intake.move(100);
		}
		else {
			Intake.move(0);
		}

		if (controller.get_digital_new_press(DIGITAL_B)) {
            mlext = !mlext; // Switch the state
            MatchLoader.set_value(mlext); // Apply to piston
        }

		if (controller.get_digital_new_press(DIGITAL_Y)) {
            rext = !rext; // Switch the state
            Descore.set_value(rext); // Apply to piston
        }

        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            dpext = !dpext; // Switch the state
            DoublePark.set_value(dpext); // Apply to piston
        }


		if (controller.get_digital_new_press(DIGITAL_DOWN)) {
			reverseIntakeUntilLoaded();
            //dpext = !dpext; // Switch the state
            //MatchLoader.set_value(dpext); // Apply to piston
        }

        // delay to save resources
        pros::delay(25);
		
    }
}