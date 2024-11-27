#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


//controller 
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//motor groups
pros::MotorGroup LeftMotor({-18,19,20}, pros::MotorGearset::blue);
pros::MotorGroup RightMotor({10,9,-8}, pros::MotorGearset::blue);
//pros::MotorGroup LadyBrown({-19,18}, pros::MotorGearset::green);


pros::MotorGroup Intake({12, -2});
pros::Motor Intake1(-2);
pros::Motor Intake2(12);
pros::Motor Hook(11,pros::MotorGearset::blue);

//pneumatic
pros::adi::Pneumatics Clamp(1, false);
//pros::ADIAnalogIn Doinker(2);

//sensors
pros::Imu inertial(7);
//pros::Rotation x(9);
//pros::Rotation y(10);

//tracking wheels
//lemlib::TrackingWheel horizontal(&x, lemlib::Omniwheel::NEW_2, 0);
//lemlib::TrackingWheel vertical(&y,lemlib::Omniwheel::NEW_2, 0);

lemlib::Drivetrain drivetrain(&LeftMotor, &RightMotor, 6, lemlib::Omniwheel::NEW_325, 600, 2);

lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//CHANGE ONCE WE GET ODOM PODS ON
lemlib::OdomSensors sensors(nullptr,
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                        	nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
ASSET(path_jerryio_txt);

void autonomous() {
chassis.setPose(0,0,0);
chassis.follow(path_jerryio_txt, 15, 10000);
}


void runIntake() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
		Intake1.move(-127);
		Intake2.move(-127);
		Hook.move(127);
	}
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        Intake1.move(127);
        Intake2.move(127);
        Hook.move(-127);
    }
	else{
		Intake.brake();
		Hook.brake();
		}
}

void Clamping(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        Clamp.extend();
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        Clamp.retract();
    }
}
/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors


    while (true) {
        // get joystick positions`
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(-rightX, -leftY,false);  
        runIntake();
        Clamping();
        // delay to save resources
        pros::delay(10);
    }
}