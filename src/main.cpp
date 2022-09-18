#include "main.h"
using namespace pros;

// motor definitions //
pros::Motor left_front_motor(3, pros::E_MOTOR_GEARSET_18, false,
                             pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(11, pros::E_MOTOR_GEARSET_18, false,
                            pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(4, pros::E_MOTOR_GEARSET_18, true,
                              pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(12, pros::E_MOTOR_GEARSET_18, true,
                             pros::E_MOTOR_ENCODER_DEGREES);

pros::Imu inertial_sensor(10);

pros::ADIEncoder Y_encoder1('A', 'B', true);
pros::ADIEncoder Y_encoder2('C', 'D', false);
pros::ADIEncoder X_encoder('E', 'F', false);
// pros::ADIPort touch_sensor(TOUCH_SENSOR_PORT, pros::E_ADI_ANALOG_IN);
////
//test
// pros::ADIDigitalIn limit_switch(LIMIT_SWITCH_PORT);
// pros::ADIDigitalIn limit_switch({{EXTENDER_PORT, LIMIT_SWITCH_PORT}});

pros::Controller master(CONTROLLER_MASTER);
// pros::Vision front_vision(FRONT_VISION_PORT);
// pros::Vision back_vision(BACK_VISION_PORT);

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
  pros::lcd::set_text(1, "Hello tiger!");
	Y_encoder1.reset();
	Y_encoder2.reset();
	X_encoder.reset();
	inertial_sensor.reset();
	pros::delay(3000);

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
void autonomous() {}

double degreesToCm(double degree, double wheel_diameter_inch){
  return degree / 360 * (3.1415926 * wheel_diameter_inch * 2.54);
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
void opcontrol() {

float forward;
float turn;
double heading = 0;
double heading1 = 0;
double xEncoder=0  ;
double yEncoder=0  ;
double xEncoder1=0  ;
double yEncoder1=0  ;
double xEncoderUpdate=0  ;
double yEncoderUpdate=0  ;
double x=0;
double y=0;

  while (true) {

    forward = master.get_analog(ANALOG_LEFT_Y);
    turn = master.get_analog(ANALOG_RIGHT_X);

    left_front_motor.move(forward + turn);
    right_front_motor.move(forward - turn);
    left_back_motor.move(forward + turn);
    right_back_motor.move(forward - turn);

	  heading = inertial_sensor.get_rotation();
    if(heading==infinity()){
      heading = heading1;
    }else{
      heading1=heading;
    }
	  xEncoder = X_encoder.get_value() ;
	  yEncoder = (Y_encoder1.get_value() + Y_encoder2.get_value())/2 ;

	  xEncoderUpdate = xEncoder - xEncoder1 ;
	  yEncoderUpdate = yEncoder - yEncoder1 ;

	  xEncoder1 = xEncoder ;
	  yEncoder1 = yEncoder ;


    x += degreesToCm(xEncoderUpdate, 2.75) * cos(heading) + 
          degreesToCm(yEncoderUpdate, 2.75) * sin(heading) ;
    y +=  degreesToCm(yEncoderUpdate, 2.75) * cos(heading) + 
          degreesToCm(xEncoderUpdate, 2.75) * sin(heading);

    pros::lcd::print(3, "Heading:%.1f", heading);
    pros::lcd::print(4, "x:%.1f", x);
    pros::lcd::print(5, "y:%.1f", y);

    pros::delay(20);
  }
}
