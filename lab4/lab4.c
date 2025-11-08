#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define DEV_6 6 
#define DEV_2 2

static double test_1[DEV_6] = { -0.27, -0.4, 0.0, 0.0, 0.0, 0.0 }; // подняли руку
static double test_2[DEV_2] = { 0.5, 0.5 }; // разжали клешни
static double test_3[DEV_6] = { -0.27, 0.6, -0.9, 0.0, 0.0, 0.0 }; // опустил руку
static double test_4[DEV_2] = { 0, 0 }; // зажали клешни
static double test_5[DEV_6] = { -1, -0.4, 0.0, 0.0, 0.0, 0.0 }; // двигаемся к столу
static double test_6[DEV_6] = { -1, -0.2, 0.0, 0.0, 0.0, 0.0 }; // опускаемся к столу


static void move_motors(WbDeviceTag* motors, double* position, double ms) {
	int steps = (int)(ms / TIME_STEP);

	for (int step = 0; step < steps; ++step) {
		for (int j = 0; j < DEV_6; ++j) {
			wb_motor_set_position(motors[j], position[j]);
		}
		wb_robot_step(TIME_STEP);
	}
}

static void move_grip_motors(WbDeviceTag* motors, double* position, double ms) {
	int steps = (int)(ms / TIME_STEP);

	for (int step = 0; step < steps; ++step) {
		for (int j = 0; j < DEV_2; ++j) {
			wb_motor_set_position(motors[j], position[j]);
		}
		wb_robot_step(TIME_STEP);
	}
}

int main(int argc, char **argv) {

  wb_robot_init();

  WbDeviceTag motors[DEV_6];
  char* names[DEV_6] = {
	"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
	"wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
 
  for (int i = 0; i < DEV_6; ++i) {
	  motors[i] = wb_robot_get_device(names[i]);
	  wb_motor_set_velocity(motors[i], 0.5);
  }

  WbDeviceTag grip_motors[DEV_2];
  char* grip_names[DEV_2] = {
	  "gripper::right",
	  "gripper::left"
  };


  for (int i = 0; i < DEV_2; ++i) {
	  grip_motors[i] = wb_robot_get_device(grip_names[i]);
	  wb_motor_set_velocity(grip_motors[i], 0.5);
  }
  
  while (wb_robot_step(TIME_STEP) != -1) {
	  move_motors(motors, test_1, 1000);
	  move_grip_motors(grip_motors, test_2, 2000);
	  move_motors(motors, test_3, 2000);
	  move_grip_motors(grip_motors, test_4, 2000);
	  move_motors(motors, test_1, 1000); // используем test_1 еще раз
	  move_motors(motors, test_5, 1000);
	  move_motors(motors, test_6, 1000);
	  move_grip_motors(grip_motors, test_2, 2000);
	  move_motors(motors, test_1, 1000);
  };

 
  wb_robot_cleanup();

  return 0;
}
