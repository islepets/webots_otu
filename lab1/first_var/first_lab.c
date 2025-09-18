#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <float.h>

#define TIME_STEP 64
#define speed_move 10
#define speed_turn 2

typedef void(*MoveFunc)(WbDeviceTag left, WbDeviceTag right);

typedef struct {
	MoveFunc move;
	int duration;
}MoveSequence;


void move_forward(WbDeviceTag left, WbDeviceTag right) {
	wb_motor_set_velocity(left, speed_move);
	wb_motor_set_velocity(right, speed_move);
}

void turn_left(WbDeviceTag left, WbDeviceTag right) {
	wb_motor_set_velocity(left, -speed_turn);
	wb_motor_set_velocity(right, speed_turn);
}

void turn_right(WbDeviceTag left, WbDeviceTag right) {
	wb_motor_set_velocity(left, speed_turn);
	wb_motor_set_velocity(right, -speed_turn);
}

void stop(WbDeviceTag left, WbDeviceTag right) {
	wb_motor_set_velocity(left, 0);
	wb_motor_set_velocity(right, 0);
}

MoveSequence seq[] = {
	{turn_left, 1},
	{move_forward, 70},
	{turn_right, 20},
	{move_forward, 100},
	{turn_left, 10},
	{move_forward, 50},
	{turn_left, 15},
	{move_forward, 170},
	{turn_left, 15},
	{move_forward, 30},
	{turn_left, 8},
	{move_forward, 40},
	{turn_left, 5},
	{move_forward, 150},
	{stop, 100000000000}
};



int main(int argc, char** argv) {

	wb_robot_init();

	WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
	WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);

	int sq_ind = 0;
	int count = 0;

	int SIZE = sizeof(seq) / sizeof(seq[0]);
	bool flag = true;

	while (wb_robot_step(TIME_STEP) != -1 && flag) {
		count++;
		seq[sq_ind].move(left_motor, right_motor);

		if (count >= seq[sq_ind].duration) {
			sq_ind = (sq_ind + 1) % SIZE;
			count = 0;
		}
	}

	wb_robot_cleanup();

	return 0;
}
