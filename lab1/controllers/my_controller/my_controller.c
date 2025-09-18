#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>


#define TIME_STEP 64
#define SPEED 8


int main(int argc, char **argv) {

  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag ps[8];
  char ps_name[8][4] = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
  };

  for (int i = 0; i < 8; i++) {
      ps[i] = wb_robot_get_device(ps_name[i]);
      wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  
  while (wb_robot_step(TIME_STEP) != -1) {
    
      double ps_value[8];
      for (int i = 0; i < 8; i++) {
          ps_value[i] = wb_distance_sensor_get_value(ps[i]);
      }


      bool right_sensor = ps_value[0] > 100 || ps_value[1] > 100 || ps_value[2] > 100 || ps_value[3] > 100;
      bool left_sensor = ps_value[4] > 100 || ps_value[5] > 100 || ps_value[6] > 100 || ps_value[7] > 100;

      double left_speed = SPEED;
      double right_speed = SPEED;

      if (right_sensor) {
          left_speed = -0.25 * SPEED;
          right_speed = 0.25 * SPEED;
      }
      else if (left_sensor) {
          left_speed = 0.25 * SPEED;
          right_speed = -0.25 * SPEED;
      }

      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);

  };

  wb_robot_cleanup();

  return 0;
}
