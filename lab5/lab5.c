#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define Kp 2

int main() {
    wb_robot_init();

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    wb_motor_set_position(left_motor, INFINITY);

    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(right_motor, INFINITY);

    WbDeviceTag ir_sensors[4];
    char* names_sensors[4] = {
        "ground left infrared sensor",
        "ground front left infrared sensor",
        "ground front right infrared sensor",
        "ground right infrared sensor"
    };

    for (int i = 0; i < 4; ++i) {
        ir_sensors[i] = wb_robot_get_device(names_sensors[i]);
        wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
    }

    double sum_weight[4] = { -2.0, -1.0, 1.0, 2.0 };

    while (wb_robot_step(TIME_STEP) != -1) {

        double values[4];

        for (int i = 0; i < 4; ++i) {
            values[i] = wb_distance_sensor_get_value(ir_sensors[i]);
        }

        double max_val = 0.0;

        for (int i = 0; i < 4; ++i) {
            if (values[i] > max_val)
                max_val = values[i];
        }

        double threshold = max_val * 0.6;

        double error = 0.0;
        double sum_val = 0.0;
        int active_sensor = 0;

        for (int i = 0; i < 4; i++) {
            if (values[i] > threshold) {
                error += sum_weight[i] * (values[i] - threshold);
                sum_val += values[i] - threshold;
                active_sensor++;
            }
        }

        double left_speed = 10.0;
        double right_speed = 10.0;
        double correction = 0.0;

        if (active_sensor > 0) {
            error = error / sum_val;

            correction = Kp * error;

            left_speed = 10.0 + correction;
            right_speed = 10.0 - correction;
        }
        else {
            left_speed = -10.0 * 0.4;
            right_speed = 10.0 * 0.4;
        }

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);

    }

    wb_robot_cleanup();
    return 0;
}