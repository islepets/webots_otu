#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define TIME_STEP 64
#define MAX_SPEED 4.0

static double Kp = 1.0;
static double Ki = 0.5;
static double Kd = 0.2;

double get_bearing_in_degrees(double* north) {
    double angle_rad = atan2(north[0], north[1]);

    while (angle_rad > 3.14) angle_rad -= 6.28;
    while (angle_rad < -3.14) angle_rad += 6.28;

    return angle_rad;
}

int main(int argc, char** argv) {

    wb_robot_init();

    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);

    WbDeviceTag compas = wb_robot_get_device("compass");
    wb_compass_enable(compas, TIME_STEP);

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    double prev_error = 0.0;
    double integral = 0.0;

    wb_robot_step(TIME_STEP);

    while (wb_robot_step(TIME_STEP) != -1) {

        double* position, * compas_value;
        double x, y, target_y, dx_dy, target_theta, current_theta, err, derivative, correction;
        double left_speed, right_speed;

        compas_value = wb_compass_get_values(compas);
        current_theta = get_bearing_in_degrees(compas_value);

        position = wb_gps_get_values(gps);

        x = position[0];
        y = position[1];


        target_y = -sqrt(x) - 4;

        dx_dy = -1.0 / (2 * sqrt(x));

        target_theta = atan(dx_dy);

        err = target_theta - current_theta;

        while (err > 3.14) err -= 6.28;
        while (err < -3.14) err += 6.28;

        integral += err * (TIME_STEP / 1000.0);
        derivative = (err - prev_error) / (TIME_STEP / 1000.0);
        prev_error = err;

        correction = Kp * err + Ki * integral + Kd * derivative;

        if (correction < -1.0) correction = -1.0;
        if (correction > 1.0) correction = 1.0;

        left_speed = MAX_SPEED * (1.0 - correction);
        right_speed = MAX_SPEED * (1.0 + correction);

        if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);

    };

    wb_robot_cleanup();

    return 0;
}