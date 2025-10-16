#include <math.h>
#include <stdio.h>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "pid_controller.h"

#define TAKEOFF_ALTITUDE 0.18  // 18 cm
#define FORWARD_DISTANCE 0.16  // 16 cm

typedef enum {
    STATE_INIT = 0,
    STATE_MOVE,
    STATE_HOVER
}drone_state_t;

int main(int argc, char** argv) {
    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
    wb_motor_set_position(m1_motor, INFINITY);
    wb_motor_set_velocity(m1_motor, -1.0);
    WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
    wb_motor_set_position(m2_motor, INFINITY);
    wb_motor_set_velocity(m2_motor, 1.0);
    WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
    wb_motor_set_position(m3_motor, INFINITY);
    wb_motor_set_velocity(m3_motor, -1.0);
    WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
    wb_motor_set_position(m4_motor, INFINITY);
    wb_motor_set_velocity(m4_motor, 1.0);

    WbDeviceTag imu = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(imu, timestep);
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, timestep);

    while (wb_robot_step(timestep) != -1) {
        if (wb_robot_get_time() > 2.0)
            break;
    }

    drone_state_t current_drone_state = STATE_INIT;
    actual_state_t actual_state = { 0 };
    desired_state_t desired_state = { 0 };
    double past_x_global = 0;
    double past_y_global = 0;
    double past_time = wb_robot_get_time();

    double initial_x = 0;
    double initial_y = 0;
    double target_altitude = TAKEOFF_ALTITUDE;

    gains_pid_t gains_pid;
    gains_pid.kp_att_y = 1;
    gains_pid.kd_att_y = 0.5;
    gains_pid.kp_att_rp = 0.5;
    gains_pid.kd_att_rp = 0.1;
    gains_pid.kp_vel_xy = 2;
    gains_pid.kd_vel_xy = 0.5;
    gains_pid.kp_z = 10;
    gains_pid.ki_z = 5;
    gains_pid.kd_z = 5;
    init_pid_attitude_fixed_height_controller();

    double height_desired = 0.0;

    motor_power_t motor_power;


    initial_x = wb_gps_get_values(gps)[0];
    initial_y = wb_gps_get_values(gps)[1];

    while (wb_robot_step(timestep) != -1) {
        const double dt = wb_robot_get_time() - past_time;

        actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
        actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
        actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
        actual_state.altitude = wb_gps_get_values(gps)[2];
        double x_global = wb_gps_get_values(gps)[0];
        double vx_global = (x_global - past_x_global) / dt;
        double y_global = wb_gps_get_values(gps)[1];
        double vy_global = (y_global - past_y_global) / dt;

        double actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
        double cosyaw = cos(actualYaw);
        double sinyaw = sin(actualYaw);
        actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
        actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

        desired_state.roll = 0;
        desired_state.pitch = 0;
        desired_state.vx = 0;
        desired_state.vy = 0;
        desired_state.yaw_rate = 0;
        desired_state.altitude = height_desired;

        
        switch (current_drone_state)
        {
        case STATE_INIT:

            height_desired = TAKEOFF_ALTITUDE;

            if (fabs(actual_state.altitude - height_desired) < 0.01) {
                current_drone_state = STATE_MOVE;
            }
            break;
        case STATE_MOVE:
            height_desired = TAKEOFF_ALTITUDE;
            desired_state.vx = 0.1;

            double current_distance = fabs(x_global - initial_x);
            if (fabs(current_distance - FORWARD_DISTANCE) < 0.025) {
                current_drone_state = STATE_HOVER;
            }
            break;
        case STATE_HOVER:
            height_desired = TAKEOFF_ALTITUDE;
            desired_state.vx = 0;
            desired_state.vy = 0;
            break;
        }


        desired_state.altitude = height_desired;
        pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

        printf("x = %.3f\ty = %.3f\n", wb_gps_get_values(gps)[0], wb_gps_get_values(gps)[2]);

        wb_motor_set_velocity(m1_motor, -motor_power.m1);
        wb_motor_set_velocity(m2_motor, motor_power.m2);
        wb_motor_set_velocity(m3_motor, -motor_power.m3);
        wb_motor_set_velocity(m4_motor, motor_power.m4);

        past_time = wb_robot_get_time();
        past_x_global = x_global;
        past_y_global = y_global;
    };

    wb_robot_cleanup();

    return 0;
}