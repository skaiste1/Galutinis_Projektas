#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define NUM_DIST_SENSORS 8
#define CRUISING_VELOCITY 5.0
#define NEAR_FRONT 74
#define NEAR_SIDE 71

int main(int argc, char **argv) {
  wb_robot_init();

  // Get simulation step length.
  int time_step = (int)wb_robot_get_basic_time_step();

  // Get left and right wheel motors.
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Get frontal distance sensors.
  WbDeviceTag dist_sensors[NUM_DIST_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_DIST_SENSORS; ++i) {
    sprintf(sensor_name, "ps%d", i);
    dist_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(dist_sensors[i], time_step);
  }

  // Disable motor PID control mode.
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Set the initial velocity of the left and right wheel motors.
  wb_motor_set_velocity(left_motor, CRUISING_VELOCITY);
  wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);

  // Main loop.
  while (wb_robot_step(time_step) != -1) {
    double dist_sensor_values[NUM_DIST_SENSORS];
    for (int i = 0; i < NUM_DIST_SENSORS; ++i) {
      dist_sensor_values[i] = wb_distance_sensor_get_value(dist_sensors[i]);
    }

    printf("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
           dist_sensor_values[0], dist_sensor_values[1], dist_sensor_values[2],
           dist_sensor_values[3], dist_sensor_values[4], dist_sensor_values[5],
           dist_sensor_values[6], dist_sensor_values[7]);

    if (dist_sensor_values[0] > NEAR_FRONT || dist_sensor_values[7] > NEAR_FRONT) {
      printf("rotate\n");
      wb_motor_set_velocity(left_motor, -CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);
    } else if (dist_sensor_values[1] > NEAR_SIDE && dist_sensor_values[1] > dist_sensor_values[6]) {
      printf("turn left\n");
      wb_motor_set_velocity(left_motor, -0.1 * CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, 1.2 * CRUISING_VELOCITY);
    } else if (dist_sensor_values[6] > NEAR_SIDE) {
      printf("turn right\n");
      wb_motor_set_velocity(left_motor, 1.2 * CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, -0.1 * CRUISING_VELOCITY);
    } else {
      printf("go\n");
      wb_motor_set_velocity(left_motor, CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);
    }
  }

  wb_robot_cleanup();
  return 0;
}