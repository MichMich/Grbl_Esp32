/*
	custom_code_template.cpp (copy and use your machine name)
	Part of Grbl_ESP32

	copyright (c) 2020 -	Bart Dring. This file was intended for use on the ESP32

  ...add your date and name here.

	CPU. Do not use this with Grbl for atMega328P

	Grbl_ESP32 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Grbl_ESP32 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

	=======================================================================

This is a template for user-defined C++ code functions.  Grbl can be
configured to call some optional functions. These functions have weak definitions
in the main code. If you create your own version they will be used instead

Put all of your functions in a .cpp file in the "Custom" folder.
Add this to your machine definition file
#define CUSTOM_CODE_FILENAME    "../Custom/YourFile.cpp"

Be careful to return the correct values

===============================================================================

Below are all the current weak function
*/

static bool alternative_joint_angle = false;

void apply_inverse_kinematics(float* motor_target);
void apply_forward_kinematics(float* cartesian);

bool limitsCheckTravel(float* target) {

    float cartesian[N_AXIS];
    motors_to_cartesian(cartesian, target, N_AXIS);
    float distance = sqrt(cartesian[X_AXIS] * cartesian[X_AXIS] + cartesian[Y_AXIS] * cartesian[Y_AXIS]);

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Target: %f, %f, Distance: %f", target[X_AXIS],  target[Y_AXIS],  distance);


    if (distance > X_LENGTH + Y_LENGTH) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Out of reach. Target too far.");
      return true;
    }

    if (distance < X_LENGTH - Y_LENGTH) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Out of reach. Target too close.");
      return true;
    }

    float min_x = -25;
    float max_x = 25;
    float min_y = -50 + target[X_AXIS] / 2;
    float max_y = 50 + target[X_AXIS] / 2;

    if (target[X_AXIS] < min_x || target[X_AXIS] > max_x) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "X Out of Reach: %f, Limits: %f - %f", target[X_AXIS], min_x, max_x);
      return true;
    }

    if ((target[Y_AXIS]) < min_y || target[Y_AXIS] > max_y) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Y Out of Reach: %f, Limits: %f - %f", target[Y_AXIS], min_y, max_y);
      return true;
    }

    if (target[Z_AXIS] < -160 || target[Z_AXIS] > 0) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Z Out of Reach: %f", target[Z_AXIS]);
      return true;
    }

    return false;
}

bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    float motor_target[N_AXIS];

    motor_target[X_AXIS] = target[X_AXIS];
    motor_target[Y_AXIS] = target[Y_AXIS];
    motor_target[Z_AXIS] = target[Z_AXIS];
    motor_target[A_AXIS] = target[A_AXIS];
    motor_target[B_AXIS] = target[B_AXIS];

    apply_inverse_kinematics(motor_target);

    // Compensate for A-motor rotation.
    // motor_target[A_AXIS] = motor_target[A_AXIS] - motor_target[X_AXIS] - motor_target[Y_AXIS];

    // Compensate relative rotation of X-motor.
    motor_target[Y_AXIS] = motor_target[Y_AXIS] + motor_target[X_AXIS] / 2;

    return mc_line(motor_target, pl_data);
}

void motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
    cartesian[X_AXIS] = motors[X_AXIS];
    cartesian[Y_AXIS] = motors[Y_AXIS];
    cartesian[Z_AXIS] = motors[Z_AXIS];
    cartesian[A_AXIS] = motors[A_AXIS];
    cartesian[B_AXIS] = motors[B_AXIS];

    // Compensate relative rotation of X-motor.
    cartesian[Y_AXIS] = cartesian[Y_AXIS] - cartesian[X_AXIS] / 2;

    // Compensate for A-motor rotation.
    cartesian[A_AXIS] = cartesian[A_AXIS] + cartesian[X_AXIS] + cartesian[Y_AXIS];

    apply_forward_kinematics(cartesian);
}

/** HELPER FUNCTIONS **/
void apply_inverse_kinematics(float* cartesian) {
    float x = cartesian[X_AXIS];
    float y = cartesian[Y_AXIS] * -1;

    float motor_angle_x;
    float motor_angle_y;

    float distance_from_base_to_target = sqrt(x * x + y * y);
    float angle_from_base_to_target = atan2(x, y) * 180 / M_PI;

    if (distance_from_base_to_target > X_LENGTH + Y_LENGTH) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Target not reachable: too far.");

      motor_angle_x = angle_from_base_to_target;
      motor_angle_y = 0;
    }

    else if (distance_from_base_to_target < X_LENGTH - Y_LENGTH) {
      grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Target not reachable: too close.");

      motor_angle_x = distance_from_base_to_target > 0 ? angle_from_base_to_target : 0;
      motor_angle_y = angle_from_base_to_target >= 0 ? 180 : -180;
    }

    else {
      // Inner angle alpha
      float cos_angle_x = ((distance_from_base_to_target * distance_from_base_to_target) + (X_LENGTH * X_LENGTH) - (Y_LENGTH * Y_LENGTH)) / (2 * distance_from_base_to_target * X_LENGTH);
      float angle_x = acos(cos_angle_x) * 180 / M_PI;

      // Inner angle beta
      float cos_angle_y = ((Y_LENGTH * Y_LENGTH) + (X_LENGTH * X_LENGTH) - (distance_from_base_to_target * distance_from_base_to_target)) / (2 * Y_LENGTH * X_LENGTH);
      float angle_y = acos(cos_angle_y) * 180 / M_PI;

      // Decide which way we want the second joint to go
      if (angle_from_base_to_target - angle_x < -90) alternative_joint_angle = false;
      if (angle_from_base_to_target + angle_x > 90) alternative_joint_angle = true;

      if (alternative_joint_angle) {
        motor_angle_x = (angle_from_base_to_target - angle_x);
        motor_angle_y = (180 - angle_y);
      } else {
        motor_angle_x = (angle_from_base_to_target + angle_x);
        motor_angle_y = (180 + angle_y);
      }

      if (motor_angle_x < -180) motor_angle_x += 360;
      if (motor_angle_x > 180) motor_angle_x -= 360;

      if (motor_angle_y < -180) motor_angle_y += 360;
      if (motor_angle_y > 180) motor_angle_y -= 360;
    }

    // Set new targets and devide by 3.6 because the angle is in DEG (360), and we need a value of 100.
    cartesian[X_AXIS] = motor_angle_x / 3.6;
    cartesian[Y_AXIS] = motor_angle_y / 3.6;
}

void apply_forward_kinematics(float* cartesian) {
  // The * 3.6 is because we work with a value of 0 - 100, in stead of 0 - 360 (DEG).
  float angle_x = cartesian[X_AXIS] * 3.6;
  float angle_y = cartesian[Y_AXIS] * 3.6;

  float alpha_x = X_LENGTH * sin(angle_x * M_PI / 180);
  float alpha_y = X_LENGTH * cos(angle_x * M_PI / 180);

  float beta_x = (Y_LENGTH * sin((angle_x + angle_y) * M_PI / 180)) + alpha_x;
  float beta_y = (Y_LENGTH * cos((angle_x + angle_y) * M_PI / 180)) + alpha_y;

  cartesian[X_AXIS] = beta_x;
  cartesian[Y_AXIS] = beta_y * -1;

  // Compensate for A-motor rotation.
  cartesian[A_AXIS] = cartesian[A_AXIS] + cartesian[X_AXIS] + cartesian[Y_AXIS];
}