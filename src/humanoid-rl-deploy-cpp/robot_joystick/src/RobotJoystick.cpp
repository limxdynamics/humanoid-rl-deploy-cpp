/**
 * @file RobotJoystick.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "RobotJoystick.h"
#include <string.h>

namespace humanoid {
  // Main control loop: Handles joystick input and controls robot states
  void RobotJoystick::on_main() {
    auto robot = get_robot_instance();

    // Subscribe to joystick sensor messages and handle button presses
    robot->subscribeSensorJoy([](const limxsdk::SensorJoyConstPtr& msg) {
      // State flags to track current robot mode
      static bool stand, mimic, damping, exited, walking;

      // Toggle to "stand" mode when buttons 4 and 3 are pressed
      int ret;
      if (msg->buttons[4] == 1 && msg->buttons[3] == 1 && !stand) {
        stand = true;
        damping = mimic = exited = walking = false;
        ret = system("echo 'switch \"mimic damping\" \"stand\"' | nc -w 1 127.0.0.1 8888");
      }
      
      // Toggle to "mimic" mode when buttons 4 and 1 are pressed
      if (msg->buttons[4] == 1 && msg->buttons[1] == 1 && !mimic) {
        mimic = true;
        damping = stand = exited = walking = false;
        ret = system("echo 'switch \"stand damping\" \"mimic\"' | nc -w 1 127.0.0.1 8888");
      }

      // Toggle to "walking" mode when buttons 7 and 0 are pressed
      if (msg->buttons[7] == 1 && msg->buttons[2] == 1 && !walking) {
        walking = true;
        damping = mimic = stand = exited = false;
        ret = system("echo 'switch \"stand damping\" \"walking\"' | nc -w 1 127.0.0.1 8888");
      }

      // Toggle to "damping" mode when buttons 4 and 0 are pressed
      if (msg->buttons[4] == 1 && msg->buttons[0] == 1 && !damping) {
        damping = true;
        mimic = stand = exited = walking = false;
        ret = system("echo 'switch \"stand mimic walking\" \"damping\"' | nc -w 1 127.0.0.1 8888");
      }
      
      // Stop all controllers when buttons 4 and 2 are pressed
      if (msg->buttons[4] == 1 && msg->buttons[2] == 1 && !exited) {
        exited = true;
        mimic = stand = damping = walking = false;
        ret = system("echo 'switch \"stand mimic damping walking\"' | nc -w 1 127.0.0.1 8888");
      }
    });

    // Prevent thread exit to sustain continuous button event reception
    limxsdk::ability::Rate rate(1000);
    while(isRunning()) {
      rate.sleep();
    }
  }
}

// Register this component as a LIMX ability
LIMX_REGISTER_ABILITY(humanoid::RobotJoystick)
