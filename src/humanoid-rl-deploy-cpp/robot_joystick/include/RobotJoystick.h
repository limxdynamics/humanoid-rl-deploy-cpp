/**
 * @file RobotJoystick.h
 * @brief Joystick control module for humanoid robot
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _HUMANOID_ROBOT_JOYSTICK_
#define _HUMANOID_ROBOT_JOYSTICK_

#include "limxsdk/ability/base_ability.h"

namespace humanoid
{

  /**
   * RobotJoystick class - Handles joystick input for humanoid robot control
   * 
   * This ability component subscribes to joystick sensor data and 
   * translates button presses into robot state transitions (stand, mimic, damping).
   */
  class RobotJoystick : public limxsdk::ability::BaseAbility
  {
  protected:
    /**
     * Main execution loop for the joystick control ability
     * - Subscribes to joystick sensor messages
     * - Implements button combination detection for mode switching
     * - Maintains active state to ensure continuous event handling
     */
    void on_main() override;
  };
}

#endif
