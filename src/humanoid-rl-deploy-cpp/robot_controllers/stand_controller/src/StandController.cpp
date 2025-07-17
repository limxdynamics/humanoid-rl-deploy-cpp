/**
 * @file StandController.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Implementation of the humanoid robot standing controller.
 * 
 * Manages initialization, startup, and runtime control for transitioning 
 * the robot from its current state to a standing position using PID control.
 */

#include "StandController.h"
#include <string.h>
#include <algorithm>

namespace humanoid {
  /**
   * Initializes controller parameters from YAML configurations.
   * Loads gains and target positions based on the ROBOT_TYPE environment variable.
   * 
   * @param config Base configuration node (typically from main ability config).
   * @return true on successful initialization, false on errors.
   */
  bool StandController::on_init(const YAML::Node& config) {
    try {
      // Load update rate from config if available
      if (config["update_rate"]) {
        update_rate_ = config["update_rate"].as<uint32_t>();
      }

      // Get robot type from environment variable
      std::string robot_type;
      const char* robot_type_value = ::getenv("ROBOT_TYPE");
      if (robot_type_value && strlen(robot_type_value) > 0) {
        robot_type = std::string(robot_type_value);
      } else {
        std::cerr << "Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.";
        abort();
      }

      // Load robot-specific parameters
      std::string param_file = limxsdk::ability::path::etc() + "/stand_controller/" + robot_type +"/param.yaml";
      YAML::Node param_node = YAML::LoadFile(param_file);
      
      stand_kd_ = param_node["stand_kd"].as<std::vector<float> >();  // Derivative gains
      stand_kp_ = param_node["stand_kp"].as<std::vector<float> >();  // Proportional gains
      stand_pos_ = param_node["stand_pos"].as<std::vector<float> >(); // Target joint positions
    } catch (const std::exception& e) {
      std::cerr << "Error parsing YAML config: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  /**
   * Initializes controller state and captures initial joint angles.
   * Waits for valid robot state data before proceeding.
   */
  void StandController::on_start() {
    auto robot_state = get_robot_state();

    // Wait for valid robot state data
    limxsdk::ability::Rate rate(1);
    while(isRunning() && robot_state.motor_names.size() <= 0) {
      std::cout <<"Waiting for robot state data" << std::endl;
      robot_state = get_robot_state();
      rate.sleep();
    }

    // Capture initial joint angles and reset progress
    init_joint_angles_ = robot_state.q;
    current_step_ = 0;
    stand_progress_ = 0.0;
  }

  /**
   * Main control loop for executing the standing behavior.
   * Interpolates between initial and target positions over time.
   */
  void StandController::on_main() {
    limxsdk::ability::Rate rate(update_rate_);
    while(isRunning()) {
      auto robot_state = get_robot_state();
      auto robot = get_robot_instance();
      limxsdk::RobotCmd cmd(robot_state.motor_names.size());

      // Update progress through the standing transition
      current_step_ += 1;
      if (current_step_ < transition_steps_) {
        stand_progress_ = current_step_ * 1.0 / transition_steps_;
      } else {
        stand_progress_ = 1.0; // Fully transitioned to standing
      }
      
      // Compute interpolated joint targets
      for (std::size_t i = 0; i < robot_state.motor_names.size(); i++) {
        // Linear interpolation between initial and target positions
        cmd.q[i] = (1 - stand_progress_) * init_joint_angles_[i] + stand_progress_ * stand_pos_[i];
      }
      
      // Apply control parameters and send command
      cmd.motor_names = robot_state.motor_names;
      cmd.Kd = stand_kd_;  // Apply derivative gains
      cmd.Kp = stand_kp_;  // Apply proportional gains
      robot->publishRobotCmd(cmd);
      
      rate.sleep(); // Maintain update rate
    }
  }
}

// Register this controller as a LIMX ability for runtime discovery
LIMX_REGISTER_ABILITY(humanoid::StandController)
