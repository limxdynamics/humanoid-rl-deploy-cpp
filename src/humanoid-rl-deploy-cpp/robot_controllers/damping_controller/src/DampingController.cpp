/**
 * @file DampingController.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Implementation of the damping controller for humanoid robots.
 * 
 * Computes and applies damping torques to motor joints based on joint velocities
 * to reduce oscillations and improve system stability.
 */

#include "DampingController.h"
#include <string.h>

namespace humanoid {
  /**
   * Initialize the damping controller with configuration parameters.
   * Loads robot-specific damping coefficients from YAML configuration.
   * 
   * @param config YAML node containing controller configuration
   * @return true if initialization successful, false otherwise
   */
  bool DampingController::on_init(const YAML::Node& config) {
    try {
      // Read update rate from config if available
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

      // Load parameters based on robot type
      std::string param_file = limxsdk::ability::path::etc() + "/damping_controller/" + robot_type +"/param.yaml";
      YAML::Node param_node = YAML::LoadFile(param_file);
      
      // Read damping coefficients (Kd values) for each joint
      // Damping torque = -Kd * velocity (opposes motion)
      damping_kd_ = param_node["damping_kd"].as<std::vector<float> >();
    } catch (const std::exception& e) {
      std::cerr << "Error parsing YAML config: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  /**
   * Perform startup procedures before entering main control loop.
   * Ensures valid robot state data is available before proceeding.
   */
  void DampingController::on_start() {
    auto robot_state = get_robot_state();

    // Wait for valid motor state data before proceeding
    limxsdk::ability::Rate rate(1);
    while(isRunning() && robot_state.motor_names.size() <= 0) {
      std::cout <<"Waiting for robot state data" << std::endl;
      robot_state = get_robot_state();
      rate.sleep();
    }
  }

  /**
   * Main control loop for applying damping forces.
   * Computes damping torques proportional to joint velocities and publishes commands.
   */
  void DampingController::on_main() {
    limxsdk::ability::Rate rate(update_rate_);
    while(isRunning()) {
      // Get current robot state
      auto robot_state = get_robot_state();

      // Prepare damping command
      auto robot = get_robot_instance();
      limxsdk::RobotCmd cmd(robot_state.motor_names.size());
      
      // Apply damping coefficients to all joints
      cmd.motor_names = robot_state.motor_names;
      cmd.Kd = damping_kd_;  // Sets velocity gain for damping control
      
      // Publish command to apply damping
      robot->publishRobotCmd(cmd);

      rate.sleep(); // Maintain specified update rate
    }
  }
}

// Register this controller as a LIMX ability for runtime discovery
LIMX_REGISTER_ABILITY(humanoid::DampingController)
