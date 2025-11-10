/**
 * @file WalkingController.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Neural network-based walking controller for humanoid robots
 * 
 * Implements real-time walking control using a single policy network:
 * - Policy network: Generates motor commands from observations
 * - Gait control: Manages walking phase and frequency
 * - Joystick input: Processes user commands for velocity control
 */

#ifndef _HUMANOID_WALKING_CONTROLLER_
#define _HUMANOID_WALKING_CONTROLLER_

#include "limxsdk/ability/base_ability.h"
#include <onnxruntime_cxx_api.h>  // For ONNX model inference
#include <Eigen/Geometry>         // For vector/matrix operations
#include <chrono>                 // For FPS timing

namespace humanoid
{
  // Eigen vector types for efficient math operations
  using vector3_t = Eigen::Matrix<float, 3, 1>;  // 3D vector (x,y,z)
  using vector_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;  // Dynamic vector

  /**
   * @class WalkingController
   * @brief Neural walking controller using ONNX models
   * 
   * Implements walking control with:
   * 1. Policy network for motor command generation
   * 2. Gait phase management for walking rhythm
   * 3. Joystick input processing for velocity commands
   */
  class WalkingController : public limxsdk::ability::BaseAbility
  {
  protected:
    /**
     * @brief Initialize controller with configuration
     * @param config YAML containing model paths and parameters
     * @return true if successful, false on error
     */
    bool on_init(const YAML::Node &config) override;

    /**
     * @brief Prepare controller for execution
     * 
     * Initializes models, buffers, and state variables before control begins
     */
    void on_start() override;

    /**
     * @brief Main control loop
     * 
     * Executes at fixed rate to:
     * 1. Process sensor data and joystick input
     * 2. Run neural network inference
     * 3. Generate motor commands
     */
    void on_main() override;

  protected:
    /**
     * @brief Load ONNX policy model for specified robot configuration
     * @param robot_type Identifier for model/parameter set
     */
    void load_model(const std::string& robot_type);

    /**
     * @brief Load control parameters from YAML
     * @param robot_type Identifier for parameter set
     */
    void load_cfg(const std::string& robot_type);

    /**
     * @brief Generate motor commands via policy inference
     */
    void compute_actions();

    /**
     * @brief Construct observation vector
     * 
     * Combines: IMU data, joint states, previous actions, commands, and gait observations
     */
    void compute_observation();

    /**
     * @brief Compute gait-related observations
     * 
     * Generates gait frequency, offset, height, and phase information
     */
    void compute_gait_observation();

    /**
     * @brief Update commands from joystick input
     */
    void update_commands_from_joy();

  private:
    // Control frequency in Hz
    uint32_t update_rate_{1000};

    // ONNX Runtime components
    std::shared_ptr<Ort::Env> onnxEnvPrt_;  // Shared ORT environment
    
    // Policy network configuration
    std::vector<std::vector<int64_t>> policy_input_shapes_;
    std::vector<std::vector<int64_t>> policy_output_shapes_;
    std::unique_ptr<Ort::Session> policy_session_ptr_;
    std::vector<const char *> policy_input_names_; 
    std::vector<const char *> policy_output_names_;

    // Gait control state
    float gait_phase_{0.};        // Normalized gait phase [0,1]
    float gait_freq_{0.85};       // Gait frequency
    float gait_offset_{0.5};      // Gait offset
    float gait_height_{0.12};     // Fixed gait height

    // Joystick input and commands
    vector3_t commands_{vector3_t::Zero()};  // Velocity commands (x, y, z)
    float max_vx_{1.0};           // Max forward velocity
    float max_vy_{0.3};           // Max lateral velocity  
    float max_vz_{1.0};           // Max angular velocity
    std::vector<float> joy_axes_; // Current joystick axes values

    // Safety limits
    std::vector<float> user_torque_limit_;  // Per-joint torque caps

    // Control parameters
    std::vector<float> action_scale_;  // Policy output scaling
    std::vector<float> stiffness_;     // Position gains (Kp)
    std::vector<float> damping_;       // Velocity gains (Kd)
    std::vector<float> default_angle_; // Default joint angles

    // Network configuration
    int32_t decimation_;        // Control cycle decimation
    int32_t actions_size_;      // Motor command count
    int32_t observations_size_; // Observation vector size  
    int32_t obs_history_length_;  // History window size

    // Normalization parameters
    float clip_obs_;     // Observation clip threshold
    float clip_actions_; // Action clip threshold
    float lin_vel_;      // Linear velocity scale
    float ang_vel_;      // Angular velocity scale  
    float dof_pos_;      // Joint position scale
    float dof_vel_;      // Joint velocity scale
    float height_measurements_; // Height measurement scale

    // Data buffers
    Eigen::Matrix<float, Eigen::Dynamic, 1> proprio_history_buffer_;  // Observation history
    Eigen::Matrix<float, Eigen::Dynamic, 1> tmp_input_;  // Placeholder for observation history

    // Action buffers
    std::vector<float> actions_;       // Raw policy outputs
    std::vector<float> observations_;  // Current observations
    vector_t last_actions_;            // Previous actions

    // System state
    bool is_first_rec_obs_;          // First obs flag
    int64_t loop_count_;             // Cycle counter
    limxsdk::RobotState robot_state_;  // Hardware state

    // FPS calculation variables
    std::chrono::high_resolution_clock::time_point start_time_;  // Start time for FPS calculation
    
    // Timing statistics for individual steps
    float robot_state_time_;     // Time for get_robot_state() in milliseconds
    float obs_compute_time_;     // Time for compute_observation() in milliseconds  
    float action_compute_time_;  // Time for compute_actions() in milliseconds
  };
}

#endif