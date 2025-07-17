/**
 * @file MimicController.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Neural network-based motion imitation controller for humanoid robots
 * 
 * Implements real-time motion generation using a three-network architecture:
 * - Policy network: Generates motor commands from observations
 * - Linear encoder: Processes motion reference trajectories
 * - Privileged encoder: Processes proprioceptive sensor data
 */

#ifndef _HUMANOID_MIMIC_CONTROLLER_
#define _HUMANOID_MIMIC_CONTROLLER_

#include "limxsdk/ability/base_ability.h"
#include <onnxruntime_cxx_api.h>  // For ONNX model inference
#include <Eigen/Geometry>         // For vector/matrix operations

namespace humanoid
{
  // Eigen vector types for efficient math operations
  using vector3_t = Eigen::Matrix<float, 3, 1>;  // 3D vector (x,y,z)
  using vector_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;  // Dynamic vector

  /**
   * @class MimicController
   * @brief Neural motion controller using ONNX models
   * 
   * Combines three neural networks to transform:
   * 1. Sensor observations → motor commands (policy)
   * 2. Motion references → trajectory features (linear encoder)
   * 3. Proprioceptive data → state features (privileged encoder)
   */
  class MimicController : public limxsdk::ability::BaseAbility
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
     * 1. Process sensor data
     * 2. Run neural network inference
     * 3. Generate motor commands
     */
    void on_main() override;

  protected:
    /**
     * @brief Load ONNX models for specified robot configuration
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
     * @brief Execute linear encoder inference
     * 
     * Processes motion references into trajectory features
     */
    void compute_lin_encoder();

    /**
     * @brief Execute privileged encoder inference
     * 
     * Processes proprioceptive data into state features
     */
    void compute_priv_encoder();

    /**
     * @brief Construct observation vector
     * 
     * Combines: IMU data, joint states, previous actions, and motion phase
     */
    void compute_observation();

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

    // Linear encoder configuration
    std::vector<float> lin_encoder_output_;
    std::unique_ptr<Ort::Session> lin_encoder_session_ptr_;
    std::vector<const char *> lin_encoder_input_names_;
    std::vector<const char *> lin_encoder_output_names_;
    std::vector<std::vector<int64_t>> lin_encoder_input_shapes_;
    std::vector<std::vector<int64_t>> lin_encoder_output_shapes_;

    // Privileged encoder configuration  
    std::vector<float> priv_encoder_output_;
    std::unique_ptr<Ort::Session> priv_encoder_session_ptr_;
    std::vector<const char *> priv_encoder_input_names_;
    std::vector<const char *> priv_encoder_output_names_;
    std::vector<std::vector<int64_t>> priv_encoder_input_shapes_;
    std::vector<std::vector<int64_t>> priv_encoder_output_shapes_;

    // Motion control state
    float motion_phase_{0.};      // Normalized progress [0,1]
    float motion_times_{5.0};     // Total duration (sec)
    int motion_frames_{1000};     // Reference frames
    int motion_iter_{0};          // Current frame
    vector_t motion_cur_ref_;     // Current target
    std::vector<vector_t> motion_refs_;  // Full motion sequence
    vector3_t linear_velocity_estimated_{vector3_t::Zero()};  // Estimated linear velocity (x, y, z)

    // Safety limits
    std::vector<float> user_torque_limit_;  // Per-joint torque caps

    // Control parameters
    std::vector<float> action_scale_;  // Policy output scaling
    std::vector<float> stiffness_;     // Position gains (Kp)
    std::vector<float> damping_;       // Velocity gains (Kd)

    // Network configuration
    std::vector<int32_t> encoder_output_size_;  // Encoder output dims
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

    // Data buffers
    Eigen::Matrix<float, Eigen::Dynamic, 1> proprio_history_buffer_;  // Observation history
    Eigen::Matrix<float, Eigen::Dynamic, 1> encoder_input_;  // Encoder inputs

    // Action buffers
    std::vector<float> actions_;       // Raw policy outputs
    std::vector<float> observations_;  // Current observations
    std::vector<float> action_filtered_;  // Filtered actions

    // System state
    vector_t last_actions_;          // Previous actions
    vector3_t command_filtered_;     // Filtered commands
    bool is_first_rec_obs_;          // First obs flag
    int64_t loop_count_;             // Cycle counter
    limxsdk::RobotState robot_state_;  // Hardware state
  };
}

#endif