/**
 * @file MimicController.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Implementation of the MimicController for humanoid robot movement replication.
 * 
 * This file contains the core logic for loading neural network models, processing sensory data,
 * running inference to generate motor commands, and executing mimicry behavior in real-time.
 */

#include "MimicController.h"
#include <string.h>
#include <algorithm> // For std::transform (clipping operations)

namespace humanoid
{
  /**
   * @brief Loads ONNX models (policy, linear encoder, privileged encoder) for the specified robot type.
   * 
   * Initializes ONNX Runtime environment, configures session options, and loads model files.
   * Extracts input/output tensor shapes and names for later inference.
   * 
   * @param robot_type Identifier for the robot model (e.g., "limx50") to load model paths.
   */
  void MimicController::load_model(const std::string& robot_type) 
  {
    // Construct paths to ONNX model files based on robot type
    std::string policy_file = limxsdk::ability::path::etc() + "/mimic_controller/" + robot_type + "/policy/policy.onnx";
    std::string lin_encoder_file = limxsdk::ability::path::etc() + "/mimic_controller/" + robot_type + "/policy/lin_encoder.onnx";
    std::string priv_encoder_file = limxsdk::ability::path::etc() + "/mimic_controller/" + robot_type + "/policy/priv_encoder.onnx";

    // Initialize ONNX Runtime environment (manages logging and resources)
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "MimicControllerOnnx"));

    // Configure ONNX session options (single-threaded execution for determinism)
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetInterOpNumThreads(1);

    Ort::AllocatorWithDefaultOptions allocator; // Allocator for ONNX metadata

    // Load and initialize policy network (generates motor commands from observations)
    std::cout << "load encoder from: " << policy_file << std::endl;
    policy_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policy_file.c_str(), sessionOptions);
    
    // Extract input metadata (names and shapes) for policy network
    policy_input_names_.clear();
    policy_input_shapes_.clear();
    for (std::size_t i = 0; i < policy_session_ptr_->GetInputCount(); i++) 
    {
      policy_input_names_.push_back(policy_session_ptr_->GetInputName(i, allocator));
      policy_input_shapes_.push_back(policy_session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::cout << policy_session_ptr_->GetInputName(i, allocator) << std::endl;
      
      // Print input shape for debugging
      std::vector<int64_t> shape = policy_session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cout << "Shape: [";
      for (size_t j = 0; j < shape.size(); ++j)
      {
        std::cout << shape[j];
        if (j != shape.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    }

    // Extract output metadata (names and shapes) for policy network
    policy_output_names_.clear();
    policy_output_shapes_.clear();
    for (std::size_t i = 0; i < policy_session_ptr_->GetOutputCount(); i++)
    {
      policy_output_names_.push_back(policy_session_ptr_->GetOutputName(i, allocator));
      policy_output_shapes_.push_back(policy_session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::cout << policy_session_ptr_->GetOutputName(i, allocator) << std::endl;
      
      // Print output shape for debugging
      std::vector<int64_t> shape = policy_session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cout << "Shape: [";
      for (size_t j = 0; j < shape.size(); ++j)
      {
        std::cout << shape[j];
        if (j != shape.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    }

    // Load and initialize linear encoder (processes motion references)
    std::cout << "load encoder from: " << lin_encoder_file << std::endl;
    lin_encoder_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, lin_encoder_file.c_str(), sessionOptions);
    
    // Extract input/output metadata for linear encoder
    lin_encoder_input_names_.clear();
    lin_encoder_input_shapes_.clear();
    for (std::size_t j = 0; j < lin_encoder_session_ptr_->GetInputCount(); ++j) 
    {
      lin_encoder_input_names_.push_back(lin_encoder_session_ptr_->GetInputName(j, allocator));
      lin_encoder_input_shapes_.push_back(lin_encoder_session_ptr_->GetInputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }
    lin_encoder_output_names_.clear();
    lin_encoder_output_shapes_.clear();
    for (std::size_t j = 0; j < lin_encoder_session_ptr_->GetOutputCount(); ++j) 
    {
      lin_encoder_output_names_.push_back(lin_encoder_session_ptr_->GetOutputName(j, allocator));
      lin_encoder_output_shapes_.push_back(lin_encoder_session_ptr_->GetOutputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }

    // Load and initialize privileged encoder (processes proprioceptive data)
    std::cout << "load encoder from: " << priv_encoder_file << std::endl;
    priv_encoder_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, priv_encoder_file.c_str(), sessionOptions);
    
    // Extract input/output metadata for privileged encoder
    priv_encoder_input_names_.clear();
    priv_encoder_input_shapes_.clear();
    for (std::size_t j = 0; j < priv_encoder_session_ptr_->GetInputCount(); ++j) 
    {
      priv_encoder_input_names_.push_back(priv_encoder_session_ptr_->GetInputName(j, allocator));
      priv_encoder_input_shapes_.push_back(priv_encoder_session_ptr_->GetInputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }
    priv_encoder_output_names_.clear();
    priv_encoder_output_shapes_.clear();
    for (std::size_t j = 0; j < priv_encoder_session_ptr_->GetOutputCount(); ++j) 
    {
      priv_encoder_output_names_.push_back(priv_encoder_session_ptr_->GetOutputName(j, allocator));
      priv_encoder_output_shapes_.push_back(priv_encoder_session_ptr_->GetOutputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }

    std::cout << "Load Onnx model from successfully !!!" << std::endl;
  }

  /**
   * @brief Loads configuration parameters for the mimic controller from a YAML file.
   * 
   * Loads motion parameters, control gains, normalization scales, and buffer sizes
   * specific to the robot type. Initializes data structures for observations and actions.
   * 
   * @param robot_type Identifier for the robot model to load configuration for.
   */
  void MimicController::load_cfg(const std::string& robot_type) 
  {
    // Load robot-specific parameters from YAML file
    std::string param_file = limxsdk::ability::path::etc() + "/mimic_controller/" + robot_type + "/param.yaml";
    YAML::Node param_node = YAML::LoadFile(param_file);

    // Motion sequence parameters
    motion_frames_ = param_node["motion_frames"].as<int>(); // Total frames in motion reference

    // Control parameters
    action_scale_ = param_node["control"]["action_scale"].as<std::vector<float>>(); // Scales policy outputs to motor commands
    user_torque_limit_ = param_node["control"]["user_torque_limit"].as<std::vector<float>>(); // Safety torque limits
    stiffness_ = param_node["control"]["kp"].as<std::vector<float>>(); // Position control gains (stiffness)
    damping_ = param_node["control"]["kd"].as<std::vector<float>>(); // Velocity control gains (damping)
    decimation_ = param_node["control"]["decimation"].as<int32_t>(); // Downsampling factor for inference

    // Normalization and clipping parameters
    clip_obs_ = param_node["normalization"]["clip_scales"]["clip_observations"].as<float>(); // Observation clipping threshold
    clip_actions_ = param_node["normalization"]["clip_scales"]["clip_actions"].as<float>(); // Action clipping threshold
    lin_vel_ = param_node["normalization"]["obs_scales"]["lin_vel"].as<float>(); // Linear velocity scaling factor
    ang_vel_ = param_node["normalization"]["obs_scales"]["ang_vel"].as<float>(); // Angular velocity scaling factor
    dof_pos_ = param_node["normalization"]["obs_scales"]["dof_pos"].as<float>(); // Joint position scaling factor
    dof_vel_ = param_node["normalization"]["obs_scales"]["dof_vel"].as<float>(); // Joint velocity scaling factor

    // Sizes of data structures
    actions_size_ = param_node["size"]["actions_size"].as<int32_t>(); // Number of motor actions
    observations_size_ = param_node["size"]["observations_size"].as<int32_t>(); // Size of observation vector
    obs_history_length_ = param_node["size"]["obs_history_length"].as<int32_t>(); // Number of past observations to store
    encoder_output_size_ = param_node["size"]["encoder_output_size"].as<std::vector<int32_t>>(); // Output sizes of encoders

    // Initialize buffers
    actions_.resize(actions_size_, 0.); // Raw policy outputs
    action_filtered_.resize(actions_size_, 0.); // Smoothed actions
    observations_.resize(observations_size_, 0.); // Current observation vector

    // Encoder output buffers
    lin_encoder_output_.resize(encoder_output_size_[0], 0.); // Output from linear encoder
    priv_encoder_output_.resize(encoder_output_size_[1], 0.); // Output from privileged encoder

    // State initialization
    last_actions_.resize(actions_size_);
    last_actions_.setZero(); // Previous actions for history
    command_filtered_.setZero(); // Smoothed user commands
  }

  /**
   * @brief Computes the observation vector from robot sensors and state.
   * 
   * Integrates IMU data (orientation, angular velocity), joint positions/velocities,
   * and motion phase into a normalized observation vector. Maintains a history of
   * observations for temporal context in inference.
   */
  void MimicController::compute_observation() 
  {
    // Get IMU data (orientation, gyroscope)
    auto imu_data = get_imu_data();
    Eigen::Quaternionf q_wi; // Orientation quaternion (world to IMU frame)
    q_wi.coeffs()(0) = imu_data.quat[1]; // x component
    q_wi.coeffs()(1) = imu_data.quat[2]; // y component
    q_wi.coeffs()(2) = imu_data.quat[3]; // z component
    q_wi.coeffs()(3) = imu_data.quat[0]; // w component

    // Compute projected gravity vector (used for orientation estimation)
    vector3_t gravity_vector(0, 0, -1); // Gravity in world frame
    vector3_t projected_gravity(q_wi.toRotationMatrix().transpose() * gravity_vector);

    // Extract angular velocity from IMU
    vector3_t base_ang_vel(
      imu_data.gyro[0],  // x component
      imu_data.gyro[1],  // y component
      imu_data.gyro[2]   // z component
    );

    // Extract joint positions and velocities from robot state
    vector_t dof_joint_pos, dof_joint_vel;
    dof_joint_pos.resize(actions_size_);
    dof_joint_vel.resize(actions_size_);
    int count = 0;
    for (std::size_t i = 0; i < actions_size_; ++i) 
    {
      dof_joint_pos(count) = robot_state_.q[i]; // Joint position
      dof_joint_vel(count++) = robot_state_.dq[i]; // Joint velocity
    }

    // Assemble observation vector
    vector_t actions(last_actions_); // Previous actions (for temporal context)
    Eigen::Matrix<float, Eigen::Dynamic, 1> obs(observations_size_);

    // Populate observation segments with normalized data
    obs.segment(0, 3) = base_ang_vel * ang_vel_; // Scaled angular velocity
    obs.segment(3, 3) = projected_gravity; // Projected gravity vector
    obs.segment(6, actions_size_) = dof_joint_pos * dof_pos_; // Scaled joint positions
    obs.segment(6 + actions_size_, actions_size_) = dof_joint_vel * dof_vel_; // Scaled joint velocities
    obs.segment(6 + 2 * actions_size_, actions_size_) = actions; // Previous actions

    // Update motion phase (normalized progress through motion sequence)
    motion_phase_ = static_cast<float>(motion_iter_) / motion_frames_;
    if (motion_phase_ >= 1.0) motion_phase_ = 1.0; // Clamp to 1.0 at end

    // Add motion phase to observation
    obs(6 + 3 * actions_size_) = motion_phase_;

    // Copy Eigen vector to std::vector for ONNX compatibility
    for (std::size_t i = 0; i < observations_size_; i++) 
    {
      observations_[i] = static_cast<float>(obs(i));
    }

    // Initialize observation history buffer on first run
    if (is_first_rec_obs_) 
    {
      proprio_history_buffer_.resize(obs_history_length_ * observations_size_);
      for (std::size_t i = 0; i < obs_history_length_; ++i) 
      {
        proprio_history_buffer_.segment(observations_size_ * i, observations_size_) = obs.head(observations_size_).cast<float>();
      }
      is_first_rec_obs_ = false;
    }

    // Update observation history (shift old data, add new observation)
    encoder_input_ = proprio_history_buffer_;
    for (std::size_t i = 0; i < observations_size_ * (obs_history_length_ - 1); ++i) 
    {
      proprio_history_buffer_[i + observations_size_] = encoder_input_[i]; // Shift left
    }
    proprio_history_buffer_.head(observations_size_) = obs.head(observations_size_).cast<float>(); // Add new observation

    // Clip observations to prevent extreme values
    float obs_min = -clip_obs_;
    float obs_max = clip_obs_;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obs_min, obs_max](float x) { return std::max(obs_min, std::min(obs_max, x)); });
  }

  /**
   * @brief Generates motor actions using the policy network.
   * 
   * Combines current observations, encoder outputs, and filtered commands into
   * an input tensor for the policy network. Runs inference to produce raw actions,
   * which are stored for further processing (filtering, clipping).
   */
  void MimicController::compute_actions() {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    std::vector<float> combined_obs = observations_; // Start with base observations

    // Append filtered commands to input
    combined_obs.push_back(command_filtered_(0));
    combined_obs.push_back(command_filtered_(1));
    combined_obs.push_back(command_filtered_(2));

    // Append encoder outputs to input
    for (const auto& item : lin_encoder_output_) 
      combined_obs.push_back(item);
    for (const auto& item : priv_encoder_output_) 
      combined_obs.push_back(item);

    // Create input tensor for policy network
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      combined_obs.data(), 
      combined_obs.size(),
      policy_input_shapes_[0].data(), 
      policy_input_shapes_[0].size()
    ));

    // Run policy inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = policy_session_ptr_->Run(
      run_options, 
      policy_input_names_.data(), 
      input_values.data(), 
      1,  // Number of inputs
      policy_output_names_.data(), 
      1   // Number of outputs
    );
    // Extract actions from policy output
    for (std::size_t i = 0; i < actions_size_; i++) 
    {
      actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    }
  }

  /**
   * @brief Runs inference on the linear encoder to process motion references.
   * 
   * Uses the observation history buffer as input to the linear encoder network,
   * producing outputs that capture high-level motion features. Results are stored
   * for use in policy inference.
   */
  void MimicController::compute_lin_encoder() 
  {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    
    // Create input tensor from observation history
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      proprio_history_buffer_.data(), 
      proprio_history_buffer_.size(), 
      lin_encoder_input_shapes_[0].data(), 
      lin_encoder_input_shapes_[0].size()
    ));

    // Run linear encoder inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = lin_encoder_session_ptr_->Run(
      run_options, 
      lin_encoder_input_names_.data(),
      input_values.data(), 
      1,  // Number of inputs
      lin_encoder_output_names_.data(), 
      1   // Number of outputs
    );

    // Extract and store encoder outputs (used for policy input)
    for (std::size_t i = 0; i < encoder_output_size_[0]; ++i) 
    {
      lin_encoder_output_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
      linear_velocity_estimated_[i] = lin_encoder_output_[i]; // Update velocity estimate
    }
  }

  /**
   * @brief Runs inference on the privileged encoder to process proprioceptive data.
   * 
   * Uses the observation history buffer as input to the privileged encoder network,
   * capturing low-level sensory features. Results are stored for use in policy inference.
   */
  void MimicController::compute_priv_encoder() 
  {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    
    // Create input tensor from observation history
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      proprio_history_buffer_.data(), 
      proprio_history_buffer_.size(), 
      priv_encoder_input_shapes_[0].data(), 
      priv_encoder_input_shapes_[0].size()
    ));

    // Run privileged encoder inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = priv_encoder_session_ptr_->Run(
      run_options, 
      priv_encoder_input_names_.data(),
      input_values.data(), 
      1,  // Number of inputs
      priv_encoder_output_names_.data(), 
      1   // Number of outputs
    );

    // Extract and store encoder outputs (used for policy input)
    for (std::size_t i = 0; i < encoder_output_size_[1]; ++i) 
    {
      priv_encoder_output_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    }
  }

  /**
   * @brief Initializes the mimic controller during startup.
   * 
   * Loads configuration parameters and ONNX models based on the robot type
   * (from environment variable). Handles errors during initialization.
   * 
   * @param config YAML node with controller setup parameters.
   * @return true if initialization succeeds; false otherwise.
   */
  bool MimicController::on_init(const YAML::Node &config)
  {
    try
    {
      // Set update rate from config if specified
      if (config["update_rate"])
        update_rate_ = config["update_rate"].as<uint32_t>();

      // Get robot type from environment variable
      std::string robot_type;
      const char *robot_type_value = ::getenv("ROBOT_TYPE");
      if (robot_type_value && strlen(robot_type_value) > 0)
        robot_type = std::string(robot_type_value);
      else
      {
        std::cerr << "Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.";
        abort(); // Critical error: robot type is required
      }

      // Load configuration and models
      load_cfg(robot_type);
      load_model(robot_type);
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error on_init: " << e.what() << std::endl;
      return false;
    }
    return true;
  }

  /**
   * @brief Prepares the controller for operation.
   * 
   * Waits for valid robot state data, then initializes buffers and state variables
   * to default values. Resets action/observation vectors and motion tracking variables.
   */
  void MimicController::on_start()
  {
    auto robot_state = get_robot_state();

    // Wait for valid motor state data
    limxsdk::ability::Rate rate(1); // 1 Hz check rate
    while (isRunning() && robot_state.motor_names.size() <= 0)
    {
      std::cout << "Waiting for robot state data" << std::endl;
      robot_state = get_robot_state();
      rate.sleep();
    }

    // Initialize state variables
    is_first_rec_obs_ = true;       // Flag for first observation capture
    loop_count_ = 0;                // Main loop counter
    std::fill(actions_.begin(), actions_.end(), 0.0);           // Reset action buffer
    std::fill(action_filtered_.begin(), action_filtered_.end(), 0.0); // Reset filtered actions
    std::fill(observations_.begin(), observations_.end(), 0.0); // Reset observation buffer
    last_actions_.setZero();        // Reset previous actions
    command_filtered_.setZero();    // Reset filtered commands
    motion_phase_ = 0;              // Reset motion progress
    motion_iter_ = 0;               // Reset motion frame index
  }

  /**
   * @brief Main control loop for the mimic controller.
   * 
   * Runs continuously while the controller is active, updating observations,
   * running inference at a decimated rate, and generating motor commands.
   * Applies safety limits and control gains before publishing commands.
   */
  void MimicController::on_main()
  {
    auto robot = get_robot_instance();
    limxsdk::ability::Rate rate(update_rate_); // Maintain specified update rate

    while (isRunning())
    {
      // Get current robot state (joint positions, velocities, etc.)
      robot_state_ = get_robot_state();

      // Initialize robot command
      limxsdk::RobotCmd robot_cmd(robot_state_.motor_names.size());
      robot_cmd.motor_names = robot_state_.motor_names; // Map commands to motor names

      // Run inference at decimated rate (reduces computational load)
      if (loop_count_ % decimation_ == 0) 
      {
        compute_observation();   // Update sensor observations
        compute_lin_encoder();   // Run linear encoder inference
        compute_priv_encoder();  // Run privileged encoder inference
        compute_actions();       // Generate new actions via policy network

        // Clip actions to safe range
        float action_min = -clip_actions_;
        float action_max = clip_actions_;
        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                       [action_min, action_max](float x) { return std::max(action_min, std::min(action_max, x)); });
        
        // Update history with new actions
        for (int i = 0; i < actions_.size(); ++i) 
          last_actions_(i) = actions_[i];
        
        motion_iter_++; // Advance motion sequence
      }

      // Generate motor commands from actions
      vector_t action_cliped(actions_size_);
      std::size_t action_index = 0;
      const float soft_torque_limit = 0.95; // 95% of hard limit for safety margin

      for (std::size_t i = 0; i < actions_size_; i++) 
      {
        // Calculate safe action bounds based on torque limits
        float action_min = robot_state_.q[i] + (damping_[action_index] * robot_state_.dq[i] -
                                                user_torque_limit_[action_index] * soft_torque_limit) /
                                                stiffness_[action_index];
        float action_max = robot_state_.q[i] + (damping_[action_index] * robot_state_.dq[i] +
                                                user_torque_limit_[action_index] * soft_torque_limit) /
                                                stiffness_[action_index];

        // Clip action to safe bounds and scale to motor command
        action_cliped[action_index] = std::max(action_min / action_scale_[action_index],
                                              std::min(action_max / action_scale_[action_index], (float)actions_[action_index]));
        float pos_des = action_cliped[action_index] * action_scale_[action_index]; // Desired position

        // Populate command with control parameters
        robot_cmd.q[i] = pos_des;       // Target position
        robot_cmd.Kp[i] = stiffness_[action_index]; // Stiffness gain
        robot_cmd.Kd[i] = damping_[action_index];   // Damping gain

        action_index++;
      }

      // Publish motor commands to robot
      robot->publishRobotCmd(robot_cmd);

      loop_count_++;       // Increment loop counter
      rate.sleep();        // Maintain update rate
    }
  }
}

// Register the controller as a LIMX ability for system integration
LIMX_REGISTER_ABILITY(humanoid::MimicController)

