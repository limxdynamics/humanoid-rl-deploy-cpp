/**
 * @file WalkingController.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Implementation of the WalkingController for humanoid robot walking.
 * 
 * This file contains the core logic for loading neural network models, processing sensory data,
 * running inference to generate motor commands, and executing walking in real-time.
 * Based on the Python implementation with simplified single-policy architecture.
 */

#include "WalkingController.h"
#include <string.h>
#include <algorithm> // For std::transform (clipping operations)
#include <cmath>     // For math functions
#include <chrono>    // For FPS timing
#include <iomanip>   // For std::fixed and std::setprecision
 
 namespace humanoid
 {
   // Global joystick axes for callback access
   std::vector<float> global_joy_axes;
 
   // Initialize joystick data with safe defaults
   void initialize_joystick_data() {
     global_joy_axes.resize(4, 0.0f); // Initialize with 4 axes, all zeros
     std::cout << "Initialized joystick data with " << global_joy_axes.size() << " axes" << std::endl;
   }
 
   // Joystick callback function
   void sensor_joy_callback(const limxsdk::SensorJoyConstPtr& msg) {
     // Safety check for message validity
     if (!msg) {
       std::cerr << "Warning: Received null joystick message" << std::endl;
       return;
     }
     
     global_joy_axes = msg->axes;
   }
   /**
    * @brief Loads ONNX policy model for walking control.
    * 
    * Initializes ONNX Runtime environment, configures session options, and loads policy model.
    * Extracts input/output tensor shapes and names for later inference.
    */
   void WalkingController::load_model(const std::string& robot_type) 
   {
     // Construct path to ONNX policy model file
     std::string policy_file = limxsdk::ability::path::etc() + "/walking_controller/" + robot_type + "/policy/policy.onnx";
 
     // Initialize ONNX Runtime environment (manages logging and resources)
     onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "WalkingControllerOnnx"));
 
     // Configure ONNX session options (single-threaded execution for determinism)
     Ort::SessionOptions sessionOptions;
     sessionOptions.SetIntraOpNumThreads(1);
     sessionOptions.SetInterOpNumThreads(1);
 
     Ort::AllocatorWithDefaultOptions allocator; // Allocator for ONNX metadata
 
     // Load and initialize policy network (generates motor commands from observations)
     std::cout << "Loading policy model from: " << policy_file << std::endl;
     policy_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policy_file.c_str(), sessionOptions);
     
     // Extract input metadata (names and shapes) for policy network
     policy_input_names_.clear();
     policy_input_shapes_.clear();
     for (std::size_t i = 0; i < policy_session_ptr_->GetInputCount(); i++) 
     {
       policy_input_names_.push_back(policy_session_ptr_->GetInputName(i, allocator));
       policy_input_shapes_.push_back(policy_session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
       std::cout << "Policy input: " << policy_session_ptr_->GetInputName(i, allocator) << std::endl;
       
       // Print input shape for debugging
       std::vector<int64_t> shape = policy_session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
       std::cout << "Input shape: [";
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
       std::cout << "Policy output: " << policy_session_ptr_->GetOutputName(i, allocator) << std::endl;
       
       // Print output shape for debugging
       std::vector<int64_t> shape = policy_session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
       std::cout << "Output shape: [";
       for (size_t j = 0; j < shape.size(); ++j)
       {
         std::cout << shape[j];
         if (j != shape.size() - 1) std::cout << ", ";
       }
       std::cout << "]" << std::endl;
     }
 
     std::cout << "Policy model loaded successfully!" << std::endl;
   }
 
   /**
    * @brief Loads configuration parameters for the walking controller from a YAML file.
    * 
    * Loads control gains, normalization scales, and buffer sizes.
    * Initializes data structures for observations and actions.
    */
   void WalkingController::load_cfg(const std::string& robot_type) 
   {
     // Load parameters from YAML file
     std::string param_file = limxsdk::ability::path::etc() + "/walking_controller/" + robot_type + "/param.yaml";
     YAML::Node param_node = YAML::LoadFile(param_file);
     YAML::Node humanoid_cfg = param_node["HumanoidRobotCfg"];
 
     // Control parameters
     action_scale_ = humanoid_cfg["control"]["action_scale"].as<std::vector<float>>(); // Scales policy outputs to motor commands
     user_torque_limit_ = humanoid_cfg["control"]["user_torque_limit"].as<std::vector<float>>(); // Safety torque limits
     stiffness_ = humanoid_cfg["control"]["kp"].as<std::vector<float>>(); // Position control gains (stiffness)
     damping_ = humanoid_cfg["control"]["kd"].as<std::vector<float>>(); // Velocity control gains (damping)
     default_angle_ = humanoid_cfg["control"]["default_angle"].as<std::vector<float>>(); // Default joint angles
     decimation_ = humanoid_cfg["control"]["decimation"].as<int32_t>(); // Downsampling factor for inference
 
     // Joystick velocity limits
     max_vx_ = humanoid_cfg["control"]["max_vx"].as<float>();
     max_vy_ = humanoid_cfg["control"]["max_vy"].as<float>();
     max_vz_ = humanoid_cfg["control"]["max_vz"].as<float>();
 
     // Normalization and clipping parameters
     clip_obs_ = humanoid_cfg["normalization"]["clip_scales"]["clip_observations"].as<float>(); // Observation clipping threshold
     clip_actions_ = humanoid_cfg["normalization"]["clip_scales"]["clip_actions"].as<float>(); // Action clipping threshold
     lin_vel_ = humanoid_cfg["normalization"]["obs_scales"]["lin_vel"].as<float>(); // Linear velocity scaling factor
     ang_vel_ = humanoid_cfg["normalization"]["obs_scales"]["ang_vel"].as<float>(); // Angular velocity scaling factor
     dof_pos_ = humanoid_cfg["normalization"]["obs_scales"]["dof_pos"].as<float>(); // Joint position scaling factor
     dof_vel_ = humanoid_cfg["normalization"]["obs_scales"]["dof_vel"].as<float>(); // Joint velocity scaling factor
     height_measurements_ = humanoid_cfg["normalization"]["obs_scales"]["height_measurements"].as<float>(); // Height measurement scaling
 
     // Sizes of data structures
     actions_size_ = humanoid_cfg["size"]["actions_size"].as<int32_t>(); // Number of motor actions
     observations_size_ = humanoid_cfg["size"]["observations_size"].as<int32_t>(); // Size of observation vector
     obs_history_length_ = humanoid_cfg["size"]["obs_history_length"].as<int32_t>(); // Number of past observations to store
 
     // Gait parameters
     gait_freq_ = humanoid_cfg["size"]["gait_freq"].as<float>();
     gait_offset_ = humanoid_cfg["size"]["gait_offset"].as<float>();
 
     // Initialize buffers
     actions_.resize(actions_size_, 0.); // Raw policy outputs
     observations_.resize(observations_size_, 0.); // Current observation vector
 
     // State initialization
     last_actions_.resize(actions_size_);
     last_actions_.setZero(); // Previous actions for history
     commands_.setZero(); // User commands
   }
 
   /**
    * @brief Compute gait-related observations
    * 
    * Generates gait frequency, offset, height, and phase information
    */
   void WalkingController::compute_gait_observation()
   {
     // Update gait frequency based on forward velocity command
     gait_freq_ = 0.8 + 0.15 * std::abs(commands_(0));
     
     // Update phase using modulo operation
     gait_phase_ = std::fmod(gait_phase_ + 0.01 * gait_freq_, 1.0);
   }
 
 
   /**
    * @brief Update commands from joystick input
    */
   void WalkingController::update_commands_from_joy()
   {
     // Check if joystick data is available and safe to access
     if (global_joy_axes.size() >= 4 && !global_joy_axes.empty()) {
      // Map joystick axes to velocity commands (matching Python implementation)
      commands_(0) = std::max(-max_vx_, std::min(max_vx_, global_joy_axes[1]-0.05f)); // Forward velocity (axis 1) with offset
      commands_(1) = std::max(-max_vy_, std::min(max_vy_, global_joy_axes[0]));       // Lateral velocity (axis 0)
      commands_(2) = std::max(-max_vz_, std::min(max_vz_, global_joy_axes[3]));       // Angular velocity (axis 3)
     } else {
       // Default commands when no joystick data available
       commands_(0) = 0.0; // Forward velocity
       commands_(1) = 0.0; // Lateral velocity  
       commands_(2) = 0.0; // Angular velocity
     }
   }
 
  /**
   * @brief Computes the observation vector from robot sensors and state.
   * 
   * Integrates IMU data (orientation, angular velocity), joint positions/velocities,
   * commands, previous actions, and gait observations into a normalized observation vector.
   */
  void WalkingController::compute_observation() 
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
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
     for (std::size_t i = 0; i < actions_size_; ++i) 
     {
       dof_joint_pos(i) = robot_state_.q[i] - default_angle_[i]; // Joint position
       dof_joint_vel(i) = robot_state_.dq[i]; // Joint velocity
     }
 
     // Get previous actions
     vector_t actions(last_actions_);
 
     // Compute gait observations
     compute_gait_observation();
     vector_t gait_obs(5);
     gait_obs(0) = gait_freq_;                                    // gait frequency
     gait_obs(1) = gait_offset_;                                  // gait offset
     gait_obs(2) = gait_height_;                                  // gait height (fixed value)
     gait_obs(3) = std::sin(2 * M_PI * gait_phase_);             // phase sine value
     gait_obs(4) = std::cos(2 * M_PI * gait_phase_);             // phase cosine value
 
     // Assemble observation vector
     Eigen::Matrix<float, Eigen::Dynamic, 1> obs(observations_size_);
     int idx = 0;
 
     // Populate observation segments with normalized data
     obs.segment(idx, 3) = base_ang_vel * ang_vel_; idx += 3;           // Scaled angular velocity
     obs.segment(idx, 3) = projected_gravity; idx += 3;                 // Projected gravity vector
     obs.segment(idx, 3) = commands_; idx += 3;                         // Command inputs
     obs.segment(idx, actions_size_) = dof_joint_pos * dof_pos_; idx += actions_size_; // Scaled joint positions
     obs.segment(idx, actions_size_) = dof_joint_vel * dof_vel_; idx += actions_size_; // Scaled joint velocities
     obs.segment(idx, actions_size_) = actions; idx += actions_size_;   // Previous actions
    //  obs.segment(idx, 5) = gait_obs; idx += 5;                         // Gait observations
 
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
    tmp_input_ = proprio_history_buffer_;
    for (std::size_t i = 0; i < observations_size_ * (obs_history_length_ - 1); ++i) 
    {
      proprio_history_buffer_[i + observations_size_] = tmp_input_[i]; // Shift left
    }
    proprio_history_buffer_.head(observations_size_) = obs.head(observations_size_).cast<float>(); // Add new observation

       // Clip observations to prevent extreme values
       float obs_min = -clip_obs_;
       float obs_max = clip_obs_;
       std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                      [obs_min, obs_max](float x) { return std::max(obs_min, std::min(obs_max, x)); });
     } catch (const std::exception& e) {
       std::cerr << "Error in compute_observation: " << e.what() << std::endl;
       // Fill with zeros as fallback
       std::fill(observations_.begin(), observations_.end(), 0.0f);
     }
     
     // Record timing for compute_observation
     auto end_time = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
     obs_compute_time_ = duration.count() / 1000.0f; // Convert to milliseconds
   }
 
  /**
   * @brief Generates motor actions using the policy network.
   * 
   * Uses proprioceptive history buffer as input to the policy network.
   * Runs inference to produce raw actions, which are stored for further processing.
   */
  void WalkingController::compute_actions() {
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
      // Create memory buffer for ONNX tensor
      Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
      std::vector<Ort::Value> input_values;

    // Create input tensor for policy network using proprio_history_buffer_ (add batch dimension)
    std::vector<int64_t> input_shape = policy_input_shapes_[0];
    if (input_shape[0] == -1) {
      input_shape[0] = 1; // Set batch size to 1
    }

    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      proprio_history_buffer_.data(), 
      proprio_history_buffer_.size(),
      input_shape.data(), 
      input_shape.size()
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
       
       // Calculate and print timing statistics every 100 inferences
       if (loop_count_ % (decimation_ * 100) == 0) {
         auto current_time = std::chrono::high_resolution_clock::now();
         auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
         float fps = (loop_count_) * 1000.0f / elapsed_time.count();
         std::cout << "=== 性能统计 ===" << std::endl;
         std::cout << "总体FPS: " << std::fixed << std::setprecision(2) << fps << " Hz" << std::endl;
         std::cout << "get_robot_state: " << std::fixed << std::setprecision(3) << robot_state_time_ << " ms" << std::endl;
         std::cout << "compute_observation: " << std::fixed << std::setprecision(3) << obs_compute_time_ << " ms" << std::endl;
         std::cout << "compute_actions: " << std::fixed << std::setprecision(3) << action_compute_time_ << " ms" << std::endl;
         std::cout << "总计算时间: " << std::fixed << std::setprecision(3) 
                   << (robot_state_time_ + obs_compute_time_ + action_compute_time_) << " ms" << std::endl;
         std::cout << "=================" << std::endl;
       }
     } catch (const std::exception& e) {
       std::cerr << "Error in compute_actions: " << e.what() << std::endl;
       // Fill with zeros as fallback
       std::fill(actions_.begin(), actions_.end(), 0.0f);
     }
     
     // Record timing for compute_actions
     auto end_time = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
     action_compute_time_ = duration.count() / 1000.0f; // Convert to milliseconds
   }
 
 
   /**
    * @brief Initializes the walking controller during startup.
    * 
    * Loads configuration parameters and ONNX models.
    * Handles errors during initialization.
    * 
    * @param config YAML node with controller setup parameters.
    * @return true if initialization succeeds; false otherwise.
    */
   bool WalkingController::on_init(const YAML::Node &config)
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
         std::cerr << "Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'." << std::endl;
         abort(); // Critical error: robot type is required
       }
 
       // Load configuration and models
       load_cfg(robot_type);
       load_model(robot_type);
 
       // Initialize joystick data (subscription will be done in on_start)
       initialize_joystick_data();
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
    * to default values. Resets action/observation vectors and gait tracking variables.
    */
   void WalkingController::on_start()
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
    std::fill(observations_.begin(), observations_.end(), 0.0); // Reset observation buffer
    last_actions_.setZero();        // Reset previous actions
    commands_.setZero();            // Reset commands
    gait_phase_ = 0;                // Reset gait phase
    
    // Initialize FPS calculation start time
    start_time_ = std::chrono::high_resolution_clock::now();
    
    // Initialize timing statistics
    robot_state_time_ = 0.0f;
    obs_compute_time_ = 0.0f;
    action_compute_time_ = 0.0f;
     
     // Subscribe to joystick sensor data (following RobotJoystick pattern)
     auto robot = get_robot_instance();
     if (robot) {
       robot->subscribeSensorJoy(sensor_joy_callback);
     }
   }
 
   /**
    * @brief Main control loop for the walking controller.
    * 
    * Runs continuously while the controller is active, updating observations,
    * running inference at a decimated rate, and generating motor commands.
    * Applies safety limits and control gains before publishing commands.
    */
   void WalkingController::on_main()
   {
     auto robot = get_robot_instance();
     limxsdk::ability::Rate rate(update_rate_); // Maintain specified update rate
     
 
     while (isRunning())
     {
       try {
         // Update commands from joystick input
         update_commands_from_joy();
         
         // Get current robot state (joint positions, velocities, etc.)
         auto robot_state_start = std::chrono::high_resolution_clock::now();
         robot_state_ = get_robot_state();
         auto robot_state_end = std::chrono::high_resolution_clock::now();
         auto robot_state_duration = std::chrono::duration_cast<std::chrono::microseconds>(robot_state_end - robot_state_start);
         robot_state_time_ = robot_state_duration.count() / 1000.0f; // Convert to milliseconds
 
       // Initialize robot command
       limxsdk::RobotCmd robot_cmd(robot_state_.motor_names.size());
       robot_cmd.motor_names = robot_state_.motor_names; // Map commands to motor names
 
       // Run inference at decimated rate (reduces computational load)
       if (loop_count_ % decimation_ == 0) 
       {
         compute_observation();   // Update sensor observations
         compute_actions();       // Generate new actions via policy network
 
         // Clip actions to safe range
         float action_min = -clip_actions_;
         float action_max = clip_actions_;
         std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                        [action_min, action_max](float x) { return std::max(action_min, std::min(action_max, x)); });
         
         // Update history with new actions
         for (int i = 0; i < actions_.size(); ++i) 
           last_actions_(i) = actions_[i];
       }
 
       // Generate motor commands from actions
       const float soft_torque_limit = 0.95; // 95% of hard limit for safety margin
 
       for (std::size_t i = 0; i < actions_size_; i++) 
       {
         // Calculate safe action bounds based on torque limits and default angles
         float action_min = (robot_state_.q[i] - default_angle_[i] + 
                            (damping_[i] * robot_state_.dq[i] - user_torque_limit_[i] * soft_torque_limit) / stiffness_[i]);
         float action_max = (robot_state_.q[i] - default_angle_[i] + 
                            (damping_[i] * robot_state_.dq[i] + user_torque_limit_[i] * soft_torque_limit) / stiffness_[i]);
 
         // Clip action to safe bounds and scale to motor command
         actions_[i] = std::max(action_min / action_scale_[i], 
                               std::min(action_max / action_scale_[i], actions_[i]));
         float pos_des = actions_[i] * action_scale_[i] + default_angle_[i]; // Desired position
 
         // Populate command with control parameters
         robot_cmd.q[i] = pos_des;       // Target position
         robot_cmd.dq[i] = 0;            // Target velocity
         robot_cmd.tau[i] = 0;           // Target torque
         robot_cmd.Kp[i] = stiffness_[i]; // Stiffness gain
         robot_cmd.Kd[i] = damping_[i];   // Damping gain
       }
 
         // Publish motor commands to robot
         robot->publishRobotCmd(robot_cmd);
 
         loop_count_++;       // Increment loop counter
         rate.sleep();        // Maintain update rate
         
         // Debug: Print status every 1000 iterations
         if (loop_count_ % 1000 == 0) {
           std::cout << "Main loop iteration: " << loop_count_ 
                     << ", commands: [" << commands_(0) << ", " << commands_(1) << ", " << commands_(2) << "]"
                     << std::endl;
         }
       } catch (const std::exception& e) {
         std::cerr << "Error in main loop iteration " << loop_count_ << ": " << e.what() << std::endl;
         // Continue running despite error
         rate.sleep();
       }
     }
   }
 }
 
 // Register the controller as a LIMX ability for system integration
 LIMX_REGISTER_ABILITY(humanoid::WalkingController)
 
 