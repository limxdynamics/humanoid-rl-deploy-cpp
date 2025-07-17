/**
 * @file StandController.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Controller for humanoid robot standing behavior.
 * 
 * This class inherits from BaseAbility and manages the robot's transition
 * from its initial state to a standing position, maintaining balance during
 * static standing through PID control.
 */

#ifndef _HUMANOID_STAND_CONTROLLER_
#define _HUMANOID_STAND_CONTROLLER_

#include "limxsdk/ability/base_ability.h"

namespace humanoid {

class StandController : public limxsdk::ability::BaseAbility {
  protected:
    /**
     * Initializes the controller with configuration parameters.
     * @param config YAML node containing control parameters (e.g., gains, positions).
     * @return true if initialization succeeds, false otherwise.
     */
    bool on_init(const YAML::Node& config) override;

    /**
     * Prepares the controller for execution, initializing start positions.
     */
    void on_start() override;
    
    /**
     * Main control loop that executes at the specified update rate,
     * managing the transition to standing and maintaining the pose.
     */
    void on_main() override;
  
  private:
    uint32_t update_rate_{1000};           // Control loop frequency (Hz)
    std::vector<float> stand_kd_;          // Derivative gains for standing PID
    std::vector<float> stand_kp_;          // Proportional gains for standing PID
    std::vector<float> stand_pos_;         // Target joint angles for standing
    std::vector<float> init_joint_angles_; // Initial joint angles at start
    uint64_t transition_steps_{2000};      // Total steps for standing transition
    uint64_t current_step_;                // Current step in transition
    float stand_progress_;                 // Transition progress ratio (0.0-1.0)
};
}

#endif
