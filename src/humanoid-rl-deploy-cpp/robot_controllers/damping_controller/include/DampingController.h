/**
 * @file DampingController.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 * 
 * @brief Damping controller for humanoid robots.
 * 
 * Applies damping forces to motor joints to reduce oscillations,
 * absorb shocks, and improve stability during movement or standing.
 */

#ifndef _HUMANOID_DAMPING_CONTROLLER_
#define _HUMANOID_DAMPING_CONTROLLER_

#include "limxsdk/ability/base_ability.h"

namespace humanoid {

/**
 * Damping controller for humanoid robots
 * Applies damping coefficients to motor joints to counteract movement
 * and reduce oscillations.
 */
class DampingController : public limxsdk::ability::BaseAbility {
  protected:
    /**
     * Initialize controller parameters from configuration.
     * 
     * @param config YAML node containing controller configuration
     * @return true if initialization successful, false otherwise
     */
    bool on_init(const YAML::Node& config) override;

    /**
     * Perform startup procedures before entering main control loop.
     * Ensures robot state is valid and initializes internal state.
     */
    void on_start() override;
    
    /**
     * Main control loop for applying damping.
     * Computes and publishes damping torque commands at specified update rate.
     */
    void on_main() override;
  
  private:
    uint32_t update_rate_{1000};        // Control loop update rate (Hz)
    std::vector<float> damping_kd_;    // Damping coefficients for motors (N·m·s/rad)
};
}

#endif
