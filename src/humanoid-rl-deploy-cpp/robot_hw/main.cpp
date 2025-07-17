/**
 * @file main.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "limxsdk/ability/ability_manager.h"
#include <stdlib.h>
#include <iostream>
#include <signal.h>

// Global ability manager instance
limxsdk::ability::AbilityManager* g_abilityManager = nullptr;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
    
    if (g_abilityManager) {
        std::cout << "Stopping all abilities..." << std::endl;
        g_abilityManager->stopRemoteServer();
    }
    
    // Terminate program
    exit(signum);
}

int main(int argc, char** argv) {
    std::cout << "LIMX SDK Ability Manager" << std::endl;
    std::cout << "------------------------" << std::endl;
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    if (argc > 1) {
        #ifdef WIN32
        _putenv_s("ROBOT_IP", argv[1]);
        #else
        ::setenv("ROBOT_IP", argv[1], 1);
        #endif
    }
    
    // Load configuration
    std::string controllers_file = limxsdk::ability::path::etc() + "/robot_hw/robot_controllers.yaml";
    
    std::cout << "Loading controllers from: " << controllers_file << std::endl;

    // Create ability manager with CLI port 8888
    limxsdk::ability::AbilityManager abilityManager(controllers_file);
    g_abilityManager = &abilityManager;
    
    // Start remote CLI server
    if (!abilityManager.startRemoteServer()) {
        std::cerr << "Failed to start remote CLI server. Exiting..." << std::endl;
        return 1;
    }
    
    // Print usage instructions
    std::cout << "\nConnect to the CLI server using Telnet or Netcat:" << std::endl;
    std::cout << "  $ telnet localhost 8888" << std::endl;
    std::cout << "Type 'help' for available commands." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Keep main thread alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
