/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <chrono>
#include <thread>

#include "Utils/Functions.hpp"
#include "Utils/Classes.hpp"
#include "Utils/AHRS.hpp"
#include "Utils/Sensors.hpp"
#include "ahrs_loc_handler.hpp"
#include "Tests/test_setup.hpp"

int main() {
    std::cout << "Inertial Localization System Example\n";
    std::cout << "====================================\n";
    
    try {
        // Create the localization handler
        AHRSLocHandler locHandler;
        
        // Create a simulated IMU sample
        ImuSample sample;
        sample.time_stamp = 0.0;
        sample.acc_ = {0.0, 0.0, -9.81};
        sample.gyro_ = {0.0, 0.0, 0.0};
        sample.roll_ = 0.0;
        sample.pitch_ = 0.0;
        sample.yaw_ = 0.0;
        
        // Simulate 5 seconds of data
        for (int i = 0; i < 500; i++) {
            // Update the timestamp
            double timestamp = i * 0.01; // 100Hz
            sample.time_stamp = timestamp;
            
            // Update IMU data (simple simulation with small rotation about z-axis)
            sample.gyro_.z = 0.01; // Small rotation rate
            
            // Update the localization system
            locHandler.UpdateIMU(sample, timestamp);
            
            // Simulate vehicle speed
            locHandler.UpdateSpeed(5.0, timestamp); // 5 m/s constant speed
            
            // Print current position and heading every 50 samples
            if (i % 50 == 0) {
                auto position = locHandler.GetPosition();
                auto heading = locHandler.GetVehicleHeading();
                std::cout << "Time: " << timestamp << "s, "
                          << "Position: (" << position[0] << ", " << position[1] << "), "
                          << "Heading: " << heading << " rad\n";
            }
            
            // Sleep to simulate real-time behavior
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "Simulation complete.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
