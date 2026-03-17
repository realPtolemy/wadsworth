// Copyright 2026 Love Mitteregger
#include "../include/arm_calibration.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace wadsworth::calibration {

void RunCalibrationRoutine(servos::Sts3215Driver& driver, const std::string& filepath) {
	std::cout << "\n==========================================\n";
	std::cout << "      SO-ARM101 INTERACTIVE CALIBRATION     \n";
	std::cout << "==========================================\n";

	std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};

	// 1. Disable torque so the arm goes limp
	for (uint8_t id : ids) {
		driver.SetTorqueEnable(id, false);
	}
	std::cout << "[INFO] Torque disabled. Arm is now limp.\n\n";

	// Initialize tracking arrays. Min starts at max possible (4095), Max starts at 0.
	std::array<uint16_t, 6> min_limits;
	std::ranges::fill(min_limits, 4095);
	std::array<uint16_t, 6> max_limits;
	std::ranges::fill(max_limits, 0);

	// 2. Start the live-tracking phase
	std::cout << "--- PHASE 1: SWEEP LIMITS ---\n";
	std::cout << "Slowly move every joint to its absolute physical limits.\n";
	std::cout << "Sweep the base, bend the arm fully, open/close the gripper.\n";
	std::cout << "Press [ENTER] when you have swept all joints...\n\n";

	// We use an atomic boolean to safely signal the loop to stop from the background thread
	std::atomic<bool> is_sweeping{true};
	std::thread input_thread([&is_sweeping]() {
		std::cin.get();
		is_sweeping = false;
	});

	std::vector<servos::ServoPosition> current_positions(6);

	// Live polling loop
	while (is_sweeping) {
		if (driver.SyncReadPositions(ids, current_positions)) {
			// Carriage return '\r' overwrites the current terminal line for a clean live UI
			std::cout << "\rLive Tracking: ";
			for (size_t i = 0; i < 6; ++i) {
				uint16_t pos = current_positions[i].position;

				// Expand our recorded min/max boundaries if we see a new extreme
				if (pos < min_limits[i]) min_limits[i] = pos;
				if (pos > max_limits[i]) max_limits[i] = pos;

				std::cout << "ID" << (i + 1) << "[" << min_limits[i] << "-" << max_limits[i]
						  << "] ";
			}
			std::cout << std::flush;
		}
		// Sleep briefly to avoid suffocating the serial bus
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	input_thread.join();  // Clean up the thread

	// 3. Capture the Zero Pose for the Math Geometry
	std::cout << "\n\n--- PHASE 2: CAPTURE ZERO POSE ---\n";
	std::cout << "Hold the arm in the exact mathematical ZERO position:\n";
	std::cout << " - Base: Forward   - Shoulder/Elbow/Wrist Pitch: Straight & Horizontal\n";
	std::cout << " - Roll: Level     - Gripper: Closed\n";
	std::cout << "Press [ENTER] while holding it steady...\n";
	std::cin.get();

	std::array<uint16_t, 6> zero_limits;
	if (driver.SyncReadPositions(ids, current_positions)) {
		for (size_t i = 0; i < 6; ++i) {
			zero_limits[i] = current_positions[i].position;
		}
	} else {
		std::cerr << "[ERROR] Failed to read zero pose. Defaulting to 2048.\n";
		std::ranges::fill(zero_limits, 2048);
	}

	// 4. Generate the Configuration Code
	std::cout << "\n==========================================\n";
	std::cout << "    CALIBRATION COMPLETE! SAVING FILE...    \n";
	std::cout << "==========================================\n\n";

	// Open a file named "calibration.cfg" in writing mode
	std::ofstream outfile(filepath);
	if (outfile.is_open()) {
		outfile << "# SO-ARM101 Calibration Data\n";
		outfile << "# ID Min Zero Max\n";
		for (size_t i = 0; i < 6; ++i) {
			outfile << (i + 1) << " " << min_limits[i] << " " << zero_limits[i] << " "
					<< max_limits[i] << "\n";
		}
		outfile.close();
		std::cout << "[SUCCESS] Calibration saved to '" << filepath << "'!\n\n";
	} else {
		std::cerr << "[ERROR] Could not open '" << filepath << "' for writing!\n\n";
	}
}

}  // namespace wadsworth::calibration
