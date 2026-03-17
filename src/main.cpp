// Copyright 2026 Love Mitteregger
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// Include our custom driver stack
#include "../drivers/linux_serial/linux_serial.h"
#include "../drivers/sts3215/sts3215.h"
#include "../include/arm_calibration.h"	 // Note: Assuming this is calibration.h based on previous steps
#include "../include/arm_kinematics.h"

int main() {
	std::cout << "--- SO-ARM 101 Test Initialization ---\n";

	// 1. Open the hardware port (HAL)
	wadsworth::io::LinuxSerial serial_port("/dev/ttyACM0");

	// 2. Initialize the protocol driver, injecting the serial port
	wadsworth::servos::Sts3215Driver driver(serial_port);

	// 3. Ping the servos
	std::vector<uint8_t> servo_ids = {1, 2, 3, 4, 5, 6};
	for (uint8_t id : servo_ids) {
		std::cout << "Pinging Servo ID " << static_cast<int>(id) << "...\n";
		if (driver.Ping(id)) {
			std::cout << "[SUCCESS] Servo " << static_cast<int>(id)
					  << " is online and responding!\n";
		} else {
			std::cerr << "[ERROR] Servo " << static_cast<int>(id)
					  << " did not respond. Check power, wiring, and ID.\n";
			return 1;  // Halt execution if we can't talk to it
		}
	}

	// 4. Prompt for Calibration Mode
	const std::string config_file = "../cfg/calibration.cfg";
	std::cout << "\nWould you like to run the interactive calibration routine? (y/n): ";
	char choice;
	std::cin >> choice;

	std::cin.ignore(256, '\n');

	if (choice == 'y' || choice == 'Y') {
		// Clear the input buffer before entering the calibration thread
		wadsworth::calibration::RunCalibrationRoutine(driver, config_file);
	}

	// 5. Initialize Kinematics Controller
	wadsworth::ik::ArmKinematics arm(driver);

	// 6. Load the physical constraints
	std::cout << "\nLoading calibration limits...\n";
	if (!arm.LoadCalibration(config_file)) {
		std::cerr << "[FATAL] Calibration missing. Please restart and choose 'y' to calibrate.\n";
		return 1;
	}

	std::cout << "\n--- Arm is Calibrated and Ready! ---\n";

	std::cout << "Moving to geometric Zero Pose in 3 seconds...\n";
	std::cout << "[WARNING] Keep hands clear!\n";

	// Give yourself a moment to step back
	std::this_thread::sleep_for(std::chrono::seconds(3));

	for (uint8_t id : servo_ids) {
		driver.SetTorqueEnable(id, true);
	}

	if (arm.MoveToHome()) {
		std::cout << "Success! Holding Zero Pose.\n";
	} else {
		std::cerr << "Failed to send movement command!\n";
	}

	// Keep the program alive so the arm holds its position
	// (If the program exits, the serial port closes and the script ends)
	std::cout << "Press [ENTER] to exit and relax the arm...\n";
	std::cin.get();
	for (uint8_t id : servo_ids) {
		driver.SetTorqueEnable(id, false);
	}

	return 0;
}
