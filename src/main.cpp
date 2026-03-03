// Copyright 2026 Love Mitteregger
#include <chrono>
#include <iostream>
#include <thread>

// Include our custom driver stack
#include "../drivers/linux_serial/linux_serial.h"
#include "../drivers/sts3215/sts3215.h"

int main() {
	std::cout << "--- SO-ARM 101 Test Initialization ---\n";

	// 1. Open the hardware port (HAL)
	// Make sure your user is in the dialout group and the adapter is plugged in!
	wadsworth::io::LinuxSerial serial_port("/dev/ttyACM0");

	// 2. Initialize the protocol driver, injecting the serial port
	// Because LinuxSerial inherits from SerialInterface, this works automatically!
	wadsworth::servos::Sts3215Driver arm(serial_port);

	// Default ID for Feetech servos out of the box is usually 1.
	// If your arm is fully assembled, the servos might have different IDs (1 through 6).
	const uint8_t test_id = 1;

	// 3. Ping the servo
	std::cout << "Pinging Servo ID " << static_cast<int>(test_id) << "...\n";
	if (arm.Ping(test_id)) {
		std::cout << "[SUCCESS] Servo " << static_cast<int>(test_id)
				  << " is online and responding!\n";
	} else {
		std::cerr << "[ERROR] Servo " << static_cast<int>(test_id)
				  << " did not respond. Check power, wiring, and ID.\n";
		return 1;  // Halt execution if we can't talk to it
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// 4. Move to Center (2048)
	std::cout << "Moving to Center Position (2048)...\n";
	arm.SetTargetPosition(test_id, 2048);
	std::this_thread::sleep_for(std::chrono::seconds(2));

	// 5. Move off-center (2048 + 500 steps)
	// 500 steps * 0.088 degrees/step = ~44 degrees
	std::cout << "Moving off Center Position (2548)...\n";
	std::array<wadsworth::servos::ServoKinematicTarget, 1> targets = {{
		{test_id, 20, 2548, 1000}  // ID 1, Accel 20, Pos 2548, Speed 1000
	}};
	arm.SyncWriteKinematics(targets);
	std::this_thread::sleep_for(std::chrono::seconds(2));

	// std::cout << "Moving to Position 2548...\n";
	// arm.SetTargetPosition(test_id, 2548);
	// std::this_thread::sleep_for(std::chrono::seconds(2));

	// 6. Return to Center
	std::cout << "Returning to Center (2048)...\n";
	arm.SetTargetPosition(test_id, 2048);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout << "Test complete. Shutting down.\n";
	return 0;
}
