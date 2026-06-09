// Copyright 2026 Love Mitteregger
//
// Joint-angle demo for the real SO-ARM101.
//
// Phase 1: Move arm to pose A (shoulder_lift=-1.05, elbow_flex=0.659,
//          wrist_flex=1.14, gripper=0.603), then sweep wrist_roll from
//          its minimum to its maximum calibrated position.
// Phase 2: Reset to pose B (shoulder_lift=-1.70, elbow_flex=1.35,
//          wrist_flex=0.647, gripper=0.2).
//
// Build:  ninja -C build hardware_demo
// Run:    ./build/hardware_demo

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include "../drivers/linux_serial/linux_serial.h"
#include "../drivers/sts3215/sts3215.h"

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

struct JointCalib {
	uint16_t min_pos;
	uint16_t zero_pos;
	uint16_t max_pos;
};

static bool LoadCalibration(const std::string& path, std::array<JointCalib, 6>& out) {
	std::ifstream f(path);
	if (!f.is_open()) {
		std::cerr << "[FATAL] Cannot open calibration file: " << path << "\n";
		return false;
	}
	std::string line;
	std::getline(f, line);
	std::getline(f, line);
	for (size_t i = 0; i < 6; ++i) {
		int id, mn, zp, mx;
		f >> id >> mn >> zp >> mx;
		out[i] = {static_cast<uint16_t>(mn), static_cast<uint16_t>(zp), static_cast<uint16_t>(mx)};
	}
	return true;
}

// Convert radians to servo ticks, clamped to calibrated joint limits.
static uint16_t ToTicks(float radians, const JointCalib& c) {
	float steps = (radians / (2.0f * std::numbers::pi_v<float>)) * 4096.0f;
	int32_t pos = c.zero_pos + static_cast<int32_t>(std::round(steps));
	return static_cast<uint16_t>(
		std::clamp(pos, static_cast<int32_t>(c.min_pos), static_cast<int32_t>(c.max_pos)));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build a SyncWriteKinematics target array from per-joint tick positions.
static std::array<wadsworth::servos::ServoKinematicTarget, 6> MakeTargets(
	const std::array<uint16_t, 6>& ticks, uint8_t accel, uint16_t speed) {
	std::array<wadsworth::servos::ServoKinematicTarget, 6> targets;
	for (size_t i = 0; i < 6; ++i) {
		targets[i] = {static_cast<uint8_t>(i + 1), accel, ticks[i], speed};
	}
	return targets;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
	wadsworth::io::LinuxSerial serial("/dev/ttyACM0");
	wadsworth::servos::Sts3215Driver driver(serial);

	static const std::vector<uint8_t> kServoIds = {1, 2, 3, 4, 5, 6};

	std::array<JointCalib, 6> calib;
	if (!LoadCalibration("../cfg/calibration.cfg", calib)) {
		return 1;
	}

	// Enable torque
	for (uint8_t id : kServoIds) {
		driver.SetTorqueEnable(id, true);
	}

	std::cout << "\nSO-ARM101 Joint Demo\n";
	std::cout << "WARNING: Keep hands clear. Moving in 3 seconds...\n\n";
	std::this_thread::sleep_for(std::chrono::seconds(3));

	// -----------------------------------------------------------------------
	// Phase 1: Move to pose A, then sweep wrist roll min → max
	// Joint order: pan, lift, elbow, wrist_flex, wrist_roll, gripper
	// -----------------------------------------------------------------------

	// All values in radians (from MuJoCo). shoulder_pan=0 (straight ahead).
	static constexpr float kPoseA_Pan = 0.000f;
	static constexpr float kPoseA_Lift = -1.050f;
	static constexpr float kPoseA_Elbow = 0.659f;
	static constexpr float kPoseA_WristFlex = 1.140f;
	static constexpr float kPoseA_Gripper = 0.603f;

	std::array<uint16_t, 6> ticks_a = {
		ToTicks(kPoseA_Pan, calib[0]),
		ToTicks(kPoseA_Lift, calib[1]),
		ToTicks(kPoseA_Elbow, calib[2]),
		ToTicks(kPoseA_WristFlex, calib[3]),
		calib[4].min_pos,  // wrist_roll starts at min
		ToTicks(kPoseA_Gripper, calib[5]),
	};

	std::cout << "Phase 1: moving to pose A...\n";
	driver.SyncWriteKinematics(MakeTargets(ticks_a, 20, 500));
	std::this_thread::sleep_for(std::chrono::seconds(3));

	// Sweep wrist_roll from min to max in 30 steps
	std::cout << "Phase 1: sweeping wrist roll min → max...\n";
	static constexpr int kRollSteps = 30;
	auto roll_ticks_a = ticks_a;
	for (int step = 0; step <= kRollSteps; ++step) {
		float t = static_cast<float>(step) / static_cast<float>(kRollSteps);
		roll_ticks_a[4] = static_cast<uint16_t>(
			calib[4].min_pos + std::round(t * (calib[4].max_pos - calib[4].min_pos)));
		driver.SyncWriteKinematics(MakeTargets(roll_ticks_a, 10, 200));
		std::this_thread::sleep_for(std::chrono::milliseconds(80));
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// -----------------------------------------------------------------------
	// Phase 2: Reset to pose B
	// -----------------------------------------------------------------------

	static constexpr float kPoseB_Pan = 0.000f;
	static constexpr float kPoseB_Lift = -1.700f;
	static constexpr float kPoseB_Elbow = 1.350f;
	static constexpr float kPoseB_WristFlex = 0.647f;
	static constexpr float kPoseB_WristRoll = 0.000f;
	static constexpr float kPoseB_Gripper = 0.200f;

	std::array<uint16_t, 6> ticks_b = {
		ToTicks(kPoseB_Pan, calib[0]),		 ToTicks(kPoseB_Lift, calib[1]),
		ToTicks(kPoseB_Elbow, calib[2]),	 ToTicks(kPoseB_WristFlex, calib[3]),
		ToTicks(kPoseB_WristRoll, calib[4]), ToTicks(kPoseB_Gripper, calib[5]),
	};

	std::cout << "Phase 2: resetting to pose B...\n";
	driver.SyncWriteKinematics(MakeTargets(ticks_b, 20, 500));
	std::this_thread::sleep_for(std::chrono::seconds(3));

	std::cout << "\nDone. Press [ENTER] to release torque and exit...\n";
	std::cin.get();
	for (uint8_t id : kServoIds) {
		driver.SetTorqueEnable(id, false);
	}

	return 0;
}
