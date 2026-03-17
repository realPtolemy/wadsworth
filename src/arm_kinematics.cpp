// Copyright 2026 Love Mitteregger

#include "../include/arm_kinematics.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>

#include "sts3215.h"

namespace wadsworth::ik {

ArmKinematics::ArmKinematics(servos::Sts3215Driver& driver) : driver_(driver) {
	for (auto& config : joint_configs_) {
		config = {0, 2048, 4095};
	}
}

bool ArmKinematics::LoadCalibration(const std::string& filepath) {
	std::ifstream infile(filepath);
	if (!infile.is_open()) {
		std::cerr << "[IK ERROR] Could not find calibration file: " << filepath << "\n";
		return false;
	}

	std::string dummy_line;
	std::getline(infile, dummy_line);
	std::getline(infile, dummy_line);

	for (size_t i = 0; i < 6; ++i) {
		int id;
		infile >> id >> joint_configs_[i].min_pos >> joint_configs_[i].zero_pos >>
			joint_configs_[i].max_pos;
		std::cout << "  -> Loaded Servo " << id << " | Zero: " << joint_configs_[i].zero_pos
				  << "\n";
	}

	std::cout << "[IK SUCCESS] Loaded physical joint limits from " << filepath << "\n";
	return true;
}

bool ArmKinematics::MoveToPose(const Pose& target_pose, uint8_t acceleration, uint16_t speed) {
	auto ik_result = CalculateInverseKinematics(target_pose);

	if (!ik_result.has_value()) {
		if (ik_result.error() == IkError::kUnreachable) {
			std::cerr << "[IK ERROR] Target is physically too far away.\n";
		} else if (ik_result.error() == IkError::kSingularity) {
			std::cerr << "[IK ERROR] Target causes arm joint lock-up.\n";
		}
		return false;
	}

	const auto& positions = ik_result.value();

	std::array<servos::ServoKinematicTarget, 6> targets;
	uint8_t id = 1;
	for (size_t i = 0; i < 6; ++i) {
		targets[i] = {id++, acceleration,
					  positions[i],	 // Safely access 0 through 5!
					  speed};
	}
	return driver_.SyncWriteKinematics(targets);
}

bool ArmKinematics::MoveToHome() {
	std::array<servos::ServoKinematicTarget, 6> targets;

	uint8_t id = 1;
	for (size_t i = 0; i < 6; ++i) {
		targets[i] = {id++, 20, joint_configs_[i].zero_pos, 1000};
	}

	return driver_.SyncWriteKinematics(targets);
}

std::expected<std::array<uint16_t, 6>, IkError> ArmKinematics::CalculateInverseKinematics(
	const Pose& pose) {
	std::array<uint16_t, 6> joint_positions;
	std::ranges::fill(joint_positions, 2048);  // init all to center pos

	// 1. solve base rotation
	float theta1 = std::atan2(pose.y, pose.x);

	// 2. solve wrist center in 2D space
	float wrist_radial_dist =
		std::sqrt(pose.x * pose.x + pose.y * pose.y) - (kLengthLink4 * std::cos(pose.pitch));
	float wrist_z = pose.z - (kLengthLink4 * std::sin(pose.pitch));

	// 3. solve shoulder & elbow
	float z_offset = wrist_z - kLengthLink1;
	float distance_squared = (wrist_radial_dist * wrist_radial_dist) + (z_offset * z_offset);
	float max_reach = kLengthLink2 + kLengthLink3;
	if (std::sqrt(distance_squared) > max_reach) {
		return std::unexpected(IkError::kUnreachable);
	}
	// law of cosines for theta3 (elbow pitch)
	float cos_theta3 =
		(distance_squared - (kLengthLink2 * kLengthLink2) - (kLengthLink3 * kLengthLink3)) /
		(2.0f * kLengthLink2 * kLengthLink3);
	cos_theta3 = std::clamp(cos_theta3, -1.0f, 1.0f);
	float theta3 = -std::acos(cos_theta3);
	// law of Cosines for theta2 (shoulder pitch)
	float k1 = kLengthLink2 + kLengthLink3 * std::cos(theta3);
	float k2 = kLengthLink3 * std::sin(theta3);
	float theta2 = std::atan2(z_offset, wrist_radial_dist) - std::atan2(k2, k1);

	// 4. solve wrist pitch
	float theta4 = pose.pitch - (theta2 + theta3);	// theta4 = whatever angle is left over

	// 5. solve wrist roll
	float theta5 = pose.roll;  // independent

	// 6. convert to servo ticks, gripper position is passed straight through
	joint_positions[0] = RadiansToServoPosition(theta1, joint_configs_[0]);
	joint_positions[1] = RadiansToServoPosition(theta2, joint_configs_[1]);
	joint_positions[2] = RadiansToServoPosition(theta3, joint_configs_[2]);
	joint_positions[3] = RadiansToServoPosition(theta4, joint_configs_[3]);
	joint_positions[4] = RadiansToServoPosition(theta5, joint_configs_[4]);
	joint_positions[5] = pose.gripper_pos;

	return joint_positions;
}

uint16_t ArmKinematics::RadiansToServoPosition(float radians, const JointConfig& config) {
	// the STS3215 has a resolution of 4096 steps over 360 degrees (2 * Pi radians).

	// convert radians --> degrees --> to servo steps
	float steps = (radians / (2.0f * std::numbers::pi_v<float>)) * 4096.0f;

	// add the center offset
	int32_t position = config.zero_pos + static_cast<int32_t>(std::round(steps));

	// clamp to hardware limits
	return static_cast<uint16_t>(std::clamp(position, static_cast<int32_t>(config.min_pos),
											static_cast<int32_t>(config.max_pos)));
}

}  // namespace wadsworth::ik
