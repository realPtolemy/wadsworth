// Copyright 2026 Love Mitteregger

#ifndef INCLUDE_ARM_KINEMATICS_H_
#define INCLUDE_ARM_KINEMATICS_H_

#include <array>
#include <cstdint>
#include <expected>
#include <string>

#include "../drivers/sts3215/sts3215.h"

namespace wadsworth::ik {

struct Pose {
	float x;
	float y;
	float z;
	float pitch;
	float roll;
	uint16_t gripper_pos;
};

struct JointConfig {
	uint16_t min_pos;
	uint16_t zero_pos;
	uint16_t max_pos;
};

enum class IkError { kUnreachable, kSingularity, kInvalidPose };

class ArmKinematics {
   public:
	explicit ArmKinematics(servos::Sts3215Driver& driver);
	bool LoadCalibration(const std::string& filepath);
	bool MoveToPose(const Pose& target_pose, uint8_t acceleration, uint16_t speed);
	bool MoveToHome();

   private:
	servos::Sts3215Driver& driver_;

	std::array<JointConfig, 6> joint_configs_;

	static constexpr float kLengthLink1 = 119.00f;
	static constexpr float kLengthLink2 = 111.00f;
	static constexpr float kLengthLink3 = 137.00f;
	static constexpr float kLengthLink4 = 166.00f;

	std::expected<std::array<uint16_t, 6>, IkError> CalculateInverseKinematics(const Pose& pose);

	uint16_t RadiansToServoPosition(float radians, const JointConfig& config);
};

}  // namespace wadsworth::ik

#endif	// INCLUDE_ARM_KINEMATICS_H_
