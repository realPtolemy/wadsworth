// Copyright 2026 Love Mitteregger

#ifndef INCLUDE_ARM_CALIBRATION_H_
#define INCLUDE_ARM_CALIBRATION_H_

#include <string>

#include "../drivers/sts3215/sts3215.h"

namespace wadsworth::calibration {

void RunCalibrationRoutine(servos::Sts3215Driver& driver,
						   const std::string& filepath = "../cfg/calibration.cfg");

}  // namespace wadsworth::calibration

#endif	// INCLUDE_ARM_CALIBRATION_H_
