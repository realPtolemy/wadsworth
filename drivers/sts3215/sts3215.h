// Copyright 2026 Love Mitteregger
// Author: Love Mitteregger
// Date: 2026-FEB-28
//
// About this file:
// It contains the definition of the Sts3215Driver class,
// and the instruction codes and their respective register addresses.

#ifndef DRIVERS_STS3215_STS3215_H_
#define DRIVERS_STS3215_STS3215_H_

#include <cstdint>
#include <span>

#include "../../include/serial_interface.h"

namespace wadsworth::servos {

namespace feetech {

constexpr uint8_t kHeaderByte = 0xFF;
constexpr uint8_t kBroadcastId = 0xFE;

enum class Instruction : uint8_t {
	kPing = 0x01,
	kReadData = 0x02,
	kWriteData = 0x03,
	kRegWrite = 0x04,
	kAction = 0x05,
	kSyncRead = 0x82,
	kSyncWrite = 0x83
};

namespace sts3215 {
// little-endian
// test voltage(V) = 7.4V
// no-load current (mA) = 150mA
// no-load speed (RPM) = 50
// no-load speed (step/second) = 3400
// maximum effective angle (degree) = 360
// resolution 12-bit magnetic encoder (step = 0-4096)
// minimum resoluton alge (degree/step) = 0.087890625
// electronic dead zone (degree) = 0.17578125
// acceleration (degree/s^2) = 8.7890625

constexpr uint16_t kMaxResolution = 4096;

// register map
enum class Register : uint8_t {
	// -- EEPROM (Non-volatile) --
	kId = 0x05,				  // [0-253] 1B: Servo ID
	kBaudRate = 0x06,		  // [0-7] 1B: 0=1M, 1=500k, 2=250k, 3=128k, 4=115200, 5=76800...
	kReturnDelay = 0x07,	  // [0-254] 1B: Response delay (Value * 2us)
	kResponseStatus = 0x08,	  // [0-1] 1B: 0=Reply to READ/PING only, 1=Reply all
	kMinAngleLimit = 0x09,	  // [-32766] 2B: Min angle limit (0 for multi-turn)
	kMaxAngleLimit = 0x0B,	  // [32767] 2B: Max angle limit (0 for multi-turn)
	kMaxTempLimit = 0x0D,	  // [0-100] 1B: Max temp limit (°C)
	kMaxVoltageLimit = 0x0E,  // [0-254] 1B: Max voltage (Value * 0.1V)
	kMinVoltageLimit = 0x0F,  // [0-254] 1B: Min voltage (Value * 0.1V)
	kMaxTorqueLimit = 0x10,	  // [0-1000] 2B: Max EEPROM torque limit (1000 = 100%)
	kOperationMode = 0x21,	  // [0-3] 1B: 0=Position, 1=Speed, 2=PWM, 3=Step

	// -- RAM (Volatile) --
	kTorqueEnable = 0x28,	 // [0-128] 1B: 0=Off, 1=On, 128=Center calibration
	kAcceleration = 0x29,	 // [0-254] 1B: Accel/Decel (Value * 100 steps/s^2)
	kGoalPosition = 0x2A,	 // [-30719~30719] 2B: Target position
	kGoalTime = 0x2C,		 // [0-1000] 2B: PWM open-loop time / direction bit
	kGoalSpeed = 0x2E,		 // [0-3400] 2B: Target speed (steps/s)
	kTorqueLimit = 0x30,	 // [0-1000] 2B: Current RAM torque limit (1000 = 100%)
	kEepromLockFlag = 0x37,	 // [0-1] 1B: 0=EEPROM unlocked, 1=EEPROM locked

	kServoPosition = 0x38,	// [Read Only] 2B: Current position
	kServoSpeed = 0x3A,		// [Read Only] 2B: Current speed (steps/s)
	kServoLoad = 0x3C,		// [Read Only] 2B: Current load (duty cycle)
	kServoVoltage = 0x3E,	// [Read Only] 1B: Current voltage
	kServoTemp = 0x3F,		// [Read Only] 1B: Current temperature (°C)
	kAsyncWrite = 0x40,		// [Read Only] 1B: Async write flag
	kServoStatus = 0x41,	// [Read Only] 1B: Error bits (Volt, Temp, Cur, Angle, Overload)
	kMoveStatus = 0x42,		// [Read Only] 1B: 0=Stopped, 1=Moving
	kServoCurrent = 0x45	// [Read Only] 2B: Current (Value * 6.5mA)
};
}  // namespace sts3215
}  // namespace feetech

struct ServoPosition {
	uint8_t id;
	uint16_t position;
};

struct ServoKinematicTarget {
	uint8_t id;
	uint8_t acceleration;  // [0-254] (Value * 100 steps/s^2)
	uint16_t position;	   // [0-4095]
	uint16_t speed;		   // [0-3400] steps/s
};

class Sts3215Driver {
   public:
	static constexpr size_t kMaxPacketSize = 256;

	// constructor relies on written HAL
	explicit Sts3215Driver(io::SerialInterface& serial_interface);

	// --- high-level servo commands ---
	// send ping to check if servo with ID (0-253) is online
	bool Ping(uint8_t id);

	// turn on/off torque
	bool SetTorqueEnable(uint8_t id, bool enable);

	// calibrate current position as center (360 degree arbitrary)
	bool SetMiddlePosition(uint8_t id);

	// move servo to specific position (0 - 4096 corresponds to 0 - 360 degrees)
	bool SetTargetPosition(uint8_t id, uint16_t position);

	// query multiple servos simultaneously about their position
	bool SyncReadPositions(std::span<const uint8_t> ids, std::span<ServoPosition> out_positions);

	// move multiple servos simultaneously (a broadcast command with no return packet)
	bool SyncWriteTargetPositions(std::span<const ServoPosition> targets);

	// move multiple servos simultaneously with coordinated position, speed, and acceleration
	bool SyncWriteKinematics(std::span<const ServoKinematicTarget> targets);

   private:
	io::SerialInterface& serial_;

	// -- Low level protocol helpers --
	uint8_t CalculateChecksum(uint8_t id, uint8_t length, feetech::Instruction instruction,
							  std::span<const uint8_t> parameters);

	// Builds the byte array and sends it via the serial interface
	void SendPacket(uint8_t id, feetech::Instruction instruction,
					std::span<const uint8_t> parameters);

	// Reads the response packet and validates the checksum. Returns true if valid.
	bool ReceivePacket(uint8_t expected_id, std::span<uint8_t> out_buffer, size_t& out_length);
};

}  // namespace wadsworth::servos

#endif	// DRIVERS_STS3215_STS3215_H_
