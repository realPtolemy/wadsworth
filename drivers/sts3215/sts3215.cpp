// Copyright 2026 Love Mitteregger
#include "sts3215.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <numeric>
#include <span>
#include <vector>

namespace wadsworth::servos {

// constructor
Sts3215Driver::Sts3215Driver(io::SerialInterface& serial_interface) : serial_(serial_interface) {}

// ----------------------------------------------------------------------------------------
// low-level protocol helper functions
// ----------------------------------------------------------------------------------------
uint8_t Sts3215Driver::CalculateChecksum(uint8_t id, uint8_t length,
										 feetech::Instruction instruction,
										 std::span<const uint8_t> parameters) {
	// feetech (dynamixel) checksum formula: ~(id + length + instruction + param1 + ... + paramn)
	uint32_t sum = id + length + static_cast<uint8_t>(instruction);
	sum = std::accumulate(parameters.begin(), parameters.end(), sum);
	return static_cast<uint8_t>(~sum);
}

void Sts3215Driver::SendPacket(uint8_t id, feetech::Instruction instruction,
							   std::span<const uint8_t> parameters) {
	const size_t parameters_size = parameters.size();
	if (parameters_size + 6 > kMaxPacketSize) return;

	serial_.Flush();  // clear receive buffer to prevent read of stale data

	std::array<uint8_t, kMaxPacketSize> buffer;
	uint8_t length = static_cast<uint8_t>(parameters_size + 2);

	buffer[0] = feetech::kHeaderByte;
	buffer[1] = feetech::kHeaderByte;
	buffer[2] = id;
	buffer[3] = length;
	buffer[4] = static_cast<uint8_t>(instruction);

	std::copy(parameters.begin(), parameters.end(), buffer.begin() + 5);
	buffer[5 + parameters_size] = CalculateChecksum(id, length, instruction, parameters);

	serial_.WriteBytes(buffer.data(), parameters_size + 6);
}

bool Sts3215Driver::ReceivePacket(uint8_t expected_id, std::vector<uint8_t>& out_parameters) {
	// loop check for 0xFF 0xFF header (so that servo bypasses half-duplex TX/RX switching noise)
	uint8_t ff_count = 0, byte = 0;
	while (ff_count < 2 && serial_.ReadBytes(&byte, 1) == 1) {
		ff_count = (byte == feetech::kHeaderByte) ? ff_count + 1 : 0;
	}
	if (ff_count < 2) return false;

	// read id and length
	uint8_t id_len[2];	// id_len[1] < 2 check because servo holds 1 status byte + 1 checksum byte
	if (serial_.ReadBytes(id_len, 2) != 2 || id_len[0] != expected_id || id_len[1] < 2) {
		serial_.Flush();
		return false;
	}

	// read payload (status+parameters+checksum)
	const uint8_t length = id_len[1];
	std::vector<uint8_t> payload(length);
	if (serial_.ReadBytes(payload.data(), length) != length) return false;

	// validate checksum
	uint32_t sum = expected_id + length;
	sum = std::accumulate(payload.begin(), payload.end() - 1, sum);
	if (static_cast<uint8_t>(~sum) != payload.back()) {
		serial_.Flush();
		return false;  // TODO(Love Mitteregger): add error log for corrupted packet
	}

	// validate that payload is error free
	if (payload[0] != 0x00) {
		serial_.Flush();
		return false;
		// TODO(Love Mitteregger): add error log here for overheated/overloaded servo etc...
		// Bit 0: Voltage | Bit 1: Angle Limit | Bit 2: Overheat | Bit 3: Range
		// Bit 4: Checksum | Bit 5: Overload | Bit 6: Instruction
	}

	// extract parameters from payload
	out_parameters.assign(payload.begin() + 1, payload.end() - 1);
	return true;
}

// ----------------------------------------------------------------------------------------
// high-level servo commands
// ----------------------------------------------------------------------------------------
bool Sts3215Driver::Ping(uint8_t id) {
	SendPacket(id, feetech::Instruction::kPing, {});
	std::vector<uint8_t> rx_parameters;
	return ReceivePacket(id, rx_parameters);
}

bool Sts3215Driver::SetMiddlePosition(uint8_t id) {
	// Write 128 (0x80) to the Torque Enable address to calibrate center
	const std::array<uint8_t, 2> parameters = {
		static_cast<uint8_t>(feetech::sts3215::Register::kTorqueEnable), 128};

	SendPacket(id, feetech::Instruction::kWriteData, parameters);
	std::vector<uint8_t> rx_parameters;
	return ReceivePacket(id, rx_parameters);
}

bool Sts3215Driver::SetTargetPosition(uint8_t id, uint16_t position) {
	if (position >= feetech::sts3215::kMaxResolution) {
		position = feetech::sts3215::kMaxResolution - 1;
	}

	// STS3215 uses Little-Endian: low byte first, then high byte
	const uint8_t low_byte = static_cast<uint8_t>(position & 0xFF);
	const uint8_t high_byte = static_cast<uint8_t>((position >> 8) & 0xFF);

	const std::array<uint8_t, 3> parameters = {
		static_cast<uint8_t>(feetech::sts3215::Register::kGoalPosition), low_byte, high_byte};

	SendPacket(id, feetech::Instruction::kWriteData, parameters);
	std::vector<uint8_t> rx_parameters;

	return ReceivePacket(id, rx_parameters);
}

bool Sts3215Driver::SyncReadPositions(std::span<const uint8_t> ids,
									  std::span<ServoPosition> out_positions) {
	if (ids.empty() || ids.size() != out_positions.size()) return false;

	// packet the parameters
	// 1 byte starting address + 1 byte data length + 1 byte per request ID
	const size_t num_parameters = 2 + ids.size();
	if (num_parameters > kMaxPacketSize) return false;
	std::array<uint8_t, kMaxPacketSize> parameters;
	parameters[0] = static_cast<uint8_t>(feetech::sts3215::Register::kServoPosition);
	parameters[1] = 2;	// data lenght per servo (16 bits, a low byte and a high byte)
	size_t idx = 2;
	for (const uint8_t id : ids) {
		parameters[idx++] = id;
	}

	// send broadcast request
	SendPacket(feetech::kBroadcastId, feetech::Instruction::kSyncRead,
			   std::span{parameters.data(), num_parameters});

	// catch cascading replies
	std::vector<uint8_t> rx_parameters;
	for (size_t i = 0; i < ids.size(); ++i) {
		if (!ReceivePacket(ids[i], rx_parameters) || rx_parameters.size() < 2) return false;

		// reconstruct the 16-bit integer from the little-endian response (low, then high byte)
		const uint16_t position = rx_parameters[0] | (static_cast<uint16_t>(rx_parameters[1]) << 8);
		out_positions[i].id = ids[i];
		out_positions[i].position = position;
	}

	return true;
}

bool Sts3215Driver::SyncWriteTargetPositions(std::span<const ServoPosition> targets) {
	if (targets.empty()) return true;

	const size_t num_parameters = 2 + (targets.size() * 3);
	if (num_parameters > kMaxPacketSize) return false;

	std::array<uint8_t, kMaxPacketSize> parameters;
	parameters[0] = static_cast<uint8_t>(feetech::sts3215::Register::kGoalPosition);
	parameters[1] = 2;	// data length per servo (16 bits, a low byte and a high byte)

	size_t idx = 2;
	for (const auto& target : targets) {
		uint16_t position = target.position;
		if (position >= feetech::sts3215::kMaxResolution) {
			position = feetech::sts3215::kMaxResolution - 1;
		}

		parameters[idx++] = target.id;
		parameters[idx++] = static_cast<uint8_t>(position & 0xFF);
		parameters[idx++] = static_cast<uint8_t>((position >> 8) & 0xFF);
	}
	SendPacket(feetech::kBroadcastId, feetech::Instruction::kSyncWrite,
			   std::span{parameters.data(), idx});

	return true;
}

}  // namespace wadsworth::servos
