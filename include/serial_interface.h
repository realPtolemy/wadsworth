// Copyright 2026 Love Mitteregger
// Author: Love Mitteregger
// Date: 2026-FEB-24
//
// About the file:
// The Abstract Base Pointer Class (Interface) for serial communication.
// Dictates that any system-specific serial port code written must implement
// these exact methods: 'writeBytes', 'readBytes', 'flush'.

#ifndef INCLUDE_SERIAL_INTERFACE_H_
#define INCLUDE_SERIAL_INTERFACE_H_

#include <cstddef>
#include <cstdint>

namespace wadsworth::io {

class SerialInterface {
   public:
	virtual ~SerialInterface() = default;
	virtual void WriteBytes(const uint8_t* data, size_t length) = 0;
	virtual size_t ReadBytes(uint8_t* buffer, size_t length) = 0;
	virtual void Flush() = 0;
};

}  // namespace wadsworth::io

#endif	// INCLUDE_SERIAL_INTERFACE_H_
