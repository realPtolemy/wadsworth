// Copyright 2026 Love Mitteregger
// Author: Love Mitteregger
// Date: 2026-FEB-25
//
// About this file:
// This file declares the linux-specific class that implements the defined
// serial communications interface ('serial_interface.h')

#ifndef DRIVERS_LINUX_SERIAL_LINUX_SERIAL_H_
#define DRIVERS_LINUX_SERIAL_LINUX_SERIAL_H_

#include <cstddef>
#include <cstdint>

#include "../../include/serial_interface.h"

namespace wadsworth::io {

class LinuxSerial : public SerialInterface {
   public:
	explicit LinuxSerial(const char* portname);
	~LinuxSerial() override;  // close the port

	void WriteBytes(const uint8_t* data, size_t length) override;
	size_t ReadBytes(uint8_t* buffer, size_t length) override;
	void Flush() override;

   private:
	int fd_;  // linux file descriptor for the serial descriptor
};

}  // namespace wadsworth::io

#endif	// DRIVERS_LINUX_SERIAL_LINUX_SERIAL_H_
