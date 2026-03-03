// Copyright 2026 Love Mitteregger
// Author: Love Mittereger
// Date: 2026-FEB-25
//
// About the file:
// This program acts as an HArdware Abstraction Layer (HAL) implementation for Linux.
// Its solely used to translate generic commands such as "send bytes" / "read bytes"
// into specific Linux systems calls.
//
// - It implements the pure virtual functions defined in 'serial_interface.h'.
// - It uses POSIX API to open '/dev/ttyACM0', which represents the CH343P USB-to-UART
// chip on the Waveshare Serial Bus Servo Driver Board Adapter
// - It puts the serial port into "raw" binary mode and sets the hardware to match
// the STS3215 requirements 8N1 requirements...
// (1 Mbps baud rate, 8 data bits, 1 stop bit, no parity bit)

// related headers
#include "linux_serial.h"

// c system headers
#include <fcntl.h>	// file control definitions
#include <poll.h>
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>	  // UNIX standard function definitions

// c++ std libs
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace wadsworth::io {

// constructor
LinuxSerial::LinuxSerial(const char* portName) {
	// open serial for read/write, non-blocking terminal, file descriptor
	fd_ = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd_ < 0) {
		std::cerr << "[LinuxSerial] Error opening port " << portName << ": " << std::strerror(errno)
				  << "\n";
		return;
	}

	struct termios tty;
	if (tcgetattr(fd_, &tty) != 0) {
		std::cerr << "[LinuxSerial] Error from tcgetattr: " << std::strerror(errno) << "\n";
		return;
	}

	// set baud rate: 1 Mbps
	cfsetospeed(&tty, B1000000);
	cfsetispeed(&tty, B1000000);

	// config 8N1 (8 data bits, no parity, 1 stop bit)
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;

	// disable hardware flow control
	tty.c_cflag &= ~CRTSCTS;

	// turn on READ & ignore ctrl lines
	tty.c_cflag |= (CREAD | CLOCAL);

	// make raw read/write (disable special char processing, echo, etc...)
	cfmakeraw(&tty);

	// config for non-block read, rely on poll() for timeouts, read() should always return instantly
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	// apply above settings
	if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
		std::cerr << "[LinuxSerial] Error from tcsetattr: " << std::strerror(errno) << "\n";
		close(fd_);
		fd_ = -1;
	} else {
		std::cout << "[LinuxSerial] Successfully opened port " << portName << " at 1  Mbps!\n";
	}
}

// destructor
LinuxSerial::~LinuxSerial() {
	if (fd_ >= 0) {
		close(fd_);
		std::cout << "[LinuxSerial] Port closed.\n";
	}
}

void LinuxSerial::WriteBytes(const uint8_t* data, size_t length) {
	if (fd_ < 0) return;

	size_t bytes_written = 0;
	while (bytes_written < length) {
		ssize_t result = write(fd_, data + bytes_written, length - bytes_written);

		if (result > 0) {
			bytes_written += static_cast<size_t>(result);
			continue;
		}

		if (result < 0 && (errno == EINTR || errno == EAGAIN)) continue;

		// TODO(Love_Mittereger): Hard error, need to add log here.
		break;
	}
}

size_t LinuxSerial::ReadBytes(uint8_t* buffer, size_t length) {
	if (fd_ < 0) return 0;

	size_t total_bytes_read = 0;
	const int kTimeoutMs = 2;  // NOTE: adjust based on control loop speed

	while (total_bytes_read < length) {
		// serve data from fast internal buffer
		if (rx_head_ < rx_tail_) {
			size_t available = rx_tail_ - rx_head_;
			size_t chunk = std::min(available, length - total_bytes_read);
			std::copy_n(rx_buffer_.begin() + rx_head_, chunk, buffer + total_bytes_read);
			rx_head_ += chunk;
			total_bytes_read += chunk;
			continue;
		}

		// buffer is empty, reset pointers to maximize contiguous space and wait for hardware data
		rx_head_ = 0;
		rx_tail_ = 0;
		struct pollfd fds[1];
		fds[0].fd = fd_;
		fds[0].events = POLLIN;

		int poll_result = poll(fds, 1, kTimeoutMs);
		if (poll_result == 0) break;					  // timeout reached
		if (poll_result < 0 && errno == EINTR) continue;  // poll interrupted by sys signal, retry
		if (poll_result < 0) break;						  // hard os error during poll
		if (!(fds[0].revents & POLLIN)) break;			  // hardware error/disconnect event

		ssize_t read_result = read(fd_, rx_buffer_.data(), rx_buffer_.size());

		if (read_result > 0) {
			rx_tail_ = static_cast<size_t>(read_result);
			continue;  // loop back up to step 1 to copy the new data out
		}

		if (read_result < 0 && (errno == EINTR || errno == EAGAIN)) continue;  // signal interrupt

		// TODO(Love_Mitteregger): Hard error, need to add log here.
		break;
	}
	return total_bytes_read;
}

void LinuxSerial::Flush() {
	if (fd_ >= 0) {
		tcflush(fd_, TCIOFLUSH);
		rx_head_ = 0;
		rx_tail_ = 0;
	}
}

}  // namespace wadsworth::io
