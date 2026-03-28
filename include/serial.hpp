#pragma once
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

bool ConfigureSerial(int fd, speed_t baud_rate) {
	termios tty{};
	if (tcgetattr(fd, &tty) != 0) {
		std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
		return false;
	}

	cfsetispeed(&tty, baud_rate);
	cfsetospeed(&tty, baud_rate);

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	tty.c_oflag &= ~OPOST;

	// Non-blocking style read: return immediately if no data.
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;  // 100 ms

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
		return false;
	}

	tcflush(fd, TCIOFLUSH);
	return true;
}

int OpenSerialPort(const std::string& dev, speed_t baud_rate) {
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "open " << dev << " failed: " << strerror(errno) << std::endl;
        return -1;
    }

    if (!ConfigureSerial(fd, baud_rate)) {
        close(fd);
        return -1;
    }

    return fd;
}
