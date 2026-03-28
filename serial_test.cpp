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

namespace {

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

}  // namespace

int main(int argc, char** argv) {
	const std::string dev = (argc > 1) ? argv[1] : "/dev/ttyUSB0";

	int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		std::cerr << "open " << dev << " failed: " << strerror(errno) << std::endl;
		return 1;
	}

	if (!ConfigureSerial(fd, B460800)) {
		close(fd);
		return 1;
	}

	std::cout << "serial opened: " << dev << " (115200 8N1 no-flow)" << std::endl;

	while(1) {
        
        std::vector<uint8_t> rx_buf = {0x59, 0x74, 0x01, 0x01, 0x02, 0xED, 0xED};
        ssize_t written_ = write(fd, rx_buf.data(), rx_buf.size());
		if (written_ < 0) {
			std::cerr << "write failed: " << strerror(errno) << std::endl;
		} else {
			tcdrain(fd);
		}

        sleep(1);

        rx_buf = {0x59, 0x74, 0x03, 0x03, 0x06, 0xED, 0xED};
        written_ = write(fd, rx_buf.data(), rx_buf.size());
		if (written_ < 0) {
			std::cerr << "write failed: " << strerror(errno) << std::endl;
		} else {
			tcdrain(fd);
            std::cout << "TX(" << written_ << " bytes): " << rx_buf[0] << " " << rx_buf[1] << " " << rx_buf[2] << std::endl;
		}
        sleep(1);


	}

	close(fd);
	return 0;
}
