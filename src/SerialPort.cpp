#include "motionsensor/SerialPort.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace motionsensor {

namespace {
speed_t toSpeedConstant(int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::runtime_error("SerialPort: unsupported baud rate " + std::to_string(baudRate));
    }
}
} // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& device, int baudRate) {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        throw std::runtime_error("SerialPort: failed to open '" + device + "': " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        int err = errno;
        close();
        throw std::runtime_error(std::string("SerialPort: tcgetattr failed: ") + std::strerror(err));
    }

    const speed_t speed = toSpeedConstant(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control, raw input/output (chapter "Serial connection").
    cfmakeraw(&tty);
    tty.c_cflag &= ~PARENB; // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // no hw flow control
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; // reads are driven by poll() in read(), not by termios timeouts

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        int err = errno;
        close();
        throw std::runtime_error(std::string("SerialPort: tcsetattr failed: ") + std::strerror(err));
    }

    tcflush(fd_, TCIOFLUSH);

    // Switch back to blocking mode for writes / poll-driven reads.
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialPort::write(const std::vector<uint8_t>& data) {
    size_t written = 0;
    while (written < data.size()) {
        ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("SerialPort: write failed: ") + std::strerror(errno));
        }
        written += static_cast<size_t>(n);
    }
}

size_t SerialPort::read(uint8_t* buffer, size_t bufferSize, int timeoutMs) {
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    int rc = poll(&pfd, 1, timeoutMs);
    if (rc < 0) {
        if (errno == EINTR) return 0;
        throw std::runtime_error(std::string("SerialPort: poll failed: ") + std::strerror(errno));
    }
    if (rc == 0) return 0; // timeout
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("SerialPort: device error/hangup");
    }

    ssize_t n = ::read(fd_, buffer, bufferSize);
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) return 0;
        throw std::runtime_error(std::string("SerialPort: read failed: ") + std::strerror(errno));
    }
    return static_cast<size_t>(n);
}

} // namespace motionsensor
