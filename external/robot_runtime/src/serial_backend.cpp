#include "parol6/serial_backend.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include "parol6/serial_protocol.hpp"

namespace parol6 {
namespace {
std::array<bool, 2> startupOutputs(const config::SerialConfig &cfg) {
    if (!cfg.startup_outputs)
        throw std::invalid_argument("serial.startup_outputs must specify both electrical output levels");
    return *cfg.startup_outputs;
}
} // namespace

Parol6SerialBackend::Parol6SerialBackend(const config::SerialConfig &cfg)
    : device_(cfg.device), startup_outputs_(startupOutputs(cfg)), telemetry_timeout_seconds_(cfg.telemetry_timeout_seconds),
      pneumatic_open_high_(cfg.pneumatic_open_high) {
    if (!std::isfinite(cfg.transmission_timeout_seconds) || cfg.transmission_timeout_seconds <= 0 || cfg.transmission_timeout_seconds > 60)
        throw std::invalid_argument("invalid serial transmission deadline");
    transmission_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(cfg.transmission_timeout_seconds));
    if (!std::isfinite(telemetry_timeout_seconds_) || telemetry_timeout_seconds_ <= 0 || telemetry_timeout_seconds_ > 60)
        throw std::invalid_argument("telemetry timeout must be finite and positive");
    if (!std::isfinite(cfg.first_response_timeout_seconds) || cfg.first_response_timeout_seconds <= 0 || cfg.first_response_timeout_seconds > 60)
        throw std::invalid_argument("first-response timeout must be within (0,60] seconds");
    fd_.value = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
        throw std::system_error(errno, std::generic_category(), "open " + device_);
    if (::ioctl(fd_, TIOCEXCL) < 0)
        throw std::system_error(errno, std::generic_category(), "exclusive serial ownership");
    configureSerial(cfg.baud);
    first_response_deadline_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                      std::chrono::duration<double>(cfg.first_response_timeout_seconds));
}

Parol6SerialBackend::FileDescriptor::~FileDescriptor() {
    if (value >= 0)
        ::close(value);
}
Parol6SerialBackend::~Parol6SerialBackend() = default;

void Parol6SerialBackend::configureSerial(int baud) {
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
        throw std::system_error(errno, std::generic_category(), "tcgetattr");
    cfmakeraw(&tty);
    speed_t speed{};
    if (baud == 3000000) {
#ifdef B3000000
        speed = B3000000;
#else
        throw std::runtime_error("This libc does not expose B3000000");
#endif
    } else if (baud == 115200) {
        speed = B115200; // useful for PTY/bench tests only
    } else {
        throw std::runtime_error("Unsupported baud; PAROL6 expects 3000000");
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        throw std::system_error(errno, std::generic_category(), "tcsetattr");
    tcflush(fd_, TCIOFLUSH);
}

void Parol6SerialBackend::drainReceive() {
    std::array<std::uint8_t, 512> scratch{};
    for (unsigned attempt = 0; attempt < 4; ++attempt) {
        const ssize_t n = ::read(fd_, scratch.data(), scratch.size());
        if (n > 0) {
            protocol::TelemetryDecoded latest;
            if (parser_.push(std::span<const std::uint8_t>(scratch.data(), static_cast<std::size_t>(n)), latest) > 0)
                applyTelemetry(latest);
        } else {
            if (n < 0 && errno == EINTR)
                continue;
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
                io_fault_ = true;
            break;
        }
    }
}

void Parol6SerialBackend::applyTelemetry(const protocol::TelemetryDecoded &decoded) {
    observation_.robot = decoded.robot;
    observation_.status = decoded.status;
    last_rx_ = std::chrono::steady_clock::now();
    observation_.robot.sequence = ++state_sequence_;
}

void Parol6SerialBackend::progressTransmit(std::chrono::steady_clock::time_point now) {
    if (!pending_ || io_fault_)
        return;
    if (now >= pending_->deadline) {
        io_fault_ = true;
        return; // Never append another packet to an abandoned partial frame.
    }
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        auto &frame = *pending_;
        const auto count = ::write(fd_, frame.bytes.data() + frame.offset, frame.bytes.size() - frame.offset);
        if (count > 0)
            frame.offset += static_cast<std::size_t>(count);
        else if (count < 0 && errno == EINTR)
            continue;
        else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
            return;
        else {
            io_fault_ = true;
            return;
        }
        if (frame.offset == frame.bytes.size()) {
            transmitted_ = Transmission{frame.request_sequence, std::chrono::steady_clock::now(), static_cast<std::uint8_t>(frame.command)};
            // This is host transmission bookkeeping, never an enabled-state measurement.
            if (frame.command == protocol::FirmwareCommand::Enable)
                backend_enabled_ = true;
            if (frame.command == protocol::FirmwareCommand::InhibitMotion || frame.command == protocol::FirmwareCommand::Idle)
                backend_enabled_ = false;
            pending_.reset();
            return;
        }
    }
}
void Parol6SerialBackend::transmit(const JointSetpoint &sp, const GripperCommand &grip) {
    const auto now = std::chrono::steady_clock::now();
    progressTransmit(now);
    if (pending_ || io_fault_)
        return;
    const bool awaiting_feedback = !observation_.robot.valid;
    protocol::FirmwareCommand command = protocol::FirmwareCommand::Idle;
    if (awaiting_feedback || observation_.robot.fault || observation_.robot.physical_estop || (sp.valid && sp.inhibit_motion)) {
        // robot::execution::Status is request-driven: repeat inhibition to observe E-stop release
        // and recovery. Startup also cancels any old firmware motion before control can be
        // acquired.
        command = protocol::FirmwareCommand::InhibitMotion;
    } else if (sp.valid && sp.enable) {
        command = backend_enabled_ ? protocol::FirmwareCommand::Move : protocol::FirmwareCommand::Enable;
    }
    // The protocol has no read-only query. Startup output ownership must be configured.
    // Once feedback exists, preserve reported outputs unless explicitly commanding OUT1.
    auto outputs = startup_outputs_;
    if (!awaiting_feedback) {
        outputs = {protocol::bitMsbFirst(observation_.status.io_bits, 2), protocol::bitMsbFirst(observation_.status.io_bits, 3)};
        if (grip.open)
            outputs[0] = pneumatic_open_high_ ? *grip.open : !*grip.open;
    }
    pending_ = PendingFrame{protocol::encodeCommandFrame(sp, command, outputs), 0, sp.sequence, command, now + transmission_timeout_};
    progressTransmit(now);
}

void Parol6SerialBackend::readAvailable(std::chrono::steady_clock::time_point now) {
    if (!observation_.robot.valid && now >= first_response_deadline_)
        io_fault_ = true;
    drainReceive();
    if (observation_.robot.valid && last_rx_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - last_rx_).count() > telemetry_timeout_seconds_)
        observation_.robot.fault = true;
    observation_.robot.fault = observation_.robot.fault || io_fault_;
}

} // namespace parol6
