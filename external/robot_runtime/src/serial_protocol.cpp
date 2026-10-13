#include "parol6/serial_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "parol6/config.hpp"

namespace parol6::protocol {

bool bitMsbFirst(std::uint8_t value, int array_index) {
    if (array_index < 0 || array_index > 7)
        return false;
    return ((value >> (7 - array_index)) & 1U) != 0;
}

std::int32_t decodeSigned24(const std::uint8_t *p) {
    std::int32_t v = (static_cast<std::int32_t>(p[0]) << 16) | (static_cast<std::int32_t>(p[1]) << 8) | static_cast<std::int32_t>(p[2]);
    if (v & 0x800000)
        v |= ~0xFFFFFF;
    return v;
}

void encodeSigned24(std::uint8_t *p, std::int32_t value) {
    if (value < -8'388'608 || value > 8'388'607)
        throw std::invalid_argument("signed 24-bit position is not representable");
    const std::uint32_t v = static_cast<std::uint32_t>(value) & 0xFFFFFFU;
    p[0] = static_cast<std::uint8_t>((v >> 16) & 0xFFU);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFU);
    p[2] = static_cast<std::uint8_t>(v & 0xFFU);
}

std::int16_t decodeSigned16(const std::uint8_t *p) {
    const auto raw = static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | static_cast<std::uint16_t>(p[1]));
    return static_cast<std::int16_t>(raw);
}

std::int32_t radiansToSteps(double radians, std::size_t joint) {
    const double steps = radians / config::kRadiansPerMotorStep * config::kGearRatio.at(joint);
    if (!std::isfinite(steps) || steps < -8'388'608.0 || steps > 8'388'607.0)
        throw std::invalid_argument("joint target exceeds signed 24-bit wire range");
    return static_cast<std::int32_t>(std::llround(steps));
}

std::int32_t radiansPerSecondToStepsPerSecond(double radians_per_second, std::size_t joint) {
    const double raw = radians_per_second / config::kRadiansPerMotorStep * config::kGearRatio.at(joint);
    if (!std::isfinite(raw))
        throw std::invalid_argument("nonfinite wire velocity");
    if (std::abs(raw) > config::kMaxSpeedStepsPerSecond.at(joint) + 1e-6)
        throw std::invalid_argument("wire velocity exceeds calibrated device speed");
    return static_cast<std::int32_t>(std::llround(raw));
}

double stepsToRadians(std::int32_t steps, std::size_t joint) {
    return static_cast<double>(steps) * config::kRadiansPerMotorStep / config::kGearRatio.at(joint);
}

double stepsPerSecondToRadiansPerSecond(std::int32_t steps_per_second, std::size_t joint) { return stepsToRadians(steps_per_second, joint); }

std::array<std::uint8_t, kCommandFrameSize> encodeCommandFrame(const JointSetpoint &setpoint, FirmwareCommand command, const std::array<bool, 2> &outputs) {
    std::array<std::uint8_t, kCommandFrameSize> frame{};
    frame[0] = frame[1] = frame[2] = kStartByte;
    frame[3] = static_cast<std::uint8_t>(kCommandLengthField);

    for (std::size_t i = 0; i < kJointCount; ++i) {
        encodeSigned24(frame.data() + 4 + 3 * i, radiansToSteps(setpoint.q[i], i));
        encodeSigned24(frame.data() + 22 + 3 * i, radiansPerSecondToStepsPerSecond(setpoint.dq[i], i));
    }

    frame[40] = static_cast<std::uint8_t>(command);
    frame[41] = 0; // affected-joints byte is currently not used by PAROL6 firmware motion.

    // Firmware bit arrays are MSB-first. OUT1 is array index 2, therefore bit 5.
    frame[42] = static_cast<std::uint8_t>((outputs[0] ? 0x20 : 0) | (outputs[1] ? 0x10 : 0));
    frame[43] = 0; // command timeout field; zero matches the official controller.

    // 44..52 are electric-gripper data. They remain zero for the pneumatic tool.
    frame[53] = 228; // Current official fixed CRC placeholder.
    frame[54] = kEnd0;
    frame[55] = kEnd1;
    return frame;
}

bool decodeTelemetryPayload(std::span<const std::uint8_t> payload, TelemetryDecoded &out) {
    if (payload.size() < kTelemetryPayloadSize)
        return false;

    RobotState state{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
        state.q[i] = stepsToRadians(decodeSigned24(payload.data() + 3 * i), i);
        state.dq[i] = stepsPerSecondToRadiansPerSecond(decodeSigned24(payload.data() + 18 + 3 * i), i);
    }

    const std::uint8_t homed = payload[36];
    const std::uint8_t io = payload[37];
    const std::uint8_t temperature_error = payload[38];
    const std::uint8_t position_error = payload[39];

    state.homed = true;
    state.fault = false;
    for (int i = 0; i < static_cast<int>(kJointCount); ++i) {
        state.homed = state.homed && bitMsbFirst(homed, i);
        state.fault = state.fault || bitMsbFirst(temperature_error, i) || bitMsbFirst(position_error, i);
    }

    // Firmware status IO array is {In1, In2, Out1, Out2, Estop, 1, 1, 1}.
    // The physical E-stop input is normally closed: 0 means pressed.
    state.physical_estop = !bitMsbFirst(io, 4);
    state.valid = true;

    out.robot = state;
    out.status.homed_bits = homed;
    out.status.io_bits = io;
    out.status.temperature_bits = temperature_error;
    out.status.position_error_bits = position_error;
    out.status.timing_ticks = static_cast<std::uint16_t>((static_cast<std::uint16_t>(payload[40]) << 8) | static_cast<std::uint16_t>(payload[41]));
    out.status.timeout_error = payload[42];
    out.status.echoed_command = payload[43];
    out.status.gripper_device_id = payload[44];
    out.status.gripper_position = decodeSigned16(payload.data() + 45);
    out.status.gripper_speed = decodeSigned16(payload.data() + 47);
    out.status.gripper_current = decodeSigned16(payload.data() + 49);
    out.status.gripper_status = payload[51];
    out.status.gripper_object_detection = (payload[51] >> 2) & 3;
    out.status.raw_tool_byte = payload[52];
    return true;
}

TelemetryStreamParser::TelemetryStreamParser() = default;
void TelemetryStreamParser::clear() { size_ = 0; }
std::size_t TelemetryStreamParser::push(std::span<const std::uint8_t> bytes, TelemetryDecoded &latest) {
    std::size_t decoded = 0;
    // At most one fixed-capacity partial frame. Work is bounded by the input span.
    for (const auto byte : bytes) {
        if (size_ == buffer_.size())
            size_ = 0;
        buffer_[size_++] = byte;
        while (size_ >= 4) {
            if (buffer_[0] != kStartByte || buffer_[1] != kStartByte || buffer_[2] != kStartByte || (buffer_[3] != kCurrentTelemetryLengthField)) {
                std::move(buffer_.begin() + 1, buffer_.begin() + size_, buffer_.begin());
                --size_;
                continue;
            }
            const auto total = std::size_t{4} + buffer_[3];
            if (size_ < total)
                break;
            if (buffer_[total - 2] == kEnd0 && buffer_[total - 1] == kEnd1 && decodeTelemetryPayload({buffer_.data() + 4, kTelemetryPayloadSize}, latest)) {
                ++decoded;
                size_ = 0;
            } else {
                std::move(buffer_.begin() + 1, buffer_.begin() + size_, buffer_.begin());
                --size_;
            }
        }
    }
    return decoded;
}
} // namespace parol6::protocol
