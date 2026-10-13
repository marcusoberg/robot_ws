#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace parol6 {
// These fixed-size types describe the PAROL6 wire protocol, not a generic robot.
inline constexpr std::size_t kJointCount = 6;
using JointVector = std::array<double, kJointCount>;
struct RobotState {
    JointVector q{}, dq{};
    bool valid{}, enabled{}, homed{}, physical_estop{}, fault{};
    std::uint64_t sequence{};
};
struct JointSetpoint {
    JointVector q{}, dq{};
    bool valid{}, enable{}, inhibit_motion{};
    std::uint64_t sequence{};
};
struct GripperCommand {
    std::optional<bool> open;
    std::uint64_t sequence{};
};
struct DeviceStatus {
    std::uint8_t homed_bits{}, io_bits{}, temperature_bits{}, position_error_bits{};
    std::uint16_t timing_ticks{};
    std::uint8_t timeout_error{}, echoed_command{255}, gripper_device_id{}, gripper_status{};
    std::int16_t gripper_position{}, gripper_speed{}, gripper_current{};
    std::uint8_t gripper_object_detection{}, raw_tool_byte{};
};
struct BackendObservation {
    RobotState robot;
    DeviceStatus status;
};
struct Transmission {
    std::uint64_t request_sequence{};
    std::chrono::steady_clock::time_point at;
    std::uint8_t command{};
};
struct TransmissionStatus {
    std::optional<Transmission> transmitted;
    std::optional<std::uint64_t> pending;
    bool fault{};
};
} // namespace parol6
