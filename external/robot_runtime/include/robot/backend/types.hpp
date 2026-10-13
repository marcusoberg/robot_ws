#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "core/types.hpp"
#include "robot/resources/model.hpp"

namespace robot::backend {
// Backend requests, measured samples and evidence of command delivery.
enum class InhibitBehavior { ActuatorForcesOff, FirmwareMotionInhibit };

struct BackendCapabilities {
    std::string_view name, feedback_source;
    InhibitBehavior inhibit;
    bool binary_tool{};
    bool homing_operation{};
    std::optional<core::Period> required_period;
};

enum class Actuation { Idle, Position, Inhibit };

struct ActuatorCommand {
    Actuation actuation{Actuation::Idle};
    resources::Positions position;
    resources::Velocities velocity;
    std::optional<bool> gripper_open;
    std::uint64_t sequence{};
    resources::Accelerations acceleration;
};

struct UnreportedDelivery {};

struct SimulatorDelivery {
    std::uint64_t request_sequence;
};

struct TransmittedFrame {
    std::uint64_t request_sequence;
    core::HostClock::time_point at;
    std::uint8_t device_command;
};

struct TransportDelivery {
    std::optional<TransmittedFrame> transmitted;
    std::optional<std::uint64_t> pending;
    bool fault{};
};

using Delivery = std::variant<UnreportedDelivery, SimulatorDelivery, TransportDelivery>;

struct SampleStamp {
    std::uint64_t sequence{};
    core::Time logical_time{};
    std::uint64_t epoch{};
    core::HostClock::time_point received{};
    // Source time is optional and labelled; it is never fabricated from publication time.
    std::optional<std::int64_t> source_time_ns;
    std::string source_clock;
};

struct NumericSensor {
    std::vector<double> values;
    std::vector<std::string> units;
};

struct ImageSensor {
    std::uint32_t width{}, height{}, stride{};
    std::string encoding;
    std::shared_ptr<const std::vector<std::byte>> bytes;
};

struct SensorSample {
    std::string name, frame, calibration;
    SampleStamp stamp;
    std::variant<NumericSensor, ImageSensor> payload;
};

struct RobotState {
    resources::Measurement joints;
    bool homed{}, physical_estop{}, fault{};
    std::optional<bool> enabled{}, gripper_open{};
    SampleStamp stamp;
};

struct BackendObservation {
    std::optional<RobotState> state;
    std::optional<bool> collision;
    std::vector<SensorSample> sensors;
};
} // namespace robot::backend
