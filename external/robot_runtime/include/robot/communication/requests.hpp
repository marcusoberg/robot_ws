#pragma once
#include <array>
#include <optional>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "core/types.hpp"
#include "robot/communication/authority.hpp"

namespace robot::communication {
// Transport-independent input descriptions. These contain external values, not
// admitted robot commands. Decoder binds them to the configured model.
struct ModelJointOrder {};
struct NamedJointOrder {
    std::vector<std::string> names;
};
using JointOrder = std::variant<ModelJointOrder, NamedJointOrder>;

struct JointPositionRequest {
    JointOrder order;
    std::vector<double> values;
};
struct JointVelocityRequest {
    JointOrder order;
    std::vector<double> values;
};
struct PoseRequest {
    std::string frame;
    std::array<double, 3> position;
    std::array<double, 4> quaternion_wxyz;
    std::map<std::string, double> auxiliary_positions;
};
struct TwistRequest {
    std::string frame;
    std::array<double, 6> values;
};
struct WaypointRequest {
    std::vector<double> position;
    std::optional<std::vector<double>> velocity;
    core::duration offset;
};
struct TrajectoryRequest {
    JointOrder order;
    std::vector<WaypointRequest> points;
};
struct ReferencePointRequest {
    std::vector<double> position, velocity, acceleration;
    core::duration offset;
};
struct ScheduleRequest {
    JointOrder order;
    core::Time start;
    std::uint64_t epoch;
    std::optional<communication::Id> replaces;
    std::vector<ReferencePointRequest> points;
};
// The original sample origin remains fixed. Model-specific output decoding stays external.
struct TimedJointTargetsRequest {
    static constexpr std::size_t maximum_samples=256;
    JointOrder order;
    std::vector<std::vector<double>> positions;
    std::string runtime_instance;
    std::uint64_t epoch{}, observation_sequence{};
    core::Time origin{}, start{};
    core::duration period{}, join{std::chrono::seconds(1)}, lead{std::chrono::milliseconds(150)};
    std::vector<double> tolerance{0.02};
    std::optional<communication::Id> replaces;
};
using MotionRequest = std::variant<JointPositionRequest, JointVelocityRequest, PoseRequest, TwistRequest, TrajectoryRequest, ScheduleRequest, TimedJointTargetsRequest>;
struct CommandRequest {
    communication::Session session;
    communication::Id id;
    communication::Expiry expiry;
    MotionRequest motion;
    std::optional<bool> gripper_open;
};
} // namespace robot::communication
