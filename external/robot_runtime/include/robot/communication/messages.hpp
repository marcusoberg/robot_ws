#pragma once
#include <optional>
#include <variant>

#include "robot/communication/authority.hpp"
#include "robot/communication/cartesian.hpp"
#include "robot/execution/scheduled_motion.hpp"
#include "robot/execution/trajectory.hpp"
#include "robot/resources/model.hpp"

namespace robot::communication {
// Commands crossing into execution. Each motion kind has its own payload type.
// The first four alternatives are latest-target streams, valid until expiry or
// explicit interruption. Only execution::Trajectory describes a finite, completable move.
// A schedule has an execution horizon independent of its renewable authority lease.
using Motion = std::variant<resources::Positions, resources::Velocities, Pose, Twist, execution::Trajectory, execution::ScheduledMotion>;

struct Command {
    Session session;
    Id id;
    Expiry expiry;
    Motion motion;
    std::optional<bool> gripper_open;
};

// Direct input is an independent device snapshot; both jog kinds are sampled
// together, so a mode switch cannot leave the previous kind authoritative.
struct DirectInput {
    bool deadman{};
    core::HostClock::time_point expires{};
    std::variant<std::monostate, resources::Velocities, Twist> jog;
};

} // namespace robot::communication
