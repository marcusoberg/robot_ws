#pragma once
#include <array>
#include <optional>
#include <utility>

#include "robot/backend/types.hpp"
#include "robot/communication/authority.hpp"
#include "robot/communication/messages.hpp"
#include "robot/execution/scheduled_motion.hpp"

namespace robot::execution {
// Coherent runtime snapshots, including authority status and measured feedback.
enum class Mode { Disarmed, Hold, Autonomous, Direct, Stopping, Fault };

enum class RejectReason { None, NoAuthority, Stale, WrongModel, InvalidState, QueueFull, Overrun, Infeasible, Late, Conflict, Discontinuous, BufferUnderrun };
struct ScheduledReference {
    execution::ScheduledMotion motion;
    communication::Reference reference;
};

// At most one future replacement. Copies retain immutable path storage; execution
// rechecks the predecessor and join so preparation cannot overwrite intervening work.
struct ScheduleSnapshot {
    ScheduledReference current;
    std::optional<ScheduledReference> pending;
};
struct AuthorityTransition {
    std::uint64_t sequence{};
    Mode from{Mode::Disarmed}, to{Mode::Disarmed};
    std::optional<communication::Session> owner;
    std::uint64_t generation{};
    RejectReason reason{RejectReason::None};
    core::Time time{};
    std::uint64_t epoch{};
};
struct Status {
    std::uint64_t transition_sequence{};
    std::array<AuthorityTransition, 32> transitions{};
    Mode mode{Mode::Disarmed};
    std::optional<communication::Session> owner;
    std::optional<communication::HostExpiry> authority_expiry;
    std::uint64_t generation{};
    std::optional<communication::Reference> active, completed, canceled, rejected, accepted;
    std::optional<ScheduleSnapshot> schedule;
    RejectReason rejection{RejectReason::None};
    RejectReason interruption{RejectReason::None};
    // Current servo is braking/held at a constraint; authority remains valid.
    bool motion_limited{};
    std::uint64_t last_operator_request{};
    communication::SourceId last_operator_source{};
    std::uint64_t operator_sequence{};
    std::array<communication::OperatorResult, 64> operator_results{};
};

struct DerivedToolPose {
    Eigen::Isometry3d base_from_tool;
    backend::SampleStamp based_on;
};

struct Snapshot {
    // Application identity is attached outside graph evaluation.
    backend::BackendObservation backend;
    Status status;
    backend::ActuatorCommand actuator_request;
    std::optional<DerivedToolPose> tool_pose;
    core::Time control_time{};
    std::uint64_t epoch{};
    core::HostClock::time_point control_received{};
    backend::Delivery delivery;
    core::duration control_dt{}, host_interval{}, cycle_duration{};
    // Last decoded proposal, including proposals later declined at admission.
    std::optional<communication::Command> requested_motion;
    // Publication identity is independent of actuator and device sample sequences.
    std::uint64_t observation_sequence{};
    std::string runtime_instance;
};

inline const char *toString(Mode mode) {
    switch (mode) {
    case Mode::Disarmed:
        return "DISABLED";
    case Mode::Hold:
        return "HOLD";
    case Mode::Autonomous:
        return "AUTO";
    case Mode::Direct:
        return "DIRECT";
    case Mode::Stopping:
        return "CONTROLLED_STOP";
    case Mode::Fault:
        return "FAULT";
    }
    std::unreachable();
}
} // namespace robot::execution
