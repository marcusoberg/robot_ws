#pragma once
#include "robot/execution/demand.hpp"
#include "robot/execution/state.hpp"
namespace robot::execution {
enum class TimingViolation { None, Brake, Fault };
struct ControlFeedback {
    Reference reference;
    Progress progress;
    bool transport_fault{}, abort_motion{};
};
struct InputFrame {
    backend::BackendObservation feedback;
    core::HostClock::time_point now;
    TimingViolation timing{TimingViolation::None};
    std::optional<communication::OperatorRequest> disable, stop;
    std::array<std::optional<communication::OperatorRequest>, 64> operators;
    std::optional<communication::RenewalWindow> renewal;
    std::array<std::optional<communication::Command>, 8> schedules;
    std::optional<communication::Command> command;
    communication::DirectInput direct;
};
struct AuthorityFrame {
    Mode mode;
    bool capture{}, clear{};
    std::optional<resources::Positions> standby;
    communication::DirectInput direct;
    std::optional<communication::Session> owner;
    communication::Id last_accepted;
    bool binary_tool{}, accept_commands{};
};
struct MotionAdmission {
    communication::Command command;
    RejectReason reason;
};
struct PlanFrame {
    Demand demand{Brake{}};
    bool underrun{}, finished{}, new_motion{};
    std::array<std::optional<MotionAdmission>, 9> admissions;
    std::optional<communication::Reference> active, canceled;
    std::optional<ScheduleSnapshot> schedule;
};
struct ControlFrame {
    Demand demand;
    bool capture;
    Mode mode;
    std::optional<bool> gripper;
    backend::Actuation disarmed_actuation{backend::Actuation::Idle};
};
struct ControlOutput {
    backend::ActuatorCommand request;
    ControlFeedback feedback;
};
struct DeliveryOutput {
    ControlOutput control;
    backend::Delivery delivery;
};
} // namespace robot::execution
