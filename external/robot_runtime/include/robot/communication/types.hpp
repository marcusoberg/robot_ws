#pragma once
#include "robot/backend/types.hpp"
#include "robot/resources/sensors.hpp"
namespace robot::communication {
enum class TimingProfile { Hardware, Simulation, ExternalPhysics };
// Immutable model/capability information; profile is bound before a driver starts.
struct Description {
    Model model;
    backend::BackendCapabilities backend;
    core::Period control_period;
    TimingProfile timing{TimingProfile::Hardware};
    std::vector<resources::SensorDescription> sensors;
    core::duration authority_lease{std::chrono::seconds(1)};
};
} // namespace robot::communication
