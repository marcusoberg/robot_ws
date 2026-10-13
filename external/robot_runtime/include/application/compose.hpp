#pragma once
#include "application/deployment.hpp"
#include "robot/execution.hpp"
#include "robot/execution/drivers/host.hpp"
#include "robot/execution/drivers/simulation.hpp"
#ifdef ROBOT_RUNTIME_HAS_SERIAL
#include "adapters/backends/parol6_serial.hpp"
#endif

namespace application {
template <robot::backend::Device Device, class Function>
int withDriver(robot::Execution &execution, Device &device, core::Period period, Function &&function,
               std::optional<robot::execution::drivers::HostTiming> timing = {}) {
    if constexpr (robot::backend::Stepped<Device>) {
        robot::execution::drivers::Simulation driver{execution, device, period};
        return function(driver);
    } else {
        robot::execution::drivers::Host driver{execution, period, timing.value_or(robot::execution::drivers::HostTiming{core::Period{2 * period.value()}})};
        return function(driver);
    }
}

// Backend choice occurs once at application composition. Device adapters are bound here, before
// execution starts.
template <class Application> int withBackend(const std::string &name, Deployment &deployment, Application &&application) {
#ifdef ROBOT_RUNTIME_HAS_MUJOCO
    if (name == "mujoco")
        return application(std::make_unique<adapters::backends::MujocoBackend>(deployment.model, deployment.mujoco, deployment.initial));
#endif
#ifdef ROBOT_RUNTIME_HAS_SERIAL
    if (name == "serial")
        return application(std::make_unique<adapters::backends::Parol6SerialBackend>(deployment.model, deployment.serial, deployment.wire_joints));
#endif
    throw std::invalid_argument("backend not built: " + name);
}
} // namespace application
