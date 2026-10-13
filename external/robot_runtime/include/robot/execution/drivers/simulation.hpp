#pragma once
#include "core/execution.hpp"
#include "robot/execution.hpp"
namespace robot::execution::drivers {
template <backend::Stepped Device> class Simulation {
public:
    using Pacing = core::SimulationPacing;
    Simulation(Execution &execution, Device &device, core::Period period) : execution_(execution), device_(device), period_(period) {
        execution.bindTiming(communication::TimingProfile::Simulation);
        const auto physics = device.physicsPeriod().value();
        if (period.value() % physics != core::duration{})
            throw std::invalid_argument("control period must divide into physics steps");
        substeps_ = period.value() / physics;
    }
    core::Period period() const { return period_; }
    void step() {
        const auto begin = device_.time();
        core::Context ctx{begin, begin + period_.value(), sequence_++, epoch_};
        execution_.control(ctx);
        device_.advance(substeps_);
        ctx.end = device_.time();
        execution_.observe(ctx);
    }
    void advance(std::size_t intervals) {
        for (std::size_t i = 0; i < intervals; ++i)
            step();
    }
    void reset()
        requires requires(Device &device) { device.reset(); }
    {
        device_.reset();
        ++epoch_;
        sequence_ = 0;
        execution_.reset();
        const core::Context reset_context{device_.time(), device_.time(), sequence_, epoch_};
        execution_.control(reset_context);
        execution_.observe(reset_context);
    }

private:
    Execution &execution_;
    Device &device_;
    core::Period period_;
    std::size_t substeps_{};
    std::uint64_t sequence_{}, epoch_{};
};
} // namespace robot::execution::drivers
