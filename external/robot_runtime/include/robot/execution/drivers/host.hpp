#pragma once
#include "core/execution.hpp"
#include "robot/execution.hpp"
namespace robot::execution::drivers {
enum class OverrunPolicy { Fault, Brake };
struct HostTiming {
    core::Period maximum_interval;
    OverrunPolicy overrun{OverrunPolicy::Fault};
};
class Host {
public:
    using Pacing = core::HostCadence;
    Host(Execution &execution, core::Period period) : Host(execution, period, HostTiming{core::Period{2 * period.value()}}) {}
    Host(Execution &execution, core::Period period, HostTiming timing)
        : execution_(execution), period_(period), timing_(timing), origin_(core::HostClock::now()), previous_(origin_) {
        execution.bindTiming(communication::TimingProfile::Hardware);
        if (auto required = execution.communication()->capabilities().required_period; required && required->value() != period.value())
            throw std::invalid_argument("control period disagrees with backend command law");
    }
    core::Period period() const { return period_; }
    void step() {
        const auto now = core::HostClock::now();
        const core::Context ctx{core::Time{previous_ - origin_}, core::Time{now - origin_}, sequence_++, epoch_};
        const auto violation = ctx.elapsed() <= timing_.maximum_interval.value() ? TimingViolation::None
                               : timing_.overrun == OverrunPolicy::Fault         ? TimingViolation::Fault
                                                                                 : TimingViolation::Brake;
        execution_.control(ctx, violation);
        execution_.observe(ctx);
        previous_ = now;
    }

private:
    Execution &execution_;
    core::Period period_;
    HostTiming timing_;
    core::HostClock::time_point origin_, previous_;
    std::uint64_t sequence_{}, epoch_{};
};
} // namespace robot::execution::drivers
