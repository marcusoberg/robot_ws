#pragma once
#include <iterator>
#include <memory>
#include <tuple>
#include <utility>

#include "robot/backend/concepts.hpp"
#include "robot/resources/sensors.hpp"

namespace robot::backend {
// A provider acquires on its own thread/device cadence. latest() returns owned
// snapshots; camera buffers use shared immutable storage in SensorSample.
template <class T>
concept SensorSource = requires(const T &source) {
    { source.latest() } -> std::same_as<std::vector<SensorSample>>;
    { source.sensorDescriptions() } -> std::same_as<const std::vector<resources::SensorDescription> &>;
};

template <Device Arm, SensorSource... Sensors> class WithSensors {
public:
    WithSensors(std::unique_ptr<Arm> arm, std::unique_ptr<Sensors>... sources) : arm_(std::move(arm)), sources_(std::move(sources)...) {
        if constexpr (requires(const Arm &device) { device.sensorDescriptions(); })
            descriptions_ = arm_->sensorDescriptions();
        std::apply(
            [&](const auto &...source) {
                const auto append = [&](const auto &provider) {
                    const auto &description = provider->sensorDescriptions();
                    descriptions_.insert(descriptions_.end(), description.begin(), description.end());
                };
                (append(source), ...);
            },
            sources_);
    }

    const std::vector<resources::SensorDescription> &sensorDescriptions() const { return descriptions_; }

    [[nodiscard]] const Model &model() const { return arm_->model(); }

    [[nodiscard]] BackendObservation read(core::HostClock::time_point now) {
        auto observation = arm_->read(now);
        std::apply(
            [&](const auto &...source) {
                const auto append = [&](const auto &provider) {
                    auto samples = provider->latest();
                    observation.sensors.insert(observation.sensors.end(), std::make_move_iterator(samples.begin()), std::make_move_iterator(samples.end()));
                };
                (append(source), ...);
            },
            sources_);
        return observation;
    }

    [[nodiscard]] BackendCapabilities capabilities() const { return arm_->capabilities(); }

    Delivery write(const ActuatorCommand &command) { return arm_->write(command); }

    [[nodiscard]] core::Period physicsPeriod() const
        requires Stepped<Arm>
    {
        return arm_->physicsPeriod();
    }

    [[nodiscard]] core::Time time() const
        requires Stepped<Arm>
    {
        return arm_->time();
    }
    void reset()
        requires requires(Arm &arm) { arm.reset(); }
    {
        arm_->reset();
    }
    void advance(std::size_t count)
        requires Stepped<Arm>
    {
        arm_->advance(count);
    }

private:
    std::unique_ptr<Arm> arm_;
    std::tuple<std::unique_ptr<Sensors>...> sources_;
    std::vector<resources::SensorDescription> descriptions_;
};
} // namespace robot::backend
