#pragma once
#include "robot/backend/sensors.hpp"
#include "core.hpp"
#include <mutex>
#include <algorithm>
#include <cmath>

namespace application {
// Acquisition remains outside execution. This is the single ingress boundary for
// externally acquired samples; backend reads consume already-owned snapshots.
class SensorInbox {
public:
    explicit SensorInbox(std::vector<robot::resources::SensorDescription> descriptions) : descriptions_(std::move(descriptions)) {}
    const auto &descriptions() const { return descriptions_; }
    std::vector<robot::backend::SensorSample> latest() const { return samples_.read(); }
    void clear() { std::scoped_lock lock(mutex_); samples_.publish({}); }
    void publish(robot::backend::SensorSample sample) {
        const auto description = std::ranges::find(descriptions_, sample.name, &robot::resources::SensorDescription::name);
        if (description == descriptions_.end()) throw std::invalid_argument("sensor was not configured");
        sample.frame = description->frame; sample.calibration = description->calibration;
        if (sample.stamp.source_clock.empty()) throw std::invalid_argument("sensor source clock must be identified");
        if (auto *numeric = std::get_if<robot::backend::NumericSensor>(&sample.payload)) {
            if (numeric->values.size() != description->units.size()) throw std::invalid_argument("sample disagrees with configured sensor schema");
            for (double value : numeric->values) if (!std::isfinite(value)) throw std::invalid_argument("nonfinite sensor input");
            numeric->units = description->units;
        }
        std::scoped_lock lock(mutex_);
        auto values = samples_.read();
        const auto it = std::ranges::find(values, sample.name, &robot::backend::SensorSample::name);
        if (it == values.end()) values.push_back(std::move(sample)); else *it = std::move(sample);
        samples_.publish(std::move(values));
    }
private:
    std::vector<robot::resources::SensorDescription> descriptions_;
    core::Snapshot<std::vector<robot::backend::SensorSample>> samples_{{}};
    std::mutex mutex_;
};
class SensorProvider {
public:
    explicit SensorProvider(std::shared_ptr<SensorInbox> inbox) : inbox_(std::move(inbox)) {}
    auto latest() const { return inbox_->latest(); }
    const auto &sensorDescriptions() const { return inbox_->descriptions(); }
private:
    std::shared_ptr<SensorInbox> inbox_;
};
}
