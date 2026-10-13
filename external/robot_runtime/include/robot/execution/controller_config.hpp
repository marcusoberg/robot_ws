#pragma once
#include <optional>
#include <utility>

#include "robot/boundary/numbers.hpp"
#include "robot/resources/model.hpp"

namespace robot::execution {
using TrackingTolerance = resources::JointMap<boundary::PositiveNumber>;
struct ControllerParameters {
    boundary::UnitFraction speed_fraction;
    boundary::PositiveNumber acceleration_time, jerk_time;
    boundary::PositiveNumber position_gain{8.0}, damping{0.04};
    std::optional<boundary::PositiveNumber> orientation_gain;
    std::optional<TrackingTolerance> tracking_error;
};

class ControllerConfig {
public:
    ControllerConfig(Model model, const ControllerParameters &p)
        : model_(std::move(model)), velocity_(model_.description().speed * p.speed_fraction.value()), acceleration_(velocity_ / p.acceleration_time.value()),
          jerk_(acceleration_ / p.jerk_time.value()), gain_(p.position_gain.value()), damping_(p.damping.value()),
          orientation_gain_(p.orientation_gain.value_or(p.position_gain).value()), tracking_error_(p.tracking_error) {}
    [[nodiscard]] const Eigen::VectorXd &velocity() const { return velocity_; }
    [[nodiscard]] const Eigen::VectorXd &acceleration() const { return acceleration_; }
    [[nodiscard]] const Eigen::VectorXd &jerk() const { return jerk_; }
    [[nodiscard]] double gain() const { return gain_; }
    [[nodiscard]] double damping() const { return damping_; }
    [[nodiscard]] double orientationGain() const { return orientation_gain_; }
    [[nodiscard]] const std::optional<TrackingTolerance> &trackingError() const { return tracking_error_; }
    [[nodiscard]] const Model &model() const { return model_; }

private:
    Model model_;
    Eigen::VectorXd velocity_, acceleration_, jerk_;
    double gain_, damping_, orientation_gain_;
    std::optional<TrackingTolerance> tracking_error_;
};

} // namespace robot::execution
