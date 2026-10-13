#pragma once
#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

#include "robot/execution/controller_config.hpp"

namespace robot::execution {
using MotionInput = ruckig::InputParameter<ruckig::DynamicDOFs, ruckig::EigenVector>;
using MotionCurve = ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::EigenVector>;
using MotionSolver = ruckig::Ruckig<ruckig::DynamicDOFs, ruckig::EigenVector>;

enum class ServoProgress { Following, Limited, Unavailable };

inline MotionInput motionInput(const ControllerConfig &config) {
    MotionInput input(config.model().dof());
    input.max_velocity = config.velocity();
    input.max_acceleration = config.acceleration();
    input.max_jerk = config.jerk();
    return input;
}

inline bool insideBounds(MotionCurve &curve, const Model &model) {
    const auto bounds = curve.get_position_extrema();
    for (Eigen::Index i = 0; i < model.dof(); ++i)
        if (bounds[i].min < model.description().lower[i] - 1e-9 || bounds[i].max > model.description().upper[i] + 1e-9)
            return false;
    return true;
}

// Retargets start from the previous reference, preserving its velocity/acceleration.
// Candidate states are committed only if both the motion and a subsequent stop fit.
class MotionGenerator {
public:
    explicit MotionGenerator(const ControllerConfig &config)
        : model_(config.model()), solver_(model_.dof()), input_(motionInput(config)), stop_input_(motionInput(config)), curve_(model_.dof()),
          stop_curve_(model_.dof()), position_(model_.dof()), velocity_(model_.dof()), acceleration_(model_.dof()), constrained_(model_.dof()),
          feasible_velocity_(model_.dof()), stationary_(Eigen::VectorXd::Zero(model_.dof())) {
        constrained_.setConstant(false);
        stop_input_.control_interface = ruckig::ControlInterface::Velocity;
        stop_input_.synchronization = ruckig::Synchronization::None;
    }

    // A streamed velocity is a desired direction, not a promise to reach an
    // endpoint. If a joint cannot fit, brake that joint while allowing other
    // joints to follow the stream. Velocity profiles are independent; every
    // accepted candidate still has to fit and permit a bounded stop on all axes.
    // Failed candidates never modify the reference, so braking starts from the
    // same position/velocity/acceleration. Retargeting remains possible next tick.
    [[nodiscard]] ServoProgress servo(Eigen::VectorXd &position, Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const Eigen::VectorXd &desired,
                                      double dt) {
        if (advance(position, velocity, acceleration, desired, ruckig::ControlInterface::Velocity, dt))
            return ServoProgress::Following;
        if (constrained_.any()) {
            feasible_velocity_ = constrained_.select(stationary_.array(), desired.array());
            if (advance(position, velocity, acceleration, feasible_velocity_, ruckig::ControlInterface::Velocity, dt))
                return ServoProgress::Limited;
        }
        if (advance(position, velocity, acceleration, stationary_, ruckig::ControlInterface::Velocity, dt))
            return ServoProgress::Limited;
        return ServoProgress::Unavailable;
    }

    bool advance(Eigen::VectorXd &position, Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const Eigen::VectorXd &target,
                 ruckig::ControlInterface interface, double dt, std::optional<double> minimum_duration = {}) {
        constrained_.setConstant(false);
        const bool same_target =
            has_curve_ && input_.control_interface == interface && (position.array() == position_.array()).all() &&
            (velocity.array() == velocity_.array()).all() && (acceleration.array() == acceleration_.array()).all() &&
            (interface == ruckig::ControlInterface::Position
                 ? (input_.target_position.array() == target.array()).all()
                 : (input_.target_velocity.array() == target.cwiseMax(-input_.max_velocity).cwiseMin(input_.max_velocity).array()).all()) &&
            (minimum_duration.has_value() == input_.minimum_duration.has_value()) &&
            (!minimum_duration || std::abs(*minimum_duration - std::max(0.0, *input_.minimum_duration - elapsed_)) <= dt + 1e-9);

        if (!same_target) {
            has_curve_ = false;
            input_.current_position = position;
            input_.current_velocity = velocity;
            input_.current_acceleration = acceleration;
            input_.control_interface = interface;
            input_.minimum_duration = minimum_duration;
            input_.synchronization = interface == ruckig::ControlInterface::Position ? ruckig::Synchronization::Time : ruckig::Synchronization::None;
            if (interface == ruckig::ControlInterface::Position) {
                input_.target_position = target;
                input_.target_velocity.setZero();
            } else {
                input_.target_velocity = target.cwiseMax(-input_.max_velocity).cwiseMin(input_.max_velocity);
            }
            if (solver_.calculate(input_, curve_) < 0 || !withinBounds(curve_))
                return false;
            elapsed_ = 0.0;
            has_curve_ = true;
        }
        // Advance the existing polynomial until the target changes. Re-solving an
        // already completed move amplifies floating-point residue into tiny new moves.
        elapsed_ += dt;
        curve_.at_time(elapsed_, position_, velocity_, acceleration_);
        if (elapsed_ >= curve_.get_duration()) {
            if (interface == ruckig::ControlInterface::Position)
                position_ = input_.target_position;
            velocity_ = input_.target_velocity;
            acceleration_ = input_.target_acceleration;
        }
        if (!canStop(position_, velocity_, acceleration_)) {
            has_curve_ = false;
            return false;
        }
        position = position_;
        velocity = velocity_;
        acceleration = acceleration_;
        return true;
    }

    bool canStop(const Eigen::VectorXd &position, const Eigen::VectorXd &velocity, const Eigen::VectorXd &acceleration) {
        stop_input_.current_position = position;
        stop_input_.current_velocity = velocity;
        stop_input_.current_acceleration = acceleration;
        return solver_.calculate(stop_input_, stop_curve_) >= 0 && withinBounds(stop_curve_);
    }

private:
    bool withinBounds(MotionCurve &curve) {
        const auto bounds = curve.get_position_extrema();
        bool feasible = true;
        for (Eigen::Index i = 0; i < model_.dof(); ++i) {
            if (bounds[i].min < model_.description().lower[i] - 1e-9 || bounds[i].max > model_.description().upper[i] + 1e-9) {
                constrained_[i] = true;
                feasible = false;
            }
        }
        return feasible;
    }

    Model model_;
    MotionSolver solver_;
    MotionInput input_, stop_input_;
    MotionCurve curve_, stop_curve_;
    Eigen::VectorXd position_, velocity_, acceleration_;
    Eigen::Array<bool, Eigen::Dynamic, 1> constrained_;
    Eigen::VectorXd feasible_velocity_, stationary_;
    double elapsed_{};
    bool has_curve_{false};
};
} // namespace robot::execution
