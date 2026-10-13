#pragma once
#include "robot/execution/demand.hpp"
#include "robot/execution/ik.hpp"
#include "robot/execution/motion_generator.hpp"

namespace robot::execution {
// Numerical execution only. The state machine supplies a Demand; the controller
// owns all integration buffers and returns progress without changing authority.
template <CartesianSolver Solver = IK> class Controller final {
public:
    explicit Controller(ControllerConfig config)
        : config_(std::move(config)), ik_(config_.model()), profile_(config_), q_(Eigen::VectorXd::Zero(model().dof())), v_(q_), a_(q_), desired_(q_),
          candidate_q_(q_), candidate_v_(q_), candidate_a_(q_),
          reference_{model().joints().template zero<resources::Positions>(), model().joints().template zero<resources::Velocities>(),
                     model().joints().template zero<resources::Accelerations>()} {}
    [[nodiscard]] const Model &model() const { return config_.model(); }
    [[nodiscard]] const ControllerConfig &config() const { return config_; }
    [[nodiscard]] const Reference &reference() const { return reference_; }
    void capture(const resources::Measurement &measured) {
        q_ = measured.position();
        v_ = measured.velocity();
        a_.setZero();
        publishReference();
    }
    void idle(const resources::Measurement &measured) {
        q_ = measured.position();
        hold();
    }
    void hold() {
        v_.setZero();
        a_.setZero();
        publishReference();
    }
    [[nodiscard]] bool trackingHealthy(const resources::Measurement &measured) const {
        if (!config_.trackingError())
            return true;
        const auto &tolerance = *config_.trackingError();
        for (Eigen::Index i = 0; i < model().dof(); ++i)
            if (std::abs(q_[i] - measured.position()[i]) > tolerance[i].value())
                return false;
        return true;
    }
    [[nodiscard]] Progress step(const Demand &demand, const resources::Measurement &measured, double dt) {
        return std::visit([&](const auto &instruction) { return advance(instruction, measured, dt); }, demand);
    }

private:
    void publishReference() {
        reference_.position.assign([&](auto i) { return q_[i]; });
        reference_.velocity.assign([&](auto i) { return v_[i]; });
        reference_.acceleration.assign([&](auto i) { return a_[i]; });
    }
    Progress progress(bool feasible, bool limited, const resources::Measurement &measured, const Eigen::VectorXd *goal = nullptr) {
        publishReference();
        const bool stationary = v_.cwiseAbs().maxCoeff() < 1e-8 && a_.cwiseAbs().maxCoeff() < 1e-8;
        const bool stopped = stationary && (measured.velocity().cwiseAbs().array() < 0.01 * config_.velocity().array()).all();
        const bool settled = stationary && (!goal || ((*goal - q_).cwiseAbs().maxCoeff() < 1e-3 && (*goal - measured.position()).cwiseAbs().maxCoeff() < 0.01));
        return {feasible, limited, stopped, settled};
    }
    Progress advance(Hold, const resources::Measurement &measured, double) {
        hold();
        return progress(true, false, measured);
    }
    Progress advance(Brake, const resources::Measurement &measured, double dt) {
        desired_.setZero();
        return progress(profile_.advance(q_, v_, a_, desired_, ruckig::ControlInterface::Velocity, dt), false, measured);
    }
    Progress servo(const resources::Measurement &measured, double dt) {
        const auto result = profile_.servo(q_, v_, a_, desired_, dt);
        return progress(result != ServoProgress::Unavailable, result == ServoProgress::Limited, measured);
    }
    Progress advance(const Stream &demand, const resources::Measurement &measured, double dt) {
        std::visit([&](const auto &target) { targetVelocity(target); }, demand.target.get());
        return servo(measured, dt);
    }
    Progress advance(const Jog &demand, const resources::Measurement &measured, double dt) {
        desired_.setZero();
        std::visit([&](const auto &target) { targetVelocity(target); }, demand.input.get().jog);
        return servo(measured, dt);
    }
    Progress advance(const Point &demand, const resources::Measurement &measured, double dt) {
        const auto remaining = demand.remaining > core::duration{} ? std::optional{std::chrono::duration<double>(demand.remaining).count()} : std::nullopt;
        const bool feasible = profile_.advance(q_, v_, a_, demand.target.get().values(), ruckig::ControlInterface::Position, dt, remaining);
        return progress(feasible, false, measured, &demand.target.get().values());
    }
    Progress advance(const Path &demand, const resources::Measurement &measured, double) {
        demand.path.get().sample(demand.elapsed, candidate_q_, candidate_v_, candidate_a_);
        const bool feasible = profile_.canStop(candidate_q_, candidate_v_, candidate_a_);
        if (feasible) {
            q_ = candidate_q_;
            v_ = candidate_v_;
            a_ = candidate_a_;
        }
        return progress(feasible, false, measured, &demand.path.get().points().back().position.values());
    }
    void targetVelocity(const resources::Positions &target) { desired_ = config_.gain() * (target.values() - q_); }
    void targetVelocity(const resources::Velocities &target) { desired_ = target.values(); }
    void targetVelocity(const communication::Twist &target) { desired_ = ik_.velocity(reference_.position, target.value(), config_.damping()).values(); }
    void targetVelocity(const communication::Pose &target) {
        desired_ = ik_.velocity(reference_.position, target, config_).values();
        for (const auto &auxiliary : target.auxiliary())
            desired_[auxiliary.index] = config_.gain() * (auxiliary.position - q_[auxiliary.index]);
    }
    void targetVelocity(std::monostate) { desired_.setZero(); }
    ControllerConfig config_;
    Solver ik_;
    MotionGenerator profile_;
    Eigen::VectorXd q_, v_, a_, desired_, candidate_q_, candidate_v_, candidate_a_;
    Reference reference_;
};
} // namespace robot::execution
