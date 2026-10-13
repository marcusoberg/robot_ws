#pragma once
#include "robot/communication/cartesian.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/resources/kinematics.hpp"
#include <Eigen/Cholesky>

namespace robot::execution {
// Cartesian geometry only. No ownership, transport, timing or state transitions.
class IK {
public:
    explicit IK(Model model) : model_(std::move(model)), workspace_(model_) {}
    [[nodiscard]] resources::Velocities velocity(const resources::Positions &reference, const Eigen::Matrix<double, 6, 1> &twist, double damping) {
        const auto &geometry = workspace_.calculate(reference);
        Eigen::Matrix<double, 6, 6> system = geometry.jacobian * geometry.jacobian.transpose();
        system.diagonal().array() += damping * damping;
        const Eigen::VectorXd solved = geometry.jacobian.transpose() * system.ldlt().solve(twist);
        return model_.joints().generate<resources::Velocities>([&](auto i) { return solved[i]; });
    }
    [[nodiscard]] resources::Velocities velocity(const resources::Positions &reference, const communication::Pose &goal,
                                                 const execution::ControllerConfig &config) {
        const auto &geometry = workspace_.calculate(reference);
        Eigen::Matrix<double, 6, 1> twist;
        twist.head<3>() = config.gain() * (goal.value().translation() - geometry.pose.translation());
        const Eigen::AngleAxisd rotation(goal.value().linear() * geometry.pose.linear().transpose());
        twist.tail<3>() = config.orientationGain() * rotation.axis() * rotation.angle();
        return velocity(reference, twist, config.damping());
    }

private:
    Model model_;
    resources::Workspace workspace_;
};
} // namespace robot::execution
