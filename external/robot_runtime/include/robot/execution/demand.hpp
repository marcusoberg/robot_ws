#pragma once
#include "robot/communication/messages.hpp"
#include "robot/execution/trajectory.hpp"
#include <concepts>
#include <functional>
#include <variant>

namespace robot::execution {
using StreamTarget = std::variant<resources::Positions, resources::Velocities, communication::Pose, communication::Twist>;
struct Hold {};
struct Brake {};
struct Stream {
    std::reference_wrapper<const StreamTarget> target;
};
struct Jog {
    std::reference_wrapper<const communication::DirectInput> input;
};
struct Point {
    std::reference_wrapper<const resources::Positions> target;
    core::duration remaining;
};
struct Path {
    std::reference_wrapper<const JointPath> path;
    core::duration elapsed;
};
using Demand = std::variant<Hold, Brake, Stream, Jog, Point, Path>;
struct Reference {
    resources::Positions position;
    resources::Velocities velocity;
    resources::Accelerations acceleration;
};
struct Progress {
    bool feasible;
    bool limited;
    bool stopped;
    bool settled;
};
template <class T>
concept CartesianSolver = std::constructible_from<T, Model> && requires(T &solver, const resources::Positions &q, const communication::Pose &pose,
                                                                        const communication::Twist &twist, const ControllerConfig &config) {
    { solver.velocity(q, pose, config) } -> std::same_as<resources::Velocities>;
    { solver.velocity(q, twist.value(), config.damping()) } -> std::same_as<resources::Velocities>;
};
} // namespace robot::execution
