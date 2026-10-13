#pragma once
#include <algorithm>

#include "core/types.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/execution/motion_generator.hpp"

namespace robot::execution {
struct ReferencePoint {
    resources::Positions position;
    resources::Velocities velocity;
    resources::Accelerations acceleration;
    core::duration offset;
};

class Planner;
} // namespace robot::execution
namespace robot::communication {
class Decoder;
}
namespace robot::execution {
// Only the receive decoder can construct a path request. It establishes shape,
// ordering and time origin once; the planner consumes that established contract.
class PathPoints {
private:
    friend class communication::Decoder;
    friend class Planner;
    explicit PathPoints(std::vector<ReferencePoint> points) : points_(std::move(points)) {}
    std::vector<ReferencePoint> points_;
};
// An immutable, exact-time joint reference. Preparation occurs outside execution.
// Derivatives belong to the path, including at joins; no implicit retiming or
// second target-chasing limiter is applied when the prepared path is sampled.
class JointPath {
public:
    [[nodiscard]] const std::vector<ReferencePoint> &points() const { return prepared_->points; }
    [[nodiscard]] core::duration duration() const { return points().back().offset; }
    void sample(core::duration elapsed, Eigen::VectorXd &position, Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration) const {
        const auto &waypoints = points();
        auto next = std::upper_bound(waypoints.begin(), waypoints.end(), elapsed, [](auto time, const ReferencePoint &p) { return time < p.offset; });
        const auto segment = std::clamp<std::size_t>(std::distance(waypoints.begin(), next), 1, waypoints.size() - 1) - 1;
        const auto time = std::clamp(elapsed, waypoints[segment].offset, waypoints[segment + 1].offset);
        prepared_->segments[segment].at_time(std::chrono::duration<double>(time - waypoints[segment].offset).count(), position, velocity, acceleration);
        if (time == waypoints[segment + 1].offset) {
            position = waypoints[segment + 1].position.values();
            velocity = waypoints[segment + 1].velocity.values();
            acceleration = waypoints[segment + 1].acceleration.values();
        }
    }

private:
    friend class Planner;
    struct Prepared {
        std::vector<ReferencePoint> points;
        std::vector<MotionCurve> segments;
    };
    explicit JointPath(std::shared_ptr<const Prepared> prepared) : prepared_(std::move(prepared)) {}
    std::shared_ptr<const Prepared> prepared_;
};

// A finite move is a path whose start/end are at rest. Rolling schedules use
// JointPath directly and may replace a future suffix while already moving.
class Trajectory {
public:
    [[nodiscard]] const auto &points() const { return path_.points(); }
    [[nodiscard]] const JointPath &path() const { return path_; }
    void sample(core::duration elapsed, Eigen::VectorXd &position, Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration) const {
        path_.sample(elapsed, position, velocity, acceleration);
    }

private:
    friend class Planner;
    explicit Trajectory(JointPath path) : path_(std::move(path)) {}
    JointPath path_;
};
} // namespace robot::execution
