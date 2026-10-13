#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <span>
#include <stdexcept>
#include <utility>

#include "robot/boundary/joints.hpp"
#include "robot/communication/messages.hpp"
#include "robot/communication/requests.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/execution/planner.hpp"
#include "robot/execution/state.hpp"
#include "robot/execution/trajectory.hpp"
#include "robot/execution/timed_targets.hpp"
#include "robot/resources/model.hpp"

namespace robot::communication {
// One conversion into model-bound commands, shared by every communication adapter.
// Overloads are exhaustive: a new MotionRequest alternative needs a matching parser.
class Decoder {
public:
    Decoder(const execution::ControllerConfig &config, const execution::Snapshot &observation, execution::TargetFitReport *fit=nullptr)
        : config_(config), model_(config.model()), observation_(observation), fit_(fit) {}

    communication::Command operator()(const CommandRequest &request) const {
        if (request.session.source.value == 0 || request.session.generation == 0 || request.id.value == 0)
            throw std::invalid_argument("command requires nonzero source, generation and id");
        return {request.session, request.id, request.expiry,
                std::visit([this](const auto &motion) -> communication::Motion { return (*this)(motion); }, request.motion), request.gripper_open};
    }
    resources::Positions operator()(const JointPositionRequest &request) const { return joints<resources::Positions>(request.order, request.values); }
    resources::Velocities operator()(const JointVelocityRequest &request) const { return joints<resources::Velocities>(request.order, request.values); }
    communication::Pose operator()(const PoseRequest &request) const {
        baseFrame(request.frame);
        const auto &p = request.position;
        const auto &q = request.quaternion_wxyz;

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        for (int i = 0; i < 3; ++i) {
            position[i] = boundary::FiniteNumber::read(p[i]).value();
        }
        orientation = {boundary::FiniteNumber::read(q[0]).value(), boundary::FiniteNumber::read(q[1]).value(), boundary::FiniteNumber::read(q[2]).value(),
                       boundary::FiniteNumber::read(q[3]).value()};
        const double norm = orientation.norm();
        if (!std::isfinite(norm) || norm < 1e-12)
            throw std::invalid_argument("invalid quaternion");
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = position;
        transform.linear() = orientation.normalized().toRotationMatrix();
        std::vector<communication::Pose::AuxiliaryPosition> auxiliary;
        const auto &description = model_.description();
        for (const auto &[name, raw] : request.auxiliary_positions) {
            if (std::ranges::find(description.auxiliary_joints, name) == description.auxiliary_joints.end())
                throw std::invalid_argument("Cartesian auxiliary demand names a non-auxiliary joint: " + name);
            const auto index = std::ranges::find(description.joints, name) - description.joints.begin();
            const double value = boundary::FiniteNumber::read(raw).value();
            if (value < description.lower[index] || value > description.upper[index])
                throw std::invalid_argument("auxiliary demand exceeds configured joint bounds: " + name);
            auxiliary.push_back({index, name, value});
        }
        return communication::Pose{transform, std::move(auxiliary)};
    }
    communication::Twist operator()(const TwistRequest &request) const {
        baseFrame(request.frame);
        Eigen::Matrix<double, 6, 1> values;
        for (int i = 0; i < 6; ++i)
            values[i] = boundary::FiniteNumber::read(request.values[i]).value();
        return communication::Twist{values};
    }
    execution::Trajectory operator()(const TrajectoryRequest &request) const {
        validateOffsets(request.points, core::duration{});
        for (const auto *endpoint : {&request.points.front(), &request.points.back()})
            if (endpoint->velocity && std::ranges::any_of(*endpoint->velocity, [](double v) { return std::abs(v) > 1e-9; }))
                throw std::invalid_argument("finite trajectory must start and finish at rest");
        std::vector<execution::ReferencePoint> points;
        for (const auto &point : request.points)
            points.push_back({joints<resources::Positions>(request.order, point.position),
                              point.velocity ? joints<resources::Velocities>(request.order, *point.velocity) : model_.joints().zero<resources::Velocities>(),
                              model_.joints().zero<resources::Accelerations>(), point.offset});
        return execution::Planner{config_}.trajectory(execution::PathPoints{std::move(points)});
    }
    execution::ScheduledMotion operator()(const ScheduleRequest &request) const {
        validateOffsets(request.points, std::nullopt);
        if (request.replaces && request.replaces->value == 0)
            throw std::invalid_argument("invalid schedule predecessor");
        const auto horizon = execution::ScheduledMotion::maximum_horizon;
        if (request.start.time_since_epoch() > core::duration::max() - horizon || request.epoch != observation_.epoch ||
            request.start <= observation_.control_time || request.points.back().offset > horizon ||
            request.start - observation_.control_time > horizon - request.points.back().offset)
            throw std::invalid_argument("schedule must fit a future horizon in the current epoch");
        auto anchor = scheduleAnchor(request);
        std::vector<execution::ReferencePoint> points;
        points.push_back(std::move(anchor));
        for (const auto &point : request.points)
            points.push_back({joints<resources::Positions>(request.order, point.position), joints<resources::Velocities>(request.order, point.velocity),
                              joints<resources::Accelerations>(request.order, point.acceleration), point.offset});
        return execution::Planner{config_}.schedule(execution::PathPoints{std::move(points)}, request.start, request.epoch, request.replaces);
    }

    execution::ScheduledMotion operator()(const TimedJointTargetsRequest &request) const {
        if(request.epoch!=observation_.epoch) throw std::invalid_argument("timed targets belong to a previous timeline");
        if(request.positions.size()<5 || request.positions.size()>TimedJointTargetsRequest::maximum_samples)
            throw std::invalid_argument("timed targets require 5..256 rows");
        Eigen::MatrixXd values(request.positions.size(),model_.dof());
        for(std::size_t i=0;i<request.positions.size();++i)
            values.row(i)=joints<resources::Velocities>(request.order,request.positions[i]).values().transpose();
        auto fitted=execution::fitTimedTargets(request,config_,values);
        if(fit_) *fit_=fitted.report;
        return (*this)(fitted.schedule);
    }
private:
    // Structural checks occur only here, before any domain path exists. Transport
    // byte limits bound allocation; there is no unrelated waypoint-count limit.
    template <class Points> static void validateOffsets(const Points &points, std::optional<core::duration> origin) {
        if (points.empty())
            throw std::invalid_argument("path needs points");
        if (origin) {
            if (points.size() == 1 || points.front().offset != *origin)
                throw std::invalid_argument("finite path needs a start at zero and an endpoint");
        } else if (points.front().offset <= core::duration{})
            throw std::invalid_argument("schedule offsets must be positive");
        auto previous = points.front().offset;
        for (auto point = std::next(points.begin()); point != points.end(); ++point) {
            if (point->offset <= previous)
                throw std::invalid_argument("path offsets must increase");
            previous = point->offset;
        }
    }
    execution::ReferencePoint scheduleAnchor(const ScheduleRequest &request) const {
        const auto &output = observation_.actuator_request;
        if (!observation_.status.schedule) {
            if (observation_.status.mode != execution::Mode::Hold || request.replaces)
                throw std::invalid_argument("initial schedule requires unmodified HOLD");
            return {output.position, output.velocity, output.acceleration, {}};
        }
        const auto &[current, pending] = *observation_.status.schedule;
        if (pending || request.replaces != current.reference.id || request.start < current.motion.start() || request.start > current.motion.end())
            throw std::invalid_argument("schedule predecessor or join unavailable");
        Eigen::VectorXd q(model_.dof()), v(model_.dof()), a(model_.dof());
        current.motion.path().sample(request.start - current.motion.start(), q, v, a);
        const auto &space = model_.joints();
        return {space.generate<resources::Positions>([&](auto i) { return q[i]; }),
                space.generate<resources::Velocities>([&](auto i) { return v[i]; }),
                space.generate<resources::Accelerations>([&](auto i) { return a[i]; }),
                {}};
    }
    template <resources::JointQuantity T> T joints(const JointOrder &order, std::span<const double> values) const {
        return std::visit([&](const auto &names) -> T { return ordered<T>(names, values); }, order);
    }
    template <resources::JointQuantity T> T ordered(ModelJointOrder, std::span<const double> values) const { return boundary::readJoints<T>(model_, values); }
    template <resources::JointQuantity T> T ordered(const NamedJointOrder &order, std::span<const double> values) const {
        // Resolve external names once, before constructing a model-sized value.
        auto names = order.names.begin();
        auto coordinates = values.begin();
        std::map<std::string, double> named;
        for (; names != order.names.end() && coordinates != values.end(); ++names, ++coordinates)
            if (!named.emplace(*names, *coordinates).second)
                throw std::invalid_argument("duplicate joint name");
        if (names != order.names.end() || coordinates != values.end())
            throw std::invalid_argument("missing named joint coordinate");
        std::vector<double> ordered;
        for (const auto &name : model_.description().joints) {
            auto found = named.find(name);
            if (found == named.end())
                throw std::invalid_argument("missing configured joint: " + name);
            ordered.push_back(found->second);
            named.erase(found);
        }
        if (!named.empty())
            throw std::invalid_argument("unknown joint name");
        return boundary::readJoints<T>(model_, ordered);
    }
    void baseFrame(const std::string &frame) const {
        if (frame != model_.description().base)
            throw std::invalid_argument("Cartesian commands must name the configured base frame");
    }
    const execution::ControllerConfig &config_;
    const robot::Model &model_;
    const execution::Snapshot &observation_;
    execution::TargetFitReport *fit_;
};
} // namespace robot::communication
