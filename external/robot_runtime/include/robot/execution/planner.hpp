#pragma once
#include "robot/execution/demand.hpp"
#include "robot/execution/scheduled_motion.hpp"
#include "robot/execution/state.hpp"
#include <stdexcept>

namespace robot::execution {
// Owns motion plans and their execution cursor. Preparation consumes decoded
// path points; admission compares a plan with the current execution reference.
class Planner final {
public:
    explicit Planner(const ControllerConfig &config) : config_(config) {}
    Trajectory trajectory(PathPoints points) const { return Trajectory{path(std::move(points))}; }
    ScheduledMotion schedule(PathPoints points, core::Time start, std::uint64_t epoch, std::optional<communication::Id> replaces, std::vector<GripperPoint> gripper = {}) const {
        return ScheduledMotion{start, epoch, replaces, path(std::move(points)), std::move(gripper)};
    }
    template <class Target>
        requires(std::same_as<Target, resources::Positions> || std::same_as<Target, resources::Velocities> || std::same_as<Target, communication::Pose> ||
                 std::same_as<Target, communication::Twist>)
    void start(const Target &target, communication::Reference reference, core::Time) {
        phase_ = Streaming{target, reference};
    }
    void start(const Trajectory &target, communication::Reference reference, core::Time time) { phase_ = FiniteMove{target, time, {}, reference}; }
    void standby(resources::Positions target, core::Time start, core::duration duration) { phase_ = FiniteMove{std::move(target), start, duration, {}}; }
    RejectReason admit(const Reference &current_reference, Mode mode, const execution::ScheduledMotion &motion, communication::Reference reference,
                       const core::Context &ctx) {
        if (motion.epoch() != ctx.epoch)
            return RejectReason::Stale;
        if (motion.start() < ctx.end + 2 * ctx.elapsed())
            return RejectReason::Late;
        if (motion.end() > ctx.end + execution::ScheduledMotion::maximum_horizon)
            return RejectReason::Infeasible;
        const auto &first = motion.path().points().front();
        bool continuous;
        auto *current = std::get_if<Scheduled>(&phase_);
        if (current) {
            if (current->view.pending)
                return RejectReason::QueueFull;
            const auto &previous = current->view.current;
            if (motion.replaces() != previous.reference.id || motion.start() < previous.motion.start() || motion.start() > previous.motion.end())
                return RejectReason::Conflict;
            continuous = canJoin(first, previous.motion.path(), motion.start() - previous.motion.start());
        } else {
            if (mode != Mode::Hold || motion.replaces())
                return RejectReason::Conflict;
            continuous = canJoin(first, current_reference);
        }
        if (!continuous)
            return RejectReason::Discontinuous;
        if (current) {
            current->view.pending = ScheduledReference{motion, reference};
        } else {
            phase_ = Scheduled{{{motion, reference}, {}}};
        }
        return RejectReason::None;
    }
    // Advance the committed cursor before checking underrun, so a replacement
    // beginning on this tick takes effect atomically.
    bool advance(const core::Context &ctx, std::optional<communication::Reference> &canceled) {
        auto *scheduled = std::get_if<Scheduled>(&phase_);
        if (!scheduled)
            return false;
        auto &view = scheduled->view;
        if (view.pending && ctx.end >= view.pending->motion.start()) {
            canceled = view.current.reference;
            view.current = std::move(*view.pending);
            view.pending.reset();
        }
        return ctx.end > view.current.motion.end();
    }
    Demand demand(const core::Context &ctx) const {
        return std::visit(
            [&](const auto &plan) -> Demand {
                using T = std::remove_cvref_t<decltype(plan)>;
                if constexpr (std::same_as<T, Streaming>)
                    return Stream{plan.target};
                else if constexpr (std::same_as<T, Scheduled>) {
                    const auto &current = plan.view.current.motion;
                    if (ctx.end < current.start())
                        return Hold{};
                    return Path{current.path(), ctx.end - current.start()};
                } else if constexpr (std::same_as<T, FiniteMove>) {
                    return std::visit(
                        [&](const auto &target) -> Demand {
                            if constexpr (std::same_as<std::remove_cvref_t<decltype(target)>, resources::Positions>)
                                return Point{target, std::max(core::duration{}, plan.minimum_duration - (ctx.begin - plan.start))};
                            else
                                return Path{target.path(), ctx.end - plan.start};
                        },
                        plan.motion);
                } else
                    return Brake{};
            },
            phase_);
    }
    std::optional<bool> gripper(const core::Context &ctx) const {
        if (const auto *scheduled = std::get_if<Scheduled>(&phase_))
            return scheduled->view.current.motion.gripper(ctx.end);
        return {};
    }
    bool finished(const core::Context &ctx) const {
        const auto *move = std::get_if<FiniteMove>(&phase_);
        return move && std::visit(
                           [&](const auto &target) {
                               if constexpr (std::same_as<std::remove_cvref_t<decltype(target)>, resources::Positions>)
                                   return true;
                               else
                                   return ctx.end - move->start >= target.path().duration();
                           },
                           move->motion);
    }
    std::optional<ScheduleSnapshot> schedule() const {
        if (const auto *scheduled = std::get_if<Scheduled>(&phase_))
            return scheduled->view;
        return {};
    }
    std::optional<communication::Reference> active() const {
        return std::visit(
            [](const auto &plan) -> std::optional<communication::Reference> {
                using T = std::remove_cvref_t<decltype(plan)>;
                if constexpr (std::same_as<T, std::monostate>)
                    return {};
                else if constexpr (std::same_as<T, Scheduled>)
                    return plan.view.current.reference;
                else
                    return plan.reference;
            },
            phase_);
    }
    std::optional<communication::Reference> canceling() const {
        const auto current = schedule();
        if (current && current->pending)
            return current->pending->reference;
        return active();
    }
    void clear() { phase_ = std::monostate{}; }

private:
    static bool canJoin(const ReferencePoint &first, const Reference &reference) {
        return (first.position.values() - reference.position.values()).cwiseAbs().maxCoeff() <= 1e-7 &&
               (first.velocity.values() - reference.velocity.values()).cwiseAbs().maxCoeff() <= 1e-7 &&
               (first.acceleration.values() - reference.acceleration.values()).cwiseAbs().maxCoeff() <= 1e-7;
    }
    bool canJoin(const ReferencePoint &first, const JointPath &path, core::duration elapsed) const {
        Eigen::VectorXd q(config_.model().dof()), v(q), a(q);
        path.sample(elapsed, q, v, a);
        return (first.position.values() - q).cwiseAbs().maxCoeff() <= 1e-7 && (first.velocity.values() - v).cwiseAbs().maxCoeff() <= 1e-7 &&
               (first.acceleration.values() - a).cwiseAbs().maxCoeff() <= 1e-7;
    }
    // Latest targets are followed until replaced, released or expired. Reaching
    // one sample never completes a stream or resets its motion reference.
    struct Streaming {
        execution::StreamTarget target;
        communication::Reference reference;
    };
    // Only an explicitly timed trajectory or the operator's standby move has
    // an endpoint with completion semantics and a transition back to HOLD.
    struct FiniteMove {
        std::variant<resources::Positions, execution::Trajectory> motion;
        core::Time start;
        core::duration minimum_duration{};
        std::optional<communication::Reference> reference;
    };
    struct Scheduled {
        ScheduleSnapshot view;
    };
    using Phase = std::variant<std::monostate, Streaming, FiniteMove, Scheduled>;
    JointPath path(PathPoints input_points) const {
        auto prepared = std::make_shared<JointPath::Prepared>();
        prepared->points = std::move(input_points.points_);
        prepared->segments.reserve(prepared->points.size() - 1);
        MotionSolver solver(config_.model().dof());
        auto input = motionInput(config_);
        for (std::size_t i = 1; i < prepared->points.size(); ++i) {
            const auto &from = prepared->points[i - 1];
            const auto &to = prepared->points[i];
            input.current_position = from.position.values();
            input.current_velocity = from.velocity.values();
            input.current_acceleration = from.acceleration.values();
            input.target_position = to.position.values();
            input.target_velocity = to.velocity.values();
            input.target_acceleration = to.acceleration.values();
            const double duration = std::chrono::duration<double>(to.offset - from.offset).count();
            input.minimum_duration = duration;
            MotionCurve curve(config_.model().dof());
            // Ruckig calculate already validates its inputs; do not run a second pass.
            if (solver.calculate(input, curve) < 0 || std::abs(curve.get_duration() - duration) > 1e-8 || !insideBounds(curve, config_.model()))
                throw std::invalid_argument("requested path segment is physically infeasible");
            prepared->segments.push_back(std::move(curve));
        }
        return JointPath{std::move(prepared)};
    }
    // The planner outlives the component constructor's configuration argument.
    // Own the immutable value; Model retains shared deployment resources.
    const ControllerConfig config_;
    Phase phase_;
};
} // namespace robot::execution
