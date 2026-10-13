#pragma once
#include "core/transform.hpp"
#include "robot/execution/frames.hpp"
#include "robot/execution/planner.hpp"
namespace robot::execution::components {
class Planning final : public core::Transform<PlanFrame, InputFrame, AuthorityFrame, ControlFeedback> {
public:
    Planning(ControllerConfig config, core::Period standby_duration, InputFrame input, AuthorityFrame authority, ControlFeedback previous)
        : core::Transform<PlanFrame, InputFrame, AuthorityFrame, ControlFeedback>(
              PlanFrame{}, std::tuple{input, authority, previous},
              [this](const auto &ctx, const auto &input, const auto &authority, const auto &previous) { return evaluate(ctx, input, authority, previous); }),
          planner_(config), standby_duration_(standby_duration) {}

private:
    PlanFrame evaluate(const core::Context &ctx, const InputFrame &input, const AuthorityFrame &authority, const ControlFeedback &previous) {
        if (authority.clear)
            planner_.clear();
        PlanFrame out{};
        if (authority.standby) {
            planner_.standby(*authority.standby, ctx.begin, standby_duration_.value());
            out.new_motion = true;
        }
        auto mode = authority.mode;
        auto accepted = authority.last_accepted;
        std::size_t next = 0;
        const auto consider = [&](const communication::Command &command) {
            auto reason = RejectReason::None;
            if (!authority.accept_commands)
                reason = RejectReason::InvalidState;
            else if (authority.owner != command.session || command.id.value <= accepted.value || !(mode == Mode::Hold || mode == Mode::Autonomous))
                reason = RejectReason::NoAuthority;
            else if (communication::expired(command.expiry, ctx, input.now))
                reason = RejectReason::Stale;
            else if (command.gripper_open && !authority.binary_tool)
                reason = RejectReason::InvalidState;
            if (reason == RejectReason::None)
                std::visit(
                    [&](const auto &motion) {
                        using T = std::remove_cvref_t<decltype(motion)>;
                        if constexpr (std::same_as<T, ScheduledMotion>)
                            reason = planner_.admit(previous.reference, mode, motion, {command.session, command.id}, ctx);
                        else {
                            if constexpr (std::same_as<T, Trajectory>)
                                if (mode != Mode::Hold ||
                                    (motion.points().front().position.values() - previous.reference.position.values()).cwiseAbs().maxCoeff() > 1e-6) {
                                    reason = RejectReason::Infeasible;
                                    return;
                                }
                            planner_.start(motion, {command.session, command.id}, ctx.begin);
                        }
                    },
                    command.motion);
            out.admissions[next++] = MotionAdmission{command, reason};
            if (reason == RejectReason::None) {
                accepted = command.id;
                mode = Mode::Autonomous;
                out.new_motion = true;
            }
        };
        // Captured requests receive an admission outcome even when supervision
        // blocks motion on this cycle. Consuming them silently leaves callers waiting.
        for (const auto &command : input.schedules)
            if (command)
                consider(*command);
        if (input.command)
            consider(*input.command);
        out.underrun = planner_.advance(ctx, out.canceled);
        out.finished = planner_.finished(ctx);
        out.active = planner_.active();
        out.schedule = planner_.schedule();
        if (out.underrun) {
            planner_.clear();
            out.active.reset();
            out.schedule.reset();
        }
        out.demand = planner_.demand(ctx);
        return out;
    }
    Planner planner_;
    core::Period standby_duration_;
};
} // namespace robot::execution::components
