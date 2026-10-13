#pragma once
#include "robot/execution/automaton.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/execution/frames.hpp"
namespace robot::execution {
// Authority only. Planner and Controller communicate through frames, never implementation pointers.
class StateMachine final {
public:
    StateMachine(ControllerConfig config, std::optional<resources::Positions> standby, core::Period standby_duration, bool binary_tool)
        : config_(std::move(config)), standby_(std::move(standby)), standby_duration_(standby_duration), binary_tool_(binary_tool) {}
    void reset() {
        stop();
        machine_.process_event(automaton::Disable{});
        epoch_.reset();
        gripper_.reset();
        disabled_actuation_ = backend::Actuation::Idle;
        was_direct_ = direct_active_ = false;
    }
    Mode mode() const {
        using namespace boost::sml;
        if (machine_.is("Disabled"_s))
            return Mode::Disarmed;
        if (machine_.is("Faulted"_s))
            return Mode::Fault;
        if (machine_.is("Stopping"_s))
            return Mode::Stopping;
        if (machine_.is<decltype(state<automaton::Enabled>)>("Holding"_s))
            return Mode::Hold;
        return direct_active_ ? Mode::Direct : Mode::Autonomous;
    }
    AuthorityFrame begin(const InputFrame &input, const ControlFeedback &previous, const core::Context &ctx) {
        context_ = ctx;
        now_ = input.now;
        capture_ = false;
        if (epoch_ && *epoch_ != ctx.epoch) {
            stop();
            machine_.process_event(automaton::Disable{});
        }
        epoch_ = ctx.epoch;
        const bool overrun = input.timing != TimingViolation::None;
        if (input.timing == TimingViolation::Fault)
            fault(RejectReason::Overrun);
        else if (input.timing == TimingViolation::Brake) {
            stop();
            status_.interruption = RejectReason::Overrun;
        }
        if (previous.abort_motion) {
            status_.interruption = RejectReason::Infeasible;
            stop();
        }
        if (previous.transport_fault || !previous.progress.feasible)
            fault(RejectReason::Infeasible);
        if (input.feedback.state) {
            if (input.feedback.state->fault || input.feedback.state->physical_estop)
                fault(RejectReason::InvalidState);
        } else if (mode() != Mode::Disarmed)
            fault(RejectReason::InvalidState);
        if (mode() == Mode::Stopping && previous.progress.stopped) {
            machine_.process_event(automaton::Complete{});
            completeOperator();
        }
        // The boundary proves uninterrupted renewal; an expired gap cannot be repaired later.
        if (input.renewal && owner_ == input.renewal->session() && lease_ && input.renewal->since() < lease_->at && input.renewal->through().at > now_)
            lease_ = input.renewal->through();
        expireAuthority();
        if (input.disable || input.stop || overrun) {
            for (const auto &op : input.operators)
                if (op)
                    report(*op, communication::OperatorDisposition::Canceled, communication::OperatorReason::Interrupted);
            if (input.disable)
                operate(*input.disable, input.feedback.state);
            else if (input.stop)
                operate(*input.stop, input.feedback.state);
            if (input.disable && input.stop)
                report(*input.stop, communication::OperatorDisposition::Canceled, communication::OperatorReason::Interrupted);
        } else
            for (const auto &op : input.operators)
                if (op)
                    operate(*op, input.feedback.state);
        if (!input.direct.deadman)
            direct_blocked_ = false;
        direct_active_ = input.direct.deadman && input.direct.expires > now_ && !direct_blocked_ && !std::holds_alternative<std::monostate>(input.direct.jog) &&
                         mode() != Mode::Disarmed && mode() != Mode::Fault;
        if (direct_active_ && !was_direct_) {
            invalidateOwner();
            cancelOperator();
            clear_ = true;
            run();
        }
        if (!direct_active_ && was_direct_)
            stop();
        was_direct_ = direct_active_;
        if (stream_expiry_ && expired(*stream_expiry_)) {
            brake(RejectReason::Stale);
            stream_expiry_.reset();
        }
        return {mode(),
                capture_,
                clear_,
                std::exchange(pending_standby_, {}),
                input.direct,
                owner_,
                last_accepted_,
                binary_tool_,
                !input.disable && !input.stop && !overrun && mode() != Mode::Direct};
    }
    void admit(const communication::Command &command, RejectReason reason) {
        if (reason != RejectReason::None) {
            reject(command, reason);
            return;
        }
        last_accepted_ = command.id;
        status_.accepted = communication::Reference{command.session, command.id};
        status_.rejected.reset();
        status_.rejection = RejectReason::None;
        status_.interruption = RejectReason::None;
        if (command.gripper_open)
            gripper_ = command.gripper_open;
        if (std::holds_alternative<ScheduledMotion>(command.motion) || std::holds_alternative<Trajectory>(command.motion))
            stream_expiry_.reset();
        else
            stream_expiry_ = command.expiry;
        run();
    }
    ControlFrame commit(const PlanFrame &plan, const AuthorityFrame &authority, const ControlFeedback &previous) {
        for (const auto &admission : plan.admissions)
            if (admission)
                admit(admission->command, admission->reason);
        status_.active = plan.active;
        status_.schedule = plan.schedule;
        if (plan.canceled)
            status_.canceled = plan.canceled;
        if (plan.underrun)
            brake(RejectReason::BufferUnderrun);
        else if (mode() == Mode::Autonomous && plan.finished && !plan.new_motion && previous.progress.settled) {
            status_.completed = plan.active;
            machine_.process_event(automaton::Complete{});
            clear_ = true;
            stream_expiry_.reset();
            completeOperator();
        }
        status_.motion_limited = previous.progress.limited && (mode() == Mode::Autonomous || mode() == Mode::Direct);
        const Demand demand = mode() == Mode::Direct       ? Demand{Jog{authority.direct}}
                              : mode() == Mode::Hold       ? Demand{Hold{}}
                              : mode() == Mode::Autonomous ? plan.demand
                                                           : Demand{Brake{}};
        return {demand, authority.capture, mode(), gripper_, disabled_actuation_};
    }
    Status status() const {
        auto out = status_;
        out.mode = mode();
        out.owner = owner_;
        out.authority_expiry = lease_;
        out.generation = generation_;
        return out;
    }
    core::Period standbyDuration() const { return standby_duration_; }
    backend::Actuation disabledActuation() const { return disabled_actuation_; }
    void clearConsumed() { clear_ = false; }
    bool needsClear() const { return clear_; }

private:
    void run() {
        if (mode() == Mode::Stopping)
            machine_.process_event(automaton::Resume{});
        machine_.process_event(automaton::Run{});
    }
    bool expired(const communication::Expiry &expiry) const { return communication::expired(expiry, context_, now_); }

    void expireAuthority() {
        if (owner_ && lease_ && now_ >= lease_->at) {
            status_.interruption = RejectReason::Stale;
            stop();
        }
    }
    void brake(RejectReason reason) {
        clear_ = true;
        status_.interruption = reason;
        stream_expiry_.reset();
        machine_.process_event(automaton::Stop{});
    }
    void stop() {
        cancelOperator();
        invalidateOwner();
        direct_blocked_ = true;
        pending_standby_.reset();
        machine_.process_event(automaton::Stop{});
    }
    void fault(RejectReason reason) {
        if (mode() != Mode::Fault)
            invalidateOwner();
        cancelOperator();
        direct_blocked_ = true;
        machine_.process_event(automaton::Fault{});
        status_.interruption = reason;
    }
    bool reject(const communication::Command &command, RejectReason reason) {
        status_.rejected = communication::Reference{command.session, command.id};
        status_.rejection = reason;
        return false;
    }
    void invalidateOwner() {
        if (status_.active)
            status_.canceled = status_.active;
        owner_.reset();
        lease_.reset();
        stream_expiry_.reset();
        ++generation_;
        last_accepted_ = {};
        clear_ = true;
    }
    void operate(const communication::OperatorRequest &op, const std::optional<backend::RobotState> &feedback) {
        using enum communication::OperatorAction;
        using Disposition = communication::OperatorDisposition;
        using Reason = communication::OperatorReason;
        status_.last_operator_request = op.request_id;
        status_.last_operator_source = op.source;
        const auto reject = [&](Reason why) { report(op, Disposition::Rejected, why); };
        switch (op.action) {
        case Enable:
            if (!(mode() == Mode::Disarmed || mode() == Mode::Fault) || !feedback || feedback->fault || feedback->physical_estop)
                return reject(Reason::InvalidState);
            if (!feedback->homed)
                return reject(Reason::NotHomed);
            if ((feedback->joints.position().array() < config_.model().description().lower.array()).any() ||
                (feedback->joints.position().array() > config_.model().description().upper.array()).any())
                return reject(Reason::InvalidState);
            cancelOperator();
            invalidateOwner();
            capture_ = true;
            status_.interruption = RejectReason::None;
            machine_.process_event(automaton::Enable{});
            pending_operator_ = op;
            report(op, Disposition::Accepted);
            return;
        case Disable:
            cancelOperator();
            invalidateOwner();
            pending_standby_.reset();
            direct_blocked_ = true;
            disabled_actuation_ = backend::Actuation::Inhibit;
            machine_.process_event(automaton::Disable{});
            report(op, Disposition::Completed);
            return;
        case Pause:
            if(!op.session_guard || owner_!=op.session_guard || op.session_guard->source!=op.source)
                return reject(Reason::NoAuthority);
            if(!(mode()==Mode::Hold || mode()==Mode::Autonomous || mode()==Mode::Stopping))
                return reject(Reason::InvalidState);
            cancelOperator();
            if(status_.active) status_.canceled=status_.active;
            pending_standby_.reset();
            brake(RejectReason::None);
            pending_operator_=op;
            report(op,Disposition::Accepted);
            return;
        case Stop:
        case Hold:
            if (mode() == Mode::Fault)
                return reject(Reason::InvalidState);
            stop();
            if (mode() == Mode::Stopping) {
                pending_operator_ = op;
                report(op, Disposition::Accepted);
            } else
                report(op, Disposition::Completed);
            return;
        case Acquire:
        case Takeover:
            if (op.source.value == 0 || (op.action == Acquire && (mode() != Mode::Hold || owner_)) ||
                (op.action == Takeover && !(mode() == Mode::Hold || mode() == Mode::Autonomous)))
                return reject(Reason::NoAuthority);
            cancelOperator();
            invalidateOwner();
            owner_ = communication::Session{op.source, generation_};
            lease_ = communication::HostExpiry{now_ + std::chrono::seconds(1)};
            if (op.action == Takeover) {
                brake(RejectReason::None);
                pending_operator_ = op;
                report(op, Disposition::Accepted);
            } else
                report(op, Disposition::Completed);
            return;
        case Release:
            if (!owner_ || owner_->source != op.source || (op.session_guard && owner_ != op.session_guard))
                return reject(Reason::NoAuthority);
            stop();
            pending_operator_ = op;
            report(op, Disposition::Accepted);
            return;
        case Standby:
            if (!standby_)
                return reject(Reason::Unavailable);
            if (mode() == Mode::Disarmed || mode() == Mode::Fault)
                return reject(Reason::InvalidState);
            cancelOperator();
            invalidateOwner();
            pending_standby_ = *standby_;
            pending_operator_ = op;
            report(op, Disposition::Accepted);
            run();
            return;
        case OpenGripper:
        case CloseGripper:
            if (!binary_tool_)
                return reject(Reason::Unavailable);
            if (owner_ && op.source.value != 0 && op.source != owner_->source)
                return reject(Reason::NoAuthority);
            gripper_ = op.action == OpenGripper;
            report(op, Disposition::Completed);
            return;
        }
    }
    void report(const communication::OperatorRequest &op, communication::OperatorDisposition disposition,
                communication::OperatorReason reason = communication::OperatorReason::None) {
        if (op.request_id == 0)
            return;
        const auto seq = ++status_.operator_sequence;
        status_.operator_results[(seq - 1) % status_.operator_results.size()] = {seq, op, disposition, reason,
            disposition==communication::OperatorDisposition::Completed &&
            (op.action==communication::OperatorAction::Acquire || op.action==communication::OperatorAction::Takeover)?owner_:std::nullopt};
    }
    void cancelOperator() {
        if (pending_operator_) {
            report(*pending_operator_, communication::OperatorDisposition::Canceled, communication::OperatorReason::Interrupted);
            pending_operator_.reset();
        }
    }
    void completeOperator() {
        if (pending_operator_) {
            report(*pending_operator_, communication::OperatorDisposition::Completed);
            pending_operator_.reset();
        }
    }
    boost::sml::sm<automaton::Machine> machine_;
    ControllerConfig config_;
    std::optional<resources::Positions> standby_, pending_standby_;
    core::Period standby_duration_;
    bool binary_tool_, capture_{}, clear_{}, direct_active_{}, was_direct_{}, direct_blocked_{};
    backend::Actuation disabled_actuation_{backend::Actuation::Idle};
    std::optional<communication::Session> owner_;
    std::optional<communication::HostExpiry> lease_;
    std::optional<communication::Expiry> stream_expiry_;
    std::optional<std::uint64_t> epoch_;
    std::uint64_t generation_{1};
    communication::Id last_accepted_{};
    Status status_;
    std::optional<bool> gripper_;
    std::optional<communication::OperatorRequest> pending_operator_;
    core::Context context_{};
    core::HostClock::time_point now_{};
};
} // namespace robot::execution
