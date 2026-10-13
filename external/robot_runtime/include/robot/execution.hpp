#pragma once
#include "core.hpp"
#include "robot/backend.hpp"
#include "robot/communication.hpp"
#include "robot/execution/components/control.hpp"
#include "robot/execution/components/planning.hpp"
#include "robot/execution/planner.hpp"
#include "robot/execution/state_machine.hpp"
#include "robot/resources/kinematics.hpp"
namespace robot {
class Execution final {
public:
    Execution(Backend backend, execution::ControllerConfig config, core::Period period, std::optional<resources::Positions> standby = {},
              core::Period standby_duration = core::Period{std::chrono::seconds(4)})
        : backend_(std::move(backend)), config_(config), machine_(config, std::move(standby), standby_duration, backend_.capabilities().binary_tool),
          communication_(std::make_shared<Communication>(config, backend_.capabilities(), period)), kinematics_(backend_.model()), period_(period),
          control_graph_(buildControl()), observation_graph_(buildObservation()) {
        communication_->description_.sensors = backend_.sensorDescriptions();
    }
    Execution(const Execution &) = delete;
    Execution &operator=(const Execution &) = delete;
    Execution(Execution &&) = delete;
    Execution &operator=(Execution &&) = delete;
    // The driver owns both calls and any environment advancement between them.
    void control(const core::Context &ctx, execution::TimingViolation timing = execution::TimingViolation::None) {
        started_ = core::HostClock::now();
        timing_ = timing;
        control_graph_.evaluate(ctx);
    }
    void observe(const core::Context &ctx) { observation_graph_.evaluate(ctx); }
    void bindTiming(communication::TimingProfile profile) { communication_->description_.timing = profile; }
    void reset() {
        communication_->clearInputs();
        machine_.reset();
        control_graph_.reset();
        observation_graph_.reset();
        requested_motion_.reset();
    }
    std::shared_ptr<Communication> communication() const { return communication_; }
    const Model &model() const { return backend_.model(); }
    core::Period controlPeriod() const { return period_; }
    std::string controlDiagram() const { return control_graph_.dot(); }
    std::string observationDiagram() const { return observation_graph_.dot(); }
    std::string stateDiagram() const { return execution::automaton::diagram(); }

private:
    using Input = execution::InputFrame;
    using Previous = execution::ControlFeedback;
    using Authority = execution::AuthorityFrame;
    using Plan = execution::PlanFrame;
    using Directive = execution::ControlFrame;
    using Control = execution::ControlOutput;
    using Delivery = execution::DeliveryOutput;
    Previous initialFeedback() const {
        return {
            {model().joints().zero<resources::Positions>(), model().joints().zero<resources::Velocities>(), model().joints().zero<resources::Accelerations>()},
            {true, false, true, true},
            false};
    }
    Input capture(const core::Context &ctx) {
        Input input{};
        input.now = core::HostClock::now();
        input.feedback = sample(ctx, input.now);
        input.timing = timing_;
        input.disable = communication_->takeDisable();
        input.stop = communication_->takeStop();
        const auto drain = [](auto &destination, auto take) {
            for (auto &slot : destination) {
                slot = take();
                if (!slot)
                    break;
            }
        };
        drain(input.operators, [&] { return communication_->takeOperator(); });
        input.renewal = communication_->renewal();
        drain(input.schedules, [&] { return communication_->takeScheduled(); });
        input.command = communication_->takeCommand();
        for (const auto &proposal : input.schedules)
            if (proposal)
                requested_motion_ = proposal;
        if (input.command)
            requested_motion_ = input.command;
        input.direct = communication_->directInput();
        return input;
    }
    core::Graph buildControl() {
        core::GraphBuilder builder;
        const auto previous = initialFeedback();
        const Input initial_input{};
        const Authority initial_authority{execution::Mode::Disarmed, false, false, {}, communication::DirectInput{}};
        const Plan initial_plan{};
        const Directive initial_directive{execution::Brake{}, false, execution::Mode::Disarmed, {}};
        const Control initial_control{
            {backend::Actuation::Idle, previous.reference.position, previous.reference.velocity, {}, 0, previous.reference.acceleration}, previous};
        const Delivery initial_delivery{initial_control, backend::UnreportedDelivery{}};
        auto &capture_stage = builder.makeComponent<core::Transform<Input>>("input", initial_input, std::tuple{}, [this](auto &ctx) { return capture(ctx); });
        auto &delay = builder.makeComponent<core::Delay<Previous>>("controller feedback", previous);
        auto &authority = builder.makeComponent<core::Transform<Authority, Input, Previous>>(
            "authority", initial_authority, std::tuple{initial_input, previous}, [this](auto &ctx, auto &input, auto &feedback) {
                auto out = machine_.begin(input, feedback, ctx);
                machine_.clearConsumed();
                return out;
            });
        auto &planning =
            builder.makeComponent<execution::components::Planning>("planning", config_, machine_.standbyDuration(), initial_input, initial_authority, previous);
        auto &commit = builder.makeComponent<core::Transform<Directive, Plan, Authority, Previous>>(
            "admission", initial_directive, std::tuple{initial_plan, initial_authority, previous},
            [this](auto &, auto &plan, auto &owner, auto &feedback) { return machine_.commit(plan, owner, feedback); });
        auto &controller = builder.makeComponent<execution::components::Control>("controller", config_, initial_control, initial_input, initial_directive);
        auto &delivery = builder.makeComponent<core::Transform<Delivery, Control>>(
            "backend delivery", initial_delivery, std::tuple{initial_control}, [this](auto &, auto &control) {
                auto out = Delivery{control, backend_.write(control.request)};
                if (auto *transport = std::get_if<backend::TransportDelivery>(&out.delivery))
                    out.control.feedback.transport_fault = transport->fault;
                return out;
            });
        auto &feedback = builder.makeComponent<core::Transform<Previous, Delivery>>("controller result", previous, std::tuple{initial_delivery},
                                                                                    [](auto &, auto &delivered) { return delivered.control.feedback; });
        capture_stage.output<Input>() >> authority.input<Input>();
        capture_stage.output<Input>() >> planning.input<Input>();
        capture_stage.output<Input>() >> controller.input<Input>();
        delay.output<Previous>() >> authority.input<Previous>();
        delay.output<Previous>() >> planning.input<Previous>();
        delay.output<Previous>() >> commit.input<Previous>();
        authority.output<Authority>() >> planning.input<Authority>();
        authority.output<Authority>() >> commit.input<Authority>();
        planning.output<Plan>() >> commit.input<Plan>();
        commit.output<Directive>() >> controller.input<Directive>();
        controller.output<Control>() >> delivery.input<Control>();
        delivery.output<Delivery>() >> feedback.input<Delivery>();
        feedback.output<Previous>() >> delay.input();
        delivered_ = &delivery.output<Delivery>();
        return std::move(builder).build();
    }
    backend::BackendObservation sample(const core::Context &ctx, core::HostClock::time_point now) {
        auto feedback = backend_.read(now);
        const auto stamp = [&](backend::SampleStamp &s) {
            if (s.epoch == 0) s.epoch = ctx.epoch;
            // External acquisition already identifies its timeline. Retaining it
            // makes delayed samples visible as old after a simulation reset.
            // Source/time correlation belongs to the backend or driver, not to
            // an interpretation of a clock label inside Execution.
        };
        if (feedback.state)
            stamp(feedback.state->stamp);
        for (auto &sensor : feedback.sensors)
            stamp(sensor.stamp);
        return feedback;
    }
    core::Graph buildObservation() {
        core::GraphBuilder builder;
        auto &read = builder.makeComponent<core::Transform<backend::BackendObservation>>("backend observation", backend::BackendObservation{}, std::tuple{},
                                                                                         [this](auto &ctx) { return sample(ctx, core::HostClock::now()); });
        auto &publish = builder.makeComponent<core::Transform<execution::Snapshot, backend::BackendObservation>>(
            "observation publication", communication_->latest(), std::tuple{backend::BackendObservation{}}, [this](const auto &ctx, const auto &measured) {
                const auto now = core::HostClock::now();
                const auto &delivery = delivered_->read();
                auto status = machine_.status();
                status.motion_limited =
                    delivery.control.feedback.progress.limited && (status.mode == execution::Mode::Autonomous || status.mode == execution::Mode::Direct);
                if (status.mode != last_status_.mode || status.owner != last_status_.owner || status.generation != last_status_.generation) {
                    const auto n = ++transition_sequence_;
                    transitions_[(n - 1) % transitions_.size()] = {
                        n, last_status_.mode, status.mode, status.owner, status.generation, status.interruption, ctx.end, ctx.epoch};
                }
                status.transition_sequence = transition_sequence_;
                status.transitions = transitions_;
                last_status_ = status;
                execution::Snapshot out{measured,
                                        std::move(status),
                                        delivery.control.request,
                                        {},
                                        ctx.end,
                                        ctx.epoch,
                                        now,
                                        delivery.delivery,
                                        ctx.elapsed(),
                                        last_cycle_ ? started_ - *last_cycle_ : core::duration{},
                                        now - started_};
                if (out.backend.state)
                    out.tool_pose = execution::DerivedToolPose{kinematics_.calculate(out.backend.state->joints.positions()).pose, out.backend.state->stamp};
                out.requested_motion = requested_motion_;
                out.observation_sequence = ++observation_sequence_;
                last_cycle_ = started_;
                communication_->publish(out);
                return out;
            });
        read.output<backend::BackendObservation>() >> publish.input<backend::BackendObservation>();
        return std::move(builder).build();
    }
    Backend backend_;
    execution::ControllerConfig config_;
    execution::StateMachine machine_;
    std::shared_ptr<Communication> communication_;
    resources::Workspace kinematics_;
    core::Period period_;
    core::OutputChannel<Delivery> *delivered_{};
    execution::TimingViolation timing_{execution::TimingViolation::None};
    core::HostClock::time_point started_{};
    std::optional<core::HostClock::time_point> last_cycle_;
    execution::Status last_status_;
    std::uint64_t transition_sequence_{};
    std::uint64_t observation_sequence_{};
    std::array<execution::AuthorityTransition, 32> transitions_{};
    std::optional<communication::Command> requested_motion_;
    core::Graph control_graph_, observation_graph_;
};
} // namespace robot
