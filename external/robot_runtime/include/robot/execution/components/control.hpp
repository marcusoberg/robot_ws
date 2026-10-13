#pragma once
#include "core/transform.hpp"
#include "robot/execution/controller.hpp"
#include "robot/execution/frames.hpp"
namespace robot::execution::components {
class Control final : public core::Transform<ControlOutput, InputFrame, ControlFrame> {
public:
    Control(ControllerConfig config, ControlOutput initial, InputFrame input, ControlFrame directive)
        : core::Transform<ControlOutput, InputFrame, ControlFrame>(
              initial, std::tuple{input, directive},
              [this](const auto &ctx, const auto &input, const auto &directive) { return evaluate(ctx, input, directive); }),
          controller_(std::move(config)) {}

private:
    ControlOutput evaluate(const core::Context &ctx, const InputFrame &input, const ControlFrame &directive) {
        Progress progress{true, false, true, true};
        bool abort = false;
        if (input.feedback.state) {
            const auto &measured = input.feedback.state->joints;
            if (directive.capture)
                controller_.capture(measured);
            if (directive.mode == Mode::Disarmed)
                controller_.idle(measured);
            else if (directive.mode == Mode::Fault)
                controller_.hold();
            else if (!controller_.trackingHealthy(measured))
                progress = {false, false, false, false};
            else {
                progress = controller_.step(directive.demand, measured, ctx.getDt());
                if (!progress.feasible) {
                    abort = true;
                    progress = controller_.step(Brake{}, measured, ctx.getDt());
                }
            }
        }
        const auto &ref = controller_.reference();
        const auto action = directive.mode == Mode::Disarmed                      ? directive.disarmed_actuation
                            : directive.mode == Mode::Fault || !progress.feasible ? backend::Actuation::Inhibit
                                                                                  : backend::Actuation::Position;
        return {{action, ref.position, ref.velocity, directive.gripper, ctx.seq + 1, ref.acceleration}, {ref, progress, false, abort}};
    }
    Controller<> controller_;
};
} // namespace robot::execution::components
