#pragma once
#include "core/mailbox.hpp"
#include "robot/backend/concepts.hpp"
#include "robot/communication/messages.hpp"
#include "robot/communication/types.hpp"
#include "robot/execution/state.hpp"

namespace robot {
class Execution;
namespace execution {
struct InputFrame;
}
// Transport-independent exchange. Adapters share this object, never an Execution or
// controller. One execution owner drains the inboxes and publishes coherent State.
class Communication final {
public:
    Communication(execution::ControllerConfig config, backend::BackendCapabilities capabilities, core::Period period)
        : config_(std::move(config)), capabilities_(capabilities), period_(period), description_{config_.model(), capabilities, period},
          observations_(execution::Snapshot{.actuator_request = {.position = model().joints().zero<resources::Positions>(),
                                                                 .velocity = model().joints().zero<resources::Velocities>(),
                                                                 .acceleration = model().joints().zero<resources::Accelerations>()}}) {}
    [[nodiscard]] const communication::Description &description() const { return description_; }
    [[nodiscard]] const Model &model() const { return config_.model(); }
    [[nodiscard]] const execution::ControllerConfig &controllerConfig() const { return config_; }
    [[nodiscard]] backend::BackendCapabilities capabilities() const { return capabilities_; }
    [[nodiscard]] core::Period controlPeriod() const { return period_; }
    [[nodiscard]] execution::Snapshot latest() const { return observations_.read(); }
    bool submit(communication::Command message) {
        if (std::holds_alternative<execution::ScheduledMotion>(message.motion))
            return scheduled_.push(std::move(message));
        commands_.publish(std::move(message));
        return true;
    }
    bool renew(communication::AuthorityRenewal renewal) {
        const auto status = latest().status;
        const auto now = core::HostClock::now();
        if (status.owner != renewal.session || !status.authority_expiry || renewal.expiry.at <= now)
            return false;
        std::scoped_lock lock(renewal_mutex_);
        const bool same = renewal_window_ && renewal_window_->session() == renewal.session;
        const auto through = same ? std::max(status.authority_expiry->at, renewal_window_->through().at) : status.authority_expiry->at;
        if (now >= through)
            return false;
        renewal_window_ = communication::RenewalWindow{renewal.session, same ? renewal_window_->since() : now, {std::max(through, renewal.expiry.at)}};
        return true;
    }
    bool request(communication::OperatorRequest request) {
        switch (request.action) {
        case communication::OperatorAction::Stop:
        case communication::OperatorAction::Hold:
            return stops_.push(request);
        case communication::OperatorAction::Disable:
            return disables_.push(request);
        default:
            return operators_.push(std::move(request));
        }
    }
    void directInput(communication::DirectInput input) { direct_.publish(std::move(input)); }

private:
    friend class Execution;
    // Adapters cannot drain channels or publish robot state. Only the execution
    // owner has that capability; it creates and retains this exchange.
    [[nodiscard]] std::optional<communication::RenewalWindow> renewal() const {
        std::scoped_lock lock(renewal_mutex_);
        return renewal_window_;
    }
    [[nodiscard]] auto takeCommand() { return commands_.exchange({}); }
    [[nodiscard]] auto takeScheduled() { return scheduled_.pop(); }
    [[nodiscard]] auto takeOperator() { return operators_.pop(); }
    [[nodiscard]] auto takeStop() { return stops_.pop(); }
    [[nodiscard]] auto takeDisable() { return disables_.pop(); }
    [[nodiscard]] auto directInput() const { return direct_.read(); }
    void publish(execution::Snapshot snapshot) { observations_.publish(std::move(snapshot)); }

    void clearInputs() {
        commands_.publish(std::nullopt);
        direct_.publish(communication::DirectInput{});
        operators_.clear();
        stops_.clear();
        disables_.clear();
        scheduled_.clear();
        {
            std::scoped_lock lock(renewal_mutex_);
            renewal_window_.reset();
        }
    }
    execution::ControllerConfig config_;
    backend::BackendCapabilities capabilities_;
    core::Period period_;
    communication::Description description_;
    core::Snapshot<std::optional<communication::Command>> commands_{std::nullopt};
    mutable std::mutex renewal_mutex_;
    std::optional<communication::RenewalWindow> renewal_window_;
    core::Inbox<communication::Command, 8> scheduled_;
    core::Inbox<communication::OperatorRequest> operators_;
    core::Inbox<communication::OperatorRequest, 1> stops_, disables_;
    core::Snapshot<communication::DirectInput> direct_{communication::DirectInput{}};
    core::Snapshot<execution::Snapshot> observations_;
};
} // namespace robot
