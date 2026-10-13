#pragma once
#include "application/deployment.hpp"
#include "robot/communication/decoder.hpp"
#include <atomic>
#include <memory>

namespace application {
enum class RunMode { Realtime, Stepped };
struct Submission {
    std::uint64_t command_id{}; bool queued{};
    std::optional<robot::execution::TargetFitReport> fit;
};
// A handle owns its identities and borrows only the transport-neutral exchange.
// Renewal is explicit: no thread automatically keeps an external actor alive.
class Operation;
class ControlSession;
class MotionOperation;
class Source final {
    struct State {
        std::shared_ptr<robot::Communication> channel;
        std::shared_ptr<std::atomic_bool> alive;
        robot::communication::SourceId id;
        RunMode mode;
        std::string runtime_instance, name, instance;
        std::atomic_uint64_t next_request{};
    };
    std::shared_ptr<State> state_;
    void check() const { if(!state_->alive->load()) throw std::logic_error("runtime is closed"); }
public:
    Source(std::shared_ptr<robot::Communication> channel,std::shared_ptr<std::atomic_bool> alive,
           robot::communication::SourceId id,RunMode mode,std::string runtime_instance,std::string name,std::string instance)
        : state_(std::make_shared<State>()) {
        state_->channel=std::move(channel); state_->alive=std::move(alive); state_->id=id; state_->mode=mode;
        state_->runtime_instance=std::move(runtime_instance); state_->name=std::move(name); state_->instance=std::move(instance);
    }
    auto id() const { return state_->id; }
    const auto &name() const { return state_->name; }
    const auto &instance() const { return state_->instance; }
    const auto &runtimeInstance() const { return state_->runtime_instance; }
    robot::execution::Snapshot snapshot() const { check(); auto value=state_->channel->latest(); value.runtime_instance=state_->runtime_instance; return value; }
    std::pair<std::uint64_t,bool> operate(robot::communication::OperatorAction action,std::optional<std::uint64_t> generation={}) const {
        check(); const auto id=++state_->next_request; return {id,state_->channel->request({action,state_->id,id,generation?std::optional{robot::communication::Session{state_->id,*generation}}:std::nullopt})};
    }
    bool renew(std::uint64_t generation,const robot::execution::Snapshot &basis,std::optional<double> valid_for={}) const {
        check();
        if(basis.runtime_instance!=state_->runtime_instance || basis.epoch!=snapshot().epoch || basis.status.owner!=robot::communication::Session{state_->id,generation}) return false;
        const auto lease=valid_for?seconds(*valid_for):state_->channel->description().authority_lease;
        if(lease>state_->channel->description().authority_lease) throw std::invalid_argument("renewal cannot exceed configured authority lease");
        // A fresh explicit renewal call proves caller liveness, even when simulation is paused.
        return state_->channel->renew({{state_->id,generation},++state_->next_request,{core::HostClock::now()+lease}});
    }
    Submission submit(robot::communication::MotionRequest motion,std::uint64_t generation,double valid_for,
                      const robot::execution::Snapshot &basis,std::optional<bool> gripper={}) const {
        check(); const auto duration=seconds(valid_for); const auto id=++state_->next_request;
        const auto current=snapshot();
        if(basis.runtime_instance!=state_->runtime_instance || basis.epoch!=current.epoch || basis.observation_sequence>current.observation_sequence) throw std::invalid_argument("observation belongs to another runtime or timeline");
        robot::execution::TargetFitReport fit; const bool timed=std::holds_alternative<robot::communication::TimedJointTargetsRequest>(motion);
        if(timed) {
            auto &target=std::get<robot::communication::TimedJointTargetsRequest>(motion);
            if(target.lead<core::duration{} || current.control_time.time_since_epoch()>core::duration::max()-target.lead) throw std::invalid_argument("invalid target lead");
            if(target.start==core::Time{}) target.start=current.control_time+target.lead;
            if(target.runtime_instance!=state_->runtime_instance || target.epoch!=basis.epoch || !target.observation_sequence || target.observation_sequence>current.observation_sequence)
                throw std::invalid_argument("timed targets belong to another runtime or timeline");
        }
        if(basis.control_time.time_since_epoch()>core::duration::max()-duration-state_->channel->controlPeriod().value() ||
           std::chrono::duration_cast<core::duration>(basis.control_received.time_since_epoch())>core::duration::max()-duration)
            throw std::invalid_argument("command expiry overflows clock range");
        const robot::communication::Expiry expiry=state_->mode==RunMode::Stepped
            ?robot::communication::Expiry{robot::communication::LogicalExpiry{basis.control_time+duration+state_->channel->controlPeriod().value(),basis.epoch}}
            :robot::communication::Expiry{robot::communication::HostExpiry{basis.control_received+duration}};
        auto command=robot::communication::Decoder{state_->channel->controllerConfig(),current,timed?&fit:nullptr}(
            {{state_->id,generation},{id},expiry,std::move(motion),gripper});
        return {id,state_->channel->submit(std::move(command)),timed?std::optional{std::move(fit)}:std::nullopt};
    }
    Operation request(robot::communication::OperatorAction action,std::optional<std::uint64_t> generation={}) const;
    Operation acquireControl(bool takeover=false) const;
    ControlSession acquired(const Operation &operation) const;
};
} // namespace application
