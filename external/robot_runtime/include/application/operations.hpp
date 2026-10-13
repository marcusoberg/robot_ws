#pragma once
#include "application/source.hpp"
#include <thread>

namespace application {
enum class Outcome { Queued, Accepted, Completed, Rejected, Canceled };
struct OperationResult { Outcome outcome{Outcome::Queued}; std::string reason{"none"}; std::optional<robot::communication::Session> granted_session; };
inline const char *operatorReason(robot::communication::OperatorReason reason) {
    using enum robot::communication::OperatorReason;
    switch(reason) {case None:return "none";case InvalidState:return "invalid_state";case NoAuthority:return "no_authority";
    case NotHomed:return "not_homed";case Unavailable:return "unavailable";case Interrupted:return "interrupted";} std::unreachable();
}
inline const char *motionReason(robot::execution::RejectReason reason) {
    using enum robot::execution::RejectReason;
    switch(reason) {case None:return "none";case NoAuthority:return "no_authority";case Stale:return "stale";case WrongModel:return "wrong_model";
    case InvalidState:return "invalid_state";case QueueFull:return "queue_full";case Overrun:return "overrun";case Infeasible:return "infeasible";
    case Late:return "late";case Conflict:return "conflict";case Discontinuous:return "discontinuous";case BufferUnderrun:return "buffer_underrun";} std::unreachable();
}
class Operation {
    Source source_;
    std::uint64_t id_,epoch_,seen_;
    robot::communication::OperatorAction action_;
    OperationResult result_;
public:
    Operation(Source source,std::uint64_t id,robot::communication::OperatorAction action,const robot::execution::Snapshot &basis)
        :source_(std::move(source)),id_(id),epoch_(basis.epoch),seen_(basis.status.operator_sequence),action_(action) {}
    auto commandId() const {return id_;} auto sourceId() const {return source_.id();} auto action() const {return action_;}
    bool done() const {return result_.outcome==Outcome::Completed || result_.outcome==Outcome::Rejected || result_.outcome==Outcome::Canceled;}
    OperationResult poll(const robot::execution::Snapshot &basis) {
        if(done())return result_;
        if(basis.runtime_instance!=source_.runtimeInstance())throw std::invalid_argument("observation belongs to another runtime");
        if(basis.epoch!=epoch_)return result_={Outcome::Canceled,"timeline_reset"};
        const auto &status=basis.status;
        const auto first=status.operator_sequence>status.operator_results.size()?status.operator_sequence-status.operator_results.size()+1:1;
        if(first>seen_+1)throw std::runtime_error("operator event history was coalesced");
        for(auto i=std::max(first,seen_+1);i<=status.operator_sequence;++i) {
            const auto &event=status.operator_results[(i-1)%status.operator_results.size()];seen_=event.sequence;
            if(event.request.source!=source_.id() || event.request.request_id!=id_)continue;
            using enum robot::communication::OperatorDisposition;
            result_={event.disposition==Completed?Outcome::Completed:event.disposition==Accepted?Outcome::Accepted:event.disposition==Rejected?Outcome::Rejected:Outcome::Canceled,operatorReason(event.reason),event.granted_session};
            if(done())break;
        }
        return result_;
    }
    OperationResult outcome() {return poll(source_.snapshot());}
    OperationResult result(core::duration timeout=std::chrono::seconds(5)) {
        const auto deadline=core::HostClock::now()+timeout;
        while(!done()) {poll(source_.snapshot());if(done())break;if(core::HostClock::now()>=deadline)throw std::runtime_error("operation wait timed out");std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        return result_;
    }
};
class MotionOperation {
    Source source_;std::uint64_t generation_,epoch_,id_;bool finite_;OperationResult result_;
    bool matches(const std::optional<robot::communication::Reference> &v) const {return v && v->session==robot::communication::Session{source_.id(),generation_} && v->id.value==id_;}
public:
    std::optional<robot::execution::TargetFitReport> fit;
    MotionOperation(Source source,std::uint64_t generation,std::uint64_t epoch,Submission submission,bool finite)
        :source_(std::move(source)),generation_(generation),epoch_(epoch),id_(submission.command_id),finite_(finite),fit(std::move(submission.fit)) {
        if(!submission.queued)result_={Outcome::Rejected,"queue_full"};
    }
    auto commandId() const {return id_;} auto sourceId() const {return source_.id();}
    bool done() const {return result_.outcome==Outcome::Completed || result_.outcome==Outcome::Rejected || result_.outcome==Outcome::Canceled || (!finite_ && result_.outcome==Outcome::Accepted);}
    OperationResult poll(const robot::execution::Snapshot &basis) {
        if(done())return result_;
        if(basis.runtime_instance!=source_.runtimeInstance())throw std::invalid_argument("observation belongs to another runtime");
        if(basis.epoch!=epoch_)return result_={Outcome::Canceled,"timeline_reset"};
        const auto &s=basis.status;
        if(matches(s.rejected))return result_={Outcome::Rejected,motionReason(s.rejection)};
        if(matches(s.completed))return result_={Outcome::Completed,"none"};
        if(matches(s.canceled))return result_={Outcome::Canceled,"none"};
        if(s.owner!=robot::communication::Session{source_.id(),generation_})return result_={Outcome::Canceled,"authority_lost"};
        if(matches(s.accepted) || matches(s.active))result_={Outcome::Accepted,"none"};
        return result_;
    }
    OperationResult outcome() {return poll(source_.snapshot());}
};
class ControlSession {
    Source source_;std::uint64_t generation_,epoch_;std::shared_ptr<std::atomic_bool> closed_=std::make_shared<std::atomic_bool>(false);
public:
    ControlSession(Source source,std::uint64_t generation,std::uint64_t epoch):source_(std::move(source)),generation_(generation),epoch_(epoch){}
    auto generation() const{return generation_;}auto epoch() const{return epoch_;}auto sourceId() const{return source_.id();}
    bool valid() const {const auto current=source_.snapshot();return !closed_->load() && current.epoch==epoch_ && current.status.owner==robot::communication::Session{source_.id(),generation_};}
    bool renew(const robot::execution::Snapshot &basis,std::optional<double> valid_for={}) const {return !closed_->load() && source_.renew(generation_,basis,valid_for);}
    MotionOperation submit(robot::communication::MotionRequest motion,const robot::execution::Snapshot &basis,double valid_for=0.3,std::optional<bool> gripper={}) const {
        if(!valid())throw std::logic_error("session no longer owns control");
        const bool finite=std::holds_alternative<robot::communication::TrajectoryRequest>(motion);
        return {source_,generation_,epoch_,source_.submit(std::move(motion),generation_,valid_for,basis,gripper),finite};
    }
    Operation pause() const {if(!valid())throw std::logic_error("session no longer owns control");return source_.request(robot::communication::OperatorAction::Pause,generation_);}
    std::optional<Operation> release() {const bool owned=valid();if(closed_->exchange(true) || !owned)return {};return source_.request(robot::communication::OperatorAction::Release,generation_);}
};
inline Operation Source::request(robot::communication::OperatorAction action,std::optional<std::uint64_t> generation) const {
    const auto basis=snapshot();const auto [id,queued]=operate(action,generation);if(!queued)throw std::runtime_error("operator ingress is full");return {*this,id,action,basis};
}
inline Operation Source::acquireControl(bool takeover) const {return request(takeover?robot::communication::OperatorAction::Takeover:robot::communication::OperatorAction::Acquire);}
inline ControlSession Source::acquired(const Operation &operation) const {
    if(operation.sourceId()!=id() || (operation.action()!=robot::communication::OperatorAction::Acquire && operation.action()!=robot::communication::OperatorAction::Takeover))throw std::invalid_argument("operation is not this source's acquisition");
    auto copy=operation; const auto result=copy.outcome();if(result.outcome!=Outcome::Completed || !result.granted_session)throw std::logic_error("acquisition has not completed");
    const auto basis=snapshot();if(basis.status.owner!=result.granted_session)throw std::logic_error("acquired authority was revoked");
    return {*this,basis.status.owner->generation,basis.epoch};
}
} // namespace application
