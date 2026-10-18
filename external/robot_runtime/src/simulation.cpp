#include <map>
#include "robot_runtime/simulation.hpp"
#include <cstring>

namespace robot_runtime {
MujocoBackend::MujocoBackend(const RobotSpec& spec,const std::string& scene,const Positions& initial) : spec_(spec), initial_(initial) {
    spec_.validate(); spec_.validate_positions(initial);
    char error[1024]{};
    model.reset(mj_loadXML(scene.c_str(),nullptr,error,sizeof(error)));
    if (!model) throw std::invalid_argument(std::string("MuJoCo model: ")+error);
    data.reset(mj_makeData(model.get()));
    if (!data) throw std::runtime_error("cannot allocate MuJoCo state");
    auto* m=model.get();
    if (!(m->opt.disableflags & mjDSBL_CLAMPCTRL)) throw std::invalid_argument("position backend requires explicit reference validation with MuJoCo control clipping disabled");
    if (m->nu != static_cast<int>(spec.joints.size())) throw std::invalid_argument("actuator count differs from robot specification");
    for (std::size_t i=0;i<spec.joints.size();++i) {
        int j=mj_name2id(m,mjOBJ_JOINT,spec.joints[i].c_str());
        if (j<0 || (m->jnt_type[j]!=mjJNT_HINGE && m->jnt_type[j]!=mjJNT_SLIDE)) throw std::invalid_argument("missing scalar joint: "+spec.joints[i]);
        if (spec.units[i]!=(m->jnt_type[j]==mjJNT_HINGE?"rad":"m")) throw std::invalid_argument("joint units disagree with simulator");
        if (!m->jnt_limited[j] || std::abs(m->jnt_range[2*j]-spec.lower[i])>1e-9 || std::abs(m->jnt_range[2*j+1]-spec.upper[i])>1e-9)
            throw std::invalid_argument("joint limits disagree with description: "+spec.joints[i]);
        int a=-1;
        for (int k=0;k<m->nu;++k) if (m->actuator_trntype[k]==mjTRN_JOINT && m->actuator_trnid[2*k]==j) {
            if (a>=0) throw std::invalid_argument("multiple actuators for joint"); a=k;
        }
        if (a<0 || m->actuator_dyntype[a]!=mjDYN_NONE || m->actuator_gaintype[a]!=mjGAIN_FIXED || m->actuator_biastype[a]!=mjBIAS_AFFINE ||
            m->actuator_biasprm[a*mjNBIAS]!=0 || m->actuator_biasprm[a*mjNBIAS+2]>0 ||
            m->actuator_gainprm[a*mjNGAIN]<=0 || std::abs(m->actuator_gainprm[a*mjNGAIN]+m->actuator_biasprm[a*mjNBIAS+1])>1e-9 ||
            m->actuator_gear[6*a]!=1 || !m->actuator_ctrllimited[a] ||
            std::abs(m->actuator_ctrlrange[2*a]-spec.lower[i])>1e-9 || std::abs(m->actuator_ctrlrange[2*a+1]-spec.upper[i])>1e-9)
            throw std::invalid_argument("position actuator disagrees with description: "+spec.joints[i]);
        for (int k=1;k<6;++k) if (m->actuator_gear[6*a+k]!=0) throw std::invalid_argument("non-scalar actuator gear");
        joints_.push_back(j); actuators_.push_back(a);
        std::array<double,mjNGAIN> gain{}; std::array<double,mjNBIAS> bias{};
        std::copy_n(m->actuator_gainprm+a*mjNGAIN,mjNGAIN,gain.begin());
        std::copy_n(m->actuator_biasprm+a*mjNBIAS,mjNBIAS,bias.begin());
        gains_.push_back(gain); biases_.push_back(bias);
    }
    reset();
}
Measurement MujocoBackend::read() {
    Measurement out;
    for (int warning : {mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC, mjWARN_BADCTRL})
        if (data->warning[warning].number) out.fault=true;
    for (int j:joints_) { out.position.push_back(data->qpos[model->jnt_qposadr[j]]); out.velocity.push_back(data->qvel[model->jnt_dofadr[j]]); }
    return out;
}
void MujocoBackend::write(const Positions& q) {
    if (q.size()!=joints_.size()) throw std::invalid_argument("backend reference size");
    for (std::size_t i=0;i<q.size();++i) data->ctrl[actuators_[i]]=q[i];
}
void MujocoBackend::enable(const Positions& q) {
    for (std::size_t i=0;i<actuators_.size();++i) {
        std::copy(gains_[i].begin(),gains_[i].end(),model->actuator_gainprm+actuators_[i]*mjNGAIN);
        std::copy(biases_[i].begin(),biases_[i].end(),model->actuator_biasprm+actuators_[i]*mjNBIAS);
    }
    write(q);
}
void MujocoBackend::hold(const Positions& q) { write(q); }
void MujocoBackend::inhibit() {
    for (int a:actuators_) {
        std::fill_n(model->actuator_gainprm+a*mjNGAIN,mjNGAIN,0.0);
        std::fill_n(model->actuator_biasprm+a*mjNBIAS,mjNBIAS,0.0);
    }
}
void MujocoBackend::reset() {
    mj_resetData(model.get(),data.get());
    for (std::size_t i=0;i<joints_.size();++i) data->qpos[model->jnt_qposadr[joints_[i]]]=initial_[i];
    write(initial_); inhibit(); mj_forward(model.get(),data.get());
}
void MujocoBackend::advance(std::size_t count) {
    for (std::size_t i=0;i<count;++i) mj_step(model.get(),data.get());
    mj_forward(model.get(),data.get());
}
Positions MujocoBackend::integration_state() const {
    Positions state(mj_stateSize(model.get(),mjSTATE_INTEGRATION));
    mj_getState(model.get(),data.get(),state.data(),mjSTATE_INTEGRATION); return state;
}
std::map<std::string,Positions> MujocoBackend::body_state() const {
    std::map<std::string,Positions> out;
    for(int b=0;b<model->nbody;++b) {
        const char* name=mj_id2name(model.get(),mjOBJ_BODY,b);
        if(!name) continue;
        Positions pose(data->xpos+3*b,data->xpos+3*b+3);
        pose.insert(pose.end(),data->xmat+9*b,data->xmat+9*b+9);
        out[name]=std::move(pose);
    }
    return out;
}
std::vector<Contact> MujocoBackend::contacts() const {
    std::vector<Contact> out;
    for(int i=0;i<data->ncon;++i) {
        const auto& c=data->contact[i];
        if(c.geom[0]<0 || c.geom[1]<0) continue;
        mjtNum force[6]{}; mj_contactForce(model.get(),data.get(),i,force);
        if(force[0]<=0) continue;
        Contact item; item.normal_force=force[0];
        for(int side=0;side<2;++side) {
            const char* geom=mj_id2name(model.get(),mjOBJ_GEOM,c.geom[side]);
            (side?item.geom2:item.geom1)=geom?geom:"";
            auto& ancestors=side?item.ancestors2:item.ancestors1;
            for(int body=model->geom_bodyid[c.geom[side]];body>0;body=model->body_parentid[body]) {
                const char* name=mj_id2name(model.get(),mjOBJ_BODY,body);
                if(name) ancestors.emplace_back(name);
            }
        }
        out.push_back(std::move(item));
    }
    return out;
}
Simulation::Simulation(RobotSpec spec,const std::string& scene,Positions initial,Time control,Time action,StopProfile stop,Time expected_physics)
    : backend_(spec,scene,initial),executor_(spec,backend_,stop),control_period_(control),action_period_(action) {
    const auto physics=std::llround(backend_.model->opt.timestep*1e9);
    if(physics!=expected_physics || physics<=0 || control<=0 || action<=0 || control%physics || action%control)
        throw std::invalid_argument("physics period must match the profile; control/action periods must be positive integral multiples");
    substeps_=control/physics; reset();
}
Simulation::~Simulation() { stop_paced(); executor_.disable(); }
Time Simulation::now() const {
    return paced_?origin_time_+std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-origin_).count():time_;
}
Snapshot Simulation::reset() {
    std::scoped_lock lock(mutex_);
    if(paced_) throw std::logic_error("stop paced driver before resetting simulation");
    backend_.reset(); time_=0; ++episode_; trace_.clear(); lag_=0; pending_.reset(); interval_end_.reset(); needs_reset_=false;
    executor_.reset(0); boundary_=snapshot(); return boundary_;
}
void Simulation::enable() { std::scoped_lock lock(mutex_); executor_.enable(now()); }
std::uint64_t Simulation::activate(const std::string& source) { std::scoped_lock lock(mutex_); return executor_.activate(source); }
void Simulation::submit(const Positions& goal,std::uint64_t session,std::uint64_t sequence,Time ttl) {
    std::scoped_lock lock(mutex_);
    if(ttl<=0 || ttl>300'000'000) throw std::invalid_argument("goal validity must be in (0,300ms]");
    auto t=now(); executor_.submit(controller_.reference(goal,session,sequence,t,t+ttl),t);
}
void Simulation::tick() {
    if(time_%action_period_==0) boundary_=snapshot();
    if(pending_ && time_>=pending_time_) {
        auto reference=std::move(*pending_); pending_.reset();
        try {
            executor_.submit(reference,now());
            interval_end_=pending_interval_only_?std::optional<Time>{pending_time_+action_period_}:std::nullopt;
        }
        catch(const std::exception& e) { executor_.stop(now(),std::string("scheduled_action_rejected: ")+e.what()); }
    }
    if(interval_end_ && time_>=*interval_end_) {
        interval_end_.reset(); executor_.stop(now(),"action_interval_exhausted");
    }
    executor_.tick(now());
    const auto delivered=executor_.status();
    backend_.advance(substeps_);
    const auto begin=time_; time_+=control_period_;
    executor_.observe(now());
    trace_.push_back({begin,time_,delivered.reference,executor_.status().measured.position,delivered.delivered_sequence});
    if(trace_.size()>512) trace_.erase(trace_.begin(),trace_.begin()+256);
}
Snapshot Simulation::step(const Positions& goal,std::uint64_t session,std::uint64_t sequence) {
    std::scoped_lock lock(mutex_);
    if(paced_ || needs_reset_) throw std::logic_error("atomic step requires stepped mode; reset after leaving paced mode");
    executor_.submit(controller_.reference(goal,session,sequence,time_,time_+300'000'000),time_);
    trace_.clear();
    for(Time t=0;t<action_period_;t+=control_period_) tick();
    return snapshot();
}
Snapshot Simulation::advance(std::size_t ticks) {
    std::scoped_lock lock(mutex_);
    if(paced_ || needs_reset_ || ticks>1000) throw std::logic_error("advance requires stepped mode and at most 1000 ticks; reset after leaving paced mode");
    trace_.clear(); for(std::size_t i=0;i<ticks;++i) tick(); return snapshot();
}
Snapshot Simulation::snapshot() {
    const auto host=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return {executor_.status(),time_,episode_,++observation_,backend_.integration_state(),trace_,backend_.body_state(),backend_.contacts(),lag_,host};
}
Snapshot Simulation::observe() { std::scoped_lock lock(mutex_); return snapshot(); }
Snapshot Simulation::latest_boundary() { std::scoped_lock lock(mutex_); return boundary_; }
void Simulation::schedule(const Positions& goal,std::uint64_t session,std::uint64_t sequence,Time application_time,bool interval_only) {
    std::scoped_lock lock(mutex_);
    const auto& s=executor_.status();
    if(needs_reset_ || pending_ || application_time<time_ || application_time%action_period_ || application_time>time_+action_period_ ||
       !session || s.session!=session || (s.mode!=Mode::Ready && s.mode!=Mode::Running) || sequence<=s.sequence)
        throw std::invalid_argument("scheduled action is late, reordered, unowned, or queue is full");
    executor_.spec().validate_positions(goal);
    auto t=now(); pending_=controller_.reference(goal,session,sequence,t,t+300'000'000); pending_time_=application_time;
    pending_interval_only_=interval_only;
}
void Simulation::pause(const std::string& reason) { std::scoped_lock lock(mutex_); pending_.reset(); interval_end_.reset(); executor_.stop(now(),reason); }
void Simulation::disable() { std::scoped_lock lock(mutex_); pending_.reset(); interval_end_.reset(); executor_.disable(); }
bool Simulation::paced() { std::scoped_lock lock(mutex_); return paced_; }
void Simulation::start_paced() {
    std::scoped_lock lock(mutex_);
    if(paced_ || needs_reset_) throw std::logic_error("paced driver already running or needs reset");
    paced_=true; origin_=std::chrono::steady_clock::now(); origin_time_=executor_.status().time;
    runner_=std::jthread([this](std::stop_token token) {
        auto next=std::chrono::steady_clock::now();
        while(!token.stop_requested()) {
            {
                std::scoped_lock guard(mutex_);
                lag_=std::max(0.0,std::chrono::duration<double>(std::chrono::steady_clock::now()-next).count());
                try { tick(); } catch(const std::exception& e) { executor_.fault(e.what()); }
            }
            next+=std::chrono::nanoseconds(control_period_);
            if(next<std::chrono::steady_clock::now()) next=std::chrono::steady_clock::now();
            std::this_thread::sleep_until(next);
        }
    });
}
void Simulation::stop_paced() {
    if(runner_.joinable()) { runner_.request_stop(); runner_.join(); }
    std::scoped_lock lock(mutex_);
    if(paced_) { pending_.reset(); interval_end_.reset(); executor_.disable(); paced_=false; needs_reset_=true; }
}
}
