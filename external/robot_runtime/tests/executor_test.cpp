#include "robot_runtime/executor.hpp"
#include <iostream>
using namespace robot_runtime;
void check(bool b) { if(!b) throw std::runtime_error("check failed"); }
template<class F> void rejects(F f) { bool rejected=false; try{f();}catch(const std::exception&){rejected=true;} check(rejected); }
struct Fake:Backend {
    Measurement measured; Positions sent; bool enabled{},broken{};
    explicit Fake(int n) : measured{Positions(n,0),Positions(n,0)} {}
    Measurement read() override {return measured;}
    void enable(const Positions& q) override {enabled=true;write(q);}
    void write(const Positions& q) override {if(broken)throw std::runtime_error("device failure");sent=q;}
    void hold(const Positions& q) override {write(q);}
    void inhibit() override {enabled=false;}
};
int main() {
 for(int n:{3,7}) {
    RobotSpec spec; spec.backend="fake";
    for(int i=0;i<n;++i){spec.joints.push_back("j"+std::to_string(i));spec.units.push_back("rad");spec.lower.push_back(-1);spec.upper.push_back(1);}
    Fake b(n); Executor e(spec,b); e.reset(0); e.enable(0); check(e.status().mode==Mode::Ready);
    rejects([&]{e.enable(0);});
    auto s=e.activate("teleop"); rejects([&]{e.activate("policy");});
    JointReference r{Positions(n,.4),s,1,0,300'000'000}; e.submit(r,0);e.tick(0);
    check(b.sent==r.position && b.sent!=e.status().measured.position);
    auto bad=r;bad.sequence=2;bad.position[0]=2;bad.valid_until=600'000'000;
    rejects([&]{e.submit(bad,0);});rejects([&]{e.submit(r,0);});check(e.status().sequence==1);
    bad=r;bad.sequence=2;bad.issued_at=1;rejects([&]{e.submit(bad,0);});
    bad=r;bad.sequence=2;bad.position[0]=std::numeric_limits<double>::quiet_NaN();rejects([&]{e.submit(bad,0);});
    e.tick(299'000'000);check(e.status().mode==Mode::Running);
    e.tick(300'000'000);check(e.status().mode==Mode::Stopping && !e.status().session && b.sent==b.measured.position);
    e.tick(400'000'000);check(e.status().mode==Mode::Ready);
    rejects([&]{e.submit(r,400'000'000);});auto s2=e.activate("policy");check(s2!=s);
    e.reset(0);e.enable(0);auto s3=e.activate("teleop");check(s3!=s2);rejects([&]{e.submit(r,0);});
    e.stop(0);b.measured.velocity.assign(n,1);e.tick(5'000'000'000);check(e.status().mode==Mode::Fault && !b.enabled);
    rejects([&]{e.enable(5'000'000'000);});e.disable();check(e.status().mode==Mode::Fault);
    b.measured.velocity.assign(n,0);e.reset(0);e.enable(0);b.broken=true;e.tick(0);check(e.status().mode==Mode::Fault);
    b.broken=false;e.reset(0);e.enable(0);b.measured.available=false;e.tick(0);check(e.status().mode==Mode::Fault && !b.enabled);
    e.reset(0);check(e.status().mode==Mode::Fault);rejects([&]{e.enable(0);});
    b.measured.available=true;e.reset(0);e.enable(0);s=e.activate("stop-capture");
    e.submit({Positions(n,.5),s,1,0,300'000'000},0);e.tick(0);
    b.measured.position.assign(n,.2);b.measured.velocity.assign(n,1);e.stop(10'000'000);
    const auto captured=b.sent;b.measured.position.assign(n,.3);e.tick(20'000'000);check(b.sent==captured);
    rejects([&]{e.activate("replacement");});
    b.measured.velocity.assign(n,0);e.tick(30'000'000);e.tick(129'000'000);check(e.status().mode==Mode::Stopping);
    e.tick(130'000'000);check(e.status().mode==Mode::Ready);s=e.activate("replacement");
    check(e.status().delivered_sequence==0);
    for(int state=0;state<4;++state) {
        e.reset(0);
        if(state>=1)e.enable(0);
        if(state>=2){s=e.activate("disable");e.submit({Positions(n,.4),s,1,0,300'000'000},0);e.tick(0);}
        if(state>=3)e.stop(0);
        e.disable();check(e.status().mode==Mode::Disabled && !e.status().session && !b.enabled);
        rejects([&]{e.activate("disabled");});
    }
    b.measured.position.assign(n,1.001);e.reset(0);e.enable(0);
    check(b.sent==b.measured.position && e.status().mode==Mode::Ready);
    s=e.activate("outside-measurement");
    rejects([&]{e.submit({b.measured.position,s,1,0,300'000'000},0);});
 }
 std::cout<<"executor checks passed for 3 and 7 joints\n";
}
