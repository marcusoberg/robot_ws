#pragma once
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include "robot/communication/requests.hpp"
#include "robot/execution/controller_config.hpp"

namespace robot::execution {
struct TargetFitReport {
    std::vector<double> deviation, budget;
    std::size_t saturation{};
};
struct TargetFitLimits {
    Eigen::VectorXd lower,upper,velocity,acceleration,jerk;
};
struct FittedTargets {
    communication::ScheduleRequest schedule;
    TargetFitReport report;
    Eigen::MatrixXd position,velocity,acceleration;
};
// Preparation only: a penalized natural cubic spline followed by exact-duration
// Ruckig segments. This fitting routine never runs in graph evaluation/servo ticks.
// Tolerance is measured at original target knots, against clipped input positions.
inline FittedTargets fitTimedTargets(const communication::TimedJointTargetsRequest &r,
                                    const TargetFitLimits &limits, const Eigen::MatrixXd &input, bool prepare_schedule=true) {
    const auto n=input.rows(), dof=input.cols();
    if (n<5 || n>communication::TimedJointTargetsRequest::maximum_samples || dof!=limits.lower.size() || !input.allFinite())
        throw std::invalid_argument("timed targets require 5..256 finite model-sized rows");
    if (r.period<=core::duration{} || r.join<=core::duration{} || r.origin.time_since_epoch()<core::duration{} || r.start.time_since_epoch()<core::duration{})
        throw std::invalid_argument("invalid timed-target period, origin, start or join");
    if (r.period.count() > (core::duration::max().count()-r.origin.time_since_epoch().count())/(n-1) ||
        r.start.time_since_epoch()>core::duration::max()-r.join)
        throw std::invalid_argument("timed target timestamps overflow");
    if (!r.gripper_open.empty() && r.gripper_open.size() != static_cast<std::size_t>(n))
        throw std::invalid_argument("gripper sample count must equal timed target count");
    Eigen::VectorXd tolerance(dof);
    if (r.tolerance.size()!=1 && r.tolerance.size()!=std::size_t(dof)) throw std::invalid_argument("fit tolerance must be scalar or model-sized");
    for (Eigen::Index j=0;j<dof;++j) {
        tolerance[j]=r.tolerance[r.tolerance.size()==1?0:j];
        if (!std::isfinite(tolerance[j]) || tolerance[j]<0) throw std::invalid_argument("fit tolerance must be finite and nonnegative");
    }
    if(dof==0 || limits.upper.size()!=dof || limits.velocity.size()!=dof || limits.acceleration.size()!=dof || limits.jerk.size()!=dof ||
       !limits.lower.allFinite() || !limits.upper.allFinite() || !limits.velocity.allFinite() || !limits.acceleration.allFinite() || !limits.jerk.allFinite() ||
       (limits.lower.array()>=limits.upper.array()).any() || (limits.velocity.array()<=0).any() || (limits.acceleration.array()<=0).any() || (limits.jerk.array()<=0).any())
        throw std::invalid_argument("invalid target-fitting bounds or execution limits");
    Eigen::MatrixXd positions=input;
    TargetFitReport report; report.budget.assign(tolerance.data(),tolerance.data()+dof);
    for (Eigen::Index i=0;i<n;++i) for(Eigen::Index j=0;j<dof;++j) {
        if(input(i,j)<limits.lower[j] || input(i,j)>limits.upper[j]) ++report.saturation;
        positions(i,j)=std::clamp(input(i,j),limits.lower[j],limits.upper[j]);
    }
    const double h=std::chrono::duration<double>(r.period).count();
    // Reinsch natural-cubic smoothing system. For unit observation weights:
    // (R + lambda Q'Q)m = Q'y, f = y - lambda Qm. The retained m are
    // the fitted spline's second derivatives at its interior knots.
    Eigen::MatrixXd q=Eigen::MatrixXd::Zero(n,n-2), curvature=Eigen::MatrixXd::Zero(n-2,n-2);
    for(Eigen::Index i=0;i<n-2;++i) {
        q(i,i)=1/h; q(i+1,i)=-2/h; q(i+2,i)=1/h;
        curvature(i,i)=2*h/3;
        if(i+1<n-2) curvature(i,i+1)=curvature(i+1,i)=h/6;
    }
    const Eigen::MatrixXd penalty=q.transpose()*q, rhs=q.transpose()*positions;
    Eigen::MatrixXd accepted_position, accepted_velocity, accepted_acceleration;
    bool accepted=false;
    for(int attempt=0;attempt<32;++attempt) {
        const double lambda=attempt==0?0:std::pow(h*0.05*std::pow(1.3,attempt-1),4)/h;
        Eigen::LDLT<Eigen::MatrixXd> solver(curvature+lambda*penalty);
        if(solver.info()!=Eigen::Success) throw std::invalid_argument("timed target fit factorization failed");
        const Eigen::MatrixXd inner=solver.solve(rhs);
        const Eigen::MatrixXd fitted=positions-lambda*q*inner;
        Eigen::MatrixXd second=Eigen::MatrixXd::Zero(n,dof); second.middleRows(1,n-2)=inner;
        if(!fitted.allFinite() || !second.allFinite()) throw std::invalid_argument("timed target fit overflow");
        const auto sample=[&](Eigen::Index interval,double t,int derivative) -> Eigen::RowVectorXd {
            const auto a=fitted.row(interval).eval();
            const auto b=((fitted.row(interval+1)-a)/h-h*(2*second.row(interval)+second.row(interval+1))/6).eval();
            const auto c=(second.row(interval)/2).eval();
            const auto d=((second.row(interval+1)-second.row(interval))/(6*h)).eval();
            switch(derivative) {
            case 0:return a+t*(b+t*(c+t*d));
            case 1:return b+t*(2*c+t*3*d);
            case 2:return 2*c+t*6*d;
            default:return 6*d;
            }
        };
        Eigen::MatrixXd probes(10*(n-1)+1,dof);
        for(Eigen::Index k=0;k<probes.rows();++k) probes.row(k)=sample(std::min(k/10,n-2),k==10*(n-1)?h:(k%10)*h/10,0);
        const Eigen::VectorXd minimum=probes.colwise().minCoeff().transpose(),maximum=probes.colwise().maxCoeff().transpose();
        Eigen::VectorXd scale(dof),shift(dof);
        for(Eigen::Index j=0;j<dof;++j) {
            const double margin=tolerance[j]*0.001, window=limits.upper[j]-limits.lower[j]-2*margin;
            if(window<0) throw std::invalid_argument("fit tolerance margin exceeds joint range");
            scale[j]=std::min(1.0,window/std::max(maximum[j]-minimum[j],1e-12));
            const double lo=limits.lower[j]+margin-scale[j]*minimum[j], hi=limits.upper[j]-margin-scale[j]*maximum[j];
            shift[j]=std::clamp(0.0,lo,std::max(lo,hi));
        }
        accepted_position=fitted*scale.asDiagonal(); accepted_position.rowwise()+=shift.transpose();
        const Eigen::VectorXd deviation=(accepted_position-positions).cwiseAbs().colwise().maxCoeff().transpose();
        report.deviation.assign(deviation.data(),deviation.data()+dof);
        bool fits=(deviation.array()<=tolerance.array()).all();
        for(Eigen::Index k=0;fits && k<probes.rows();++k) {
            const auto interval=std::min(k/10,n-2); const double t=k==10*(n-1)?h:(k%10)*h/10;
            for(Eigen::Index j=0;j<dof;++j) {
                const double value=probes(k,j)*scale[j]+shift[j];
                if(value<limits.lower[j] || value>limits.upper[j]) { fits=false; break; }
            }
            for(int order=1;fits && order<=3;++order) {
                const auto derivative=(sample(interval,t,order)*scale.asDiagonal()).eval();
                const auto &limit=order==1?limits.velocity:order==2?limits.acceleration:limits.jerk;
                fits=(derivative.cwiseAbs().array()<=limit.transpose().array()).all();
            }
        }
        if(!fits) continue;
        accepted_velocity.resize(n,dof); accepted_acceleration=second*scale.asDiagonal();
        for(Eigen::Index i=0;i<n;++i) accepted_velocity.row(i)=sample(std::min(i,n-2),i==n-1?h:0,1)*scale.asDiagonal();
        accepted=true; break;
    }
    if(!accepted) throw std::invalid_argument("prediction cannot fit execution limits within declared target tolerance");
    if(!prepare_schedule) return {{},std::move(report),std::move(accepted_position),std::move(accepted_velocity),std::move(accepted_acceleration)};
    const auto threshold=r.start+r.join;
    const auto delta=threshold>r.origin?threshold-r.origin:core::duration{};
    const auto first=delta.count()/r.period.count()+(delta.count()%r.period.count()!=0);
    if(first>=n-1) throw std::invalid_argument("prediction has insufficient future motion after its join");
    communication::ScheduleRequest schedule{communication::ModelJointOrder{},r.start,r.epoch,r.replaces,{}};
    schedule.points.reserve(n-first);
    // Tool states keep their original timestamps, including during the arm's
    // join segment. Otherwise frequent replacements could postpone them forever.
    if (!r.gripper_open.empty()) {
        schedule.gripper.reserve(n);
        for (Eigen::Index i=0;i<n;++i) {
            const auto time=r.origin+i*r.period;
            if (time>=r.start) schedule.gripper.push_back({time-r.start,r.gripper_open[static_cast<std::size_t>(i)]});
        }
    }
    for(Eigen::Index i=first;i<n;++i) {
        communication::ReferencePointRequest point;
        for(Eigen::Index j=0;j<dof;++j) {
            point.position.push_back(accepted_position(i,j)); point.velocity.push_back(accepted_velocity(i,j)); point.acceleration.push_back(accepted_acceleration(i,j));
        }
        point.offset=r.origin+i*r.period-r.start;
        schedule.points.push_back(std::move(point));
    }
    return {std::move(schedule),std::move(report),std::move(accepted_position),std::move(accepted_velocity),std::move(accepted_acceleration)};
}
inline FittedTargets fitTimedTargets(const communication::TimedJointTargetsRequest &r,const ControllerConfig &config,const Eigen::MatrixXd &input) {
    const auto &model=config.model().description();
    return fitTimedTargets(r,TargetFitLimits{model.lower,model.upper,config.velocity(),config.acceleration(),config.jerk()},input);
}
} // namespace robot::execution
