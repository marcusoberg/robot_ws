#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/variant.h>
#include "robot_runtime/robot.hpp"
#include <cstring>

namespace nb = nanobind;
namespace comm = robot::communication;
namespace {
const char *reason(robot::execution::RejectReason r) {
    using enum robot::execution::RejectReason;
    switch(r) {
    case None:return "none";case NoAuthority:return "no_authority";case Stale:return "stale";
    case WrongModel:return "wrong_model";case InvalidState:return "invalid_state";case QueueFull:return "queue_full";
    case Overrun:return "overrun";case Infeasible:return "infeasible";case Late:return "late";
    case Conflict:return "conflict";case Discontinuous:return "discontinuous";case BufferUnderrun:return "buffer_underrun";
    } std::unreachable();
}
std::vector<double> vector(const Eigen::VectorXd &v) { return {v.data(),v.data()+v.size()}; }
struct ImagePlane {
    std::string name,dtype,frame,calibration,source_clock;
    std::vector<std::size_t> shape;
    std::size_t offset{},bytes{};
    std::uint64_t sequence{},epoch{};
    std::optional<std::int64_t> source_time_ns,host_received_ns;
    std::array<double,9> intrinsics{};
    std::array<double,16> world_from_camera{};
};
struct Snapshot {
    std::shared_ptr<const robot::execution::Snapshot> value;
    explicit Snapshot(robot::execution::Snapshot v) : value(std::make_shared<const robot::execution::Snapshot>(std::move(v))) {}
};
nb::object array(const Snapshot &owner, const double *data, std::size_t count) {
    nb::capsule lifetime(new std::shared_ptr<const robot::execution::Snapshot>(owner.value), [](void *p) noexcept {
        delete static_cast<std::shared_ptr<const robot::execution::Snapshot> *>(p);
    });
    return nb::cast(nb::ndarray<nb::numpy, const double>(data, {count}, lifetime));
}
nb::object coordinates(const Snapshot &owner, const Eigen::VectorXd &v) { return array(owner, v.data(), v.size()); }
nb::dict reference(const comm::Reference &v) {
    nb::dict out; out["source"] = v.session.source.value; out["generation"] = v.session.generation; out["id"] = v.id.value; return out;
}
nb::object reference(const std::optional<comm::Reference> &v) { return v ? nb::object(reference(*v)) : nb::object(nb::none()); }
nb::dict pose(const Eigen::Isometry3d &v) {
    const Eigen::Quaterniond q{v.rotation()}; nb::dict out;
    out["position"] = nb::cast(std::array{v.translation().x(), v.translation().y(), v.translation().z()});
    out["quaternion_wxyz"] = nb::cast(std::array{q.w(), q.x(), q.y(), q.z()}); return out;
}
void stamp(nb::dict &out, const robot::backend::SampleStamp &s) {
    out["sequence"] = s.sequence; out["epoch"] = s.epoch; out["source_clock"] = s.source_clock;
    out["source_time_ns"] = nb::cast(s.source_time_ns);
    out["host_received_ns"] = std::chrono::duration_cast<core::duration>(s.received.time_since_epoch()).count();
}
nb::dict snapshotData(const Snapshot &box) {
    const auto &v = *box.value; const auto &s = v.status; const auto &r = v.actuator_request; nb::dict out;
    out["runtime_instance"] = v.runtime_instance;
    out["epoch"] = v.epoch; out["time_ns"] = v.control_time.time_since_epoch().count();
    out["observation_sequence"] = v.observation_sequence;
    out["begin_ns"] = (v.control_time - v.control_dt).time_since_epoch().count();
    out["mode"] = robot::execution::toString(s.mode); out["generation"] = s.generation;
    out["owner"] = s.owner ? nb::cast(s.owner->source.value) : nb::object(nb::none());
    out["motion_limited"] = s.motion_limited; out["rejection"] = reason(s.rejection); out["interruption"] = reason(s.interruption);
    out["active"] = reference(s.active); out["accepted"] = reference(s.accepted); out["rejected"] = reference(s.rejected);
    out["completed"] = reference(s.completed); out["canceled"] = reference(s.canceled);
    out["state"] = nb::none(); out["tool_pose"] = nb::none();
    if (v.backend.state) {
        const auto &measured = *v.backend.state; nb::dict state;
        state["position"] = coordinates(box, measured.joints.position()); state["velocity"] = coordinates(box, measured.joints.velocity());
        state["effort"] = measured.joints.effort() ? coordinates(box, measured.joints.effort()->values()) : nb::object(nb::none());
        state["homed"] = measured.homed; state["fault"] = measured.fault; state["physical_estop"] = measured.physical_estop;
        state["enabled"] = nb::cast(measured.enabled); state["gripper_open"] = nb::cast(measured.gripper_open); stamp(state, measured.stamp);
        out["state"] = state;
    }
    if (v.tool_pose) out["tool_pose"] = pose(v.tool_pose->base_from_tool);
    nb::dict sensors;
    for (const auto &sensor : v.backend.sensors) {
        nb::dict sample; sample["frame"] = sensor.frame; sample["calibration"] = sensor.calibration; stamp(sample, sensor.stamp);
        if (const auto *numeric = std::get_if<robot::backend::NumericSensor>(&sensor.payload)) {
            sample["values"] = array(box, numeric->values.data(), numeric->values.size()); sample["units"] = nb::cast(numeric->units);
            sensors[nb::str(sensor.name.c_str())] = sample;
        }
    }
    out["sensors"] = sensors;
    out["tracking_error"] = nb::none();
    if (v.backend.state) {
        const auto error = (r.position.values() - v.backend.state->joints.position()).eval();
        out["tracking_error"] = nb::cast(vector(error));
    }
    nb::dict actuator; actuator["position"] = coordinates(box, r.position.values()); actuator["velocity"] = coordinates(box, r.velocity.values());
    actuator["acceleration"] = coordinates(box, r.acceleration.values()); actuator["sequence"] = r.sequence;
    actuator["actuation"] = unsigned(r.actuation); actuator["gripper_open"] = nb::cast(r.gripper_open); out["actuator_request"] = actuator;
    nb::list events;
    const auto first = s.operator_sequence > s.operator_results.size() ? s.operator_sequence - s.operator_results.size() + 1 : 1;
    for (auto n = first; n <= s.operator_sequence; ++n) {
        const auto &e = s.operator_results[(n - 1) % s.operator_results.size()]; nb::dict event;
        event["sequence"] = e.sequence; event["source"] = e.request.source.value; event["id"] = e.request.request_id;
        event["action"] = unsigned(e.request.action); event["disposition"] = unsigned(e.disposition); event["reason"] = unsigned(e.reason); event["session_generation"] = e.granted_session?nb::cast(e.granted_session->generation):nb::object(nb::none()); events.append(event);
    }
    out["operator_results"] = events; out["operator_sequence"] = s.operator_sequence;
    nb::dict timing; timing["control_dt_ns"] = v.control_dt.count(); timing["host_interval_ns"] = v.host_interval.count();
    timing["cycle_duration_ns"] = v.cycle_duration.count(); timing["coalesced_observations"] = 0; out["timing"] = timing;
    out["collision"] = nb::cast(v.backend.collision);
    out["schedule"] = nb::none();
    if (s.schedule) {
        const auto scheduled = [](const auto &p) {
            nb::dict d; d["command"] = reference(p.reference); d["start_ns"] = p.motion.start().time_since_epoch().count();
            d["end_ns"] = p.motion.end().time_since_epoch().count(); return d;
        };
        nb::dict schedule; schedule["current"] = scheduled(s.schedule->current);
        schedule["pending"] = s.schedule->pending ? nb::object(scheduled(*s.schedule->pending)) : nb::object(nb::none()); out["schedule"] = schedule;
    }
    nb::dict delivery;
    std::visit([&](const auto &e) {
        using T = std::remove_cvref_t<decltype(e)>;
        if constexpr (std::same_as<T, robot::backend::SimulatorDelivery>) { delivery["evidence"] = "simulator_accepted"; delivery["request_sequence"] = e.request_sequence; }
        else if constexpr (std::same_as<T, robot::backend::TransportDelivery>) {
            delivery["evidence"] = "transport"; delivery["fault"] = e.fault; delivery["pending"] = nb::cast(e.pending);
            delivery["transmitted"] = nb::none();
            if (e.transmitted) {
                nb::dict transmitted; transmitted["request_sequence"] = e.transmitted->request_sequence;
                transmitted["host_time_ns"] = std::chrono::duration_cast<core::duration>(e.transmitted->at.time_since_epoch()).count();
                transmitted["device_command"] = e.transmitted->device_command; delivery["transmitted"] = transmitted;
            }
        }
        else delivery["evidence"] = "unreported";
    }, v.delivery);
    out["delivery"] = delivery;
    nb::list transitions;
    const auto transition_first = s.transition_sequence > s.transitions.size() ? s.transition_sequence - s.transitions.size() + 1 : 1;
    for (auto n = transition_first; n <= s.transition_sequence; ++n) {
        const auto &t = s.transitions[(n - 1) % s.transitions.size()]; nb::dict event;
        event["sequence"] = t.sequence; event["from"] = robot::execution::toString(t.from); event["to"] = robot::execution::toString(t.to);
        event["generation"] = t.generation; event["time_ns"] = t.time.time_since_epoch().count(); event["epoch"] = t.epoch;
        event["reason"] = reason(t.reason); event["owner"] = t.owner ? nb::cast(t.owner->source.value) : nb::object(nb::none()); transitions.append(event);
    }
    out["transitions"] = transitions; out["transition_sequence"] = s.transition_sequence;
    out["requested_motion"] = nb::none();
    if (v.requested_motion) {
        const auto &proposal = *v.requested_motion; nb::dict requested;
        requested["command"] = reference(comm::Reference{proposal.session, proposal.id}); requested["gripper_open"] = nb::cast(proposal.gripper_open);
        nb::dict motion;
        std::visit([&](const auto &payload) {
            using T = std::remove_cvref_t<decltype(payload)>;
            if constexpr (std::same_as<T, robot::resources::Positions> || std::same_as<T, robot::resources::Velocities>) {
                motion["kind"] = std::same_as<T, robot::resources::Positions> ? "joint_position" : "joint_velocity";
                motion["values"] = coordinates(box, payload.values());
            } else if constexpr (std::same_as<T, comm::Pose>) { motion = pose(payload.value()); motion["kind"] = "pose";
                nb::dict auxiliary; for (const auto &joint : payload.auxiliary()) auxiliary[nb::str(joint.name.c_str())] = joint.position;
                motion["auxiliary_positions"] = auxiliary; }
            else if constexpr (std::same_as<T, comm::Twist>) {
                motion["kind"] = "twist"; motion["values"] = array(box, payload.value().data(), 6);
            } else {
                motion["kind"] = std::same_as<T, robot::execution::Trajectory> ? "trajectory" : "joint_schedule";
                const auto &points = [&]() -> const auto & { if constexpr (std::same_as<T, robot::execution::Trajectory>) return payload.points(); else return payload.path().points(); }();
                nb::list path;
                for (const auto &point : points) {
                    nb::dict p; p["position"] = coordinates(box, point.position.values()); p["velocity"] = coordinates(box, point.velocity.values());
                    p["seconds"] = std::chrono::duration<double>(point.offset).count();
                    if constexpr (requires { point.acceleration; }) p["acceleration"] = coordinates(box, point.acceleration.values());
                    path.append(p);
                }
                motion["points"] = path;
                if constexpr (std::same_as<T, robot::execution::ScheduledMotion>) { motion["start_ns"] = payload.start().time_since_epoch().count(); motion["epoch"] = payload.epoch(); }
            }
        }, proposal.motion);
        requested["motion"] = motion; out["requested_motion"] = requested;
    }
    out["host_control_completed_ns"] = std::chrono::duration_cast<core::duration>(v.control_received.time_since_epoch()).count();
    return out;
}
struct NativeRuntime {
    std::unique_ptr<application::Runtime> runtime;
    NativeRuntime(const application::DeploymentSpecification &spec,const std::string &backend,bool stepped,std::optional<double> period)
        :runtime(std::make_unique<application::Runtime>(spec,backend,stepped?application::RunMode::Stepped:application::RunMode::Realtime,period)) {}
    ~NativeRuntime() { if(PyGILState_Check()) { nb::gil_scoped_release release; runtime->close(); } else runtime->close(); }
};
nb::dict fitReport(const std::optional<robot::execution::TargetFitReport> &fit) {
    nb::dict result; if(fit) { result["deviation"]=nb::cast(fit->deviation); result["budget"]=nb::cast(fit->budget); result["saturation"]=fit->saturation; } return result;
}

}
#define F(TYPE, MEMBER) .def_rw(#MEMBER, &TYPE::MEMBER)
NB_MODULE(_native, m) {
    m.def("fit_timed_targets",[](const std::vector<std::vector<double>> &actions,std::int64_t origin,std::int64_t period,
                                  const std::vector<double> &lower,const std::vector<double> &upper,const std::vector<double> &velocity,
                                  const std::vector<double> &acceleration,const std::vector<double> &jerk,const std::vector<double> &tolerance){
        comm::TimedJointTargetsRequest request;request.positions=actions;request.origin=core::Time{core::duration{origin}};request.period=core::duration{period};request.tolerance=tolerance;
        if(actions.size()<5 || actions.size()>256 || lower.empty())throw std::invalid_argument("timed target samples exceed fitting budget");
        Eigen::MatrixXd values(actions.size(),lower.size());
        for(std::size_t i=0;i<actions.size();++i){if(actions[i].size()!=lower.size())throw std::invalid_argument("action rows disagree with joint count");for(std::size_t j=0;j<lower.size();++j)values(i,j)=actions[i][j];}
        const auto coordinates=[](const auto &v){return Eigen::Map<const Eigen::VectorXd>(v.data(),v.size()).eval();};
        const robot::execution::TargetFitLimits limits{coordinates(lower),coordinates(upper),coordinates(velocity),coordinates(acceleration),coordinates(jerk)};
        robot::execution::FittedTargets fitted;{nb::gil_scoped_release release;fitted=robot::execution::fitTimedTargets(request,limits,values,false);}
        const auto rows=[](const Eigen::MatrixXd &matrix){std::vector<std::vector<double>> result;for(Eigen::Index i=0;i<matrix.rows();++i){std::vector<double> row;for(Eigen::Index j=0;j<matrix.cols();++j)row.push_back(matrix(i,j));result.push_back(std::move(row));}return result;};
        nb::dict result;result["position"]=nb::cast(rows(fitted.position));result["velocity"]=nb::cast(rows(fitted.velocity));result["acceleration"]=nb::cast(rows(fitted.acceleration));
        result["deviation"]=nb::cast(fitted.report.deviation);result["budget"]=nb::cast(fitted.report.budget);result["saturation"]=fitted.report.saturation;result["origin_ns"]=origin;result["period_ns"]=period;return result;
    });

    nb::class_<application::RobotSpecification>(m, "RobotSpecification").def(nb::init<>())
        F(application::RobotSpecification, description) F(application::RobotSpecification, base) F(application::RobotSpecification, tip)
        F(application::RobotSpecification, auxiliary_joints) F(application::RobotSpecification, initial) F(application::RobotSpecification, standby)
        F(application::RobotSpecification, control_limits);
    nb::class_<robot::resources::ControlLimits>(m, "ControlLimit").def(nb::init<>())
        F(robot::resources::ControlLimits, joint) F(robot::resources::ControlLimits, lower) F(robot::resources::ControlLimits, upper) F(robot::resources::ControlLimits, velocity);
    nb::class_<application::ControllerSpecification>(m, "ControllerSpecification").def(nb::init<>())
        F(application::ControllerSpecification, period) F(application::ControllerSpecification, standby_duration) F(application::ControllerSpecification, speed_fraction)
        F(application::ControllerSpecification, acceleration_time) F(application::ControllerSpecification, jerk_time) F(application::ControllerSpecification, position_gain)
        F(application::ControllerSpecification, damping) F(application::ControllerSpecification, orientation_gain) F(application::ControllerSpecification, tracking_error);
    nb::class_<robot::resources::SensorDescription>(m, "SensorDescription").def(nb::init<>())
        F(robot::resources::SensorDescription, name) F(robot::resources::SensorDescription, frame) F(robot::resources::SensorDescription, calibration) F(robot::resources::SensorDescription, units);
    nb::class_<adapters::backends::MujocoConfig>(m, "MujocoSpecification").def(nb::init<>())
        F(adapters::backends::MujocoConfig, file) F(adapters::backends::MujocoConfig, base_frame) F(adapters::backends::MujocoConfig, tip_frame)
        F(adapters::backends::MujocoConfig, binary_actuator) F(adapters::backends::MujocoConfig, open_position) F(adapters::backends::MujocoConfig, closed_position)
        F(adapters::backends::MujocoConfig, sensors);
    nb::class_<parol6::config::SerialConfig>(m, "SerialSpecification").def(nb::init<>())
        F(parol6::config::SerialConfig, device) F(parol6::config::SerialConfig, baud) F(parol6::config::SerialConfig, startup_outputs)
        F(parol6::config::SerialConfig, first_response_timeout_seconds) F(parol6::config::SerialConfig, transmission_timeout_seconds)
        F(parol6::config::SerialConfig, telemetry_timeout_seconds) F(parol6::config::SerialConfig, pneumatic_open_high);
    nb::enum_<robot::execution::drivers::OverrunPolicy>(m, "OverrunPolicy").value("Fault", robot::execution::drivers::OverrunPolicy::Fault).value("Brake", robot::execution::drivers::OverrunPolicy::Brake);
    nb::class_<robot::execution::drivers::HostTiming>(m, "HostTiming")
        .def(nb::new_([](double maximum, robot::execution::drivers::OverrunPolicy policy) { return robot::execution::drivers::HostTiming{core::Period{application::seconds(maximum)}, policy}; }));
    nb::class_<application::DeploymentSpecification>(m, "Specification").def(nb::init<>())
        F(application::DeploymentSpecification, external_sensors) F(application::DeploymentSpecification, robot) F(application::DeploymentSpecification, controller) F(application::DeploymentSpecification, mujoco)
        F(application::DeploymentSpecification, serial) F(application::DeploymentSpecification, wire_joints) F(application::DeploymentSpecification, host_timing);
    nb::class_<comm::ModelJointOrder>(m, "ModelJointOrder").def(nb::init<>());
    nb::class_<comm::NamedJointOrder>(m, "NamedJointOrder").def(nb::init<>()) F(comm::NamedJointOrder, names);
    nb::class_<comm::JointPositionRequest>(m, "JointPosition").def(nb::init<>()) F(comm::JointPositionRequest, order) F(comm::JointPositionRequest, values);
    nb::class_<comm::JointVelocityRequest>(m, "JointVelocity").def(nb::init<>()) F(comm::JointVelocityRequest, order) F(comm::JointVelocityRequest, values);
    nb::class_<comm::PoseRequest>(m, "Pose").def(nb::init<>()) F(comm::PoseRequest, frame) F(comm::PoseRequest, position) F(comm::PoseRequest, quaternion_wxyz) F(comm::PoseRequest, auxiliary_positions);
    nb::class_<comm::TwistRequest>(m, "Twist").def(nb::init<>()) F(comm::TwistRequest, frame) F(comm::TwistRequest, values);
    nb::class_<comm::WaypointRequest>(m, "Waypoint").def(nb::init<>()) F(comm::WaypointRequest, position) F(comm::WaypointRequest, velocity)
        .def_prop_rw("offset", [](const comm::WaypointRequest &v) { return std::chrono::duration<double>(v.offset).count(); }, [](comm::WaypointRequest &v, double t) {
            if (!robot::boundary::Finite::accepts(t) || t < 0 || t >= std::chrono::duration<double>(core::duration::max()).count()) throw std::invalid_argument("invalid offset");
            v.offset = std::chrono::duration_cast<core::duration>(std::chrono::duration<double>{t}); });
    nb::class_<comm::TrajectoryRequest>(m, "Trajectory").def(nb::init<>()) F(comm::TrajectoryRequest, order) F(comm::TrajectoryRequest, points);
    nb::class_<comm::ReferencePointRequest>(m, "ReferencePoint").def(nb::init<>()) F(comm::ReferencePointRequest, position) F(comm::ReferencePointRequest, velocity)
        F(comm::ReferencePointRequest, acceleration) .def_prop_rw("offset", [](const comm::ReferencePointRequest &v) { return std::chrono::duration<double>(v.offset).count(); },
        [](comm::ReferencePointRequest &v, double t) { v.offset = application::seconds(t); });
    nb::class_<comm::ScheduleRequest>(m, "Schedule").def(nb::init<>()) F(comm::ScheduleRequest, order) F(comm::ScheduleRequest, epoch) F(comm::ScheduleRequest, points)
        .def_prop_rw("start_ns", [](const comm::ScheduleRequest &v) { return v.start.time_since_epoch().count(); }, [](comm::ScheduleRequest &v, std::int64_t t) { v.start = core::Time{core::duration{t}}; })
        .def_prop_rw("replaces", [](const comm::ScheduleRequest &v) -> std::optional<std::uint64_t> { if (v.replaces) return v.replaces->value; return {}; },
                     [](comm::ScheduleRequest &v, std::optional<std::uint64_t> id) { v.replaces = id ? std::optional{comm::Id{*id}} : std::nullopt; });
    nb::enum_<comm::OperatorAction>(m, "Action")
        .value("Enable", comm::OperatorAction::Enable).value("Disable", comm::OperatorAction::Disable).value("Stop", comm::OperatorAction::Stop)
        .value("Hold", comm::OperatorAction::Hold).value("Standby", comm::OperatorAction::Standby).value("OpenGripper", comm::OperatorAction::OpenGripper)
        .value("CloseGripper", comm::OperatorAction::CloseGripper).value("Acquire", comm::OperatorAction::Acquire).value("Release", comm::OperatorAction::Release).value("Takeover", comm::OperatorAction::Takeover).value("Pause",comm::OperatorAction::Pause);
    nb::class_<Snapshot>(m, "Snapshot").def_prop_ro("data", &snapshotData);

    nb::class_<comm::TimedJointTargetsRequest>(m,"TimedJointTargets").def(nb::init<>())
        F(comm::TimedJointTargetsRequest,order) F(comm::TimedJointTargetsRequest,positions) F(comm::TimedJointTargetsRequest,runtime_instance)
        F(comm::TimedJointTargetsRequest,epoch) F(comm::TimedJointTargetsRequest,observation_sequence) F(comm::TimedJointTargetsRequest,tolerance)
        .def_prop_rw("origin_ns",[](const comm::TimedJointTargetsRequest &v){return v.origin.time_since_epoch().count();},[](comm::TimedJointTargetsRequest &v,std::int64_t t){v.origin=core::Time{core::duration{t}};})
        .def_prop_rw("start_ns",[](const comm::TimedJointTargetsRequest &v){return v.start.time_since_epoch().count();},[](comm::TimedJointTargetsRequest &v,std::int64_t t){v.start=core::Time{core::duration{t}};})
        .def_prop_rw("period_ns",[](const comm::TimedJointTargetsRequest &v){return v.period.count();},[](comm::TimedJointTargetsRequest &v,std::int64_t t){v.period=core::duration{t};})
        .def_prop_rw("lead_ns",[](const comm::TimedJointTargetsRequest &v){return v.lead.count();},[](comm::TimedJointTargetsRequest &v,std::int64_t t){v.lead=core::duration{t};})
        .def_prop_rw("join_seconds",[](const comm::TimedJointTargetsRequest &v){return std::chrono::duration<double>(v.join).count();},[](comm::TimedJointTargetsRequest &v,double t){v.join=application::seconds(t);})
        .def_prop_rw("replaces",[](const comm::TimedJointTargetsRequest &v)->std::optional<std::uint64_t>{if(v.replaces)return v.replaces->value;return {};},[](comm::TimedJointTargetsRequest &v,std::optional<std::uint64_t> id){v.replaces=id?std::optional{comm::Id{*id}}:std::nullopt;});
    nb::class_<ImagePlane>(m,"ImagePlane").def(nb::init<>())
        F(ImagePlane,name) F(ImagePlane,dtype) F(ImagePlane,frame) F(ImagePlane,calibration) F(ImagePlane,source_clock)
        F(ImagePlane,shape) F(ImagePlane,offset) F(ImagePlane,bytes) F(ImagePlane,sequence) F(ImagePlane,epoch)
        F(ImagePlane,source_time_ns) F(ImagePlane,host_received_ns) F(ImagePlane,intrinsics) F(ImagePlane,world_from_camera);
    nb::class_<application::Source>(m,"Source")
        .def_prop_ro("source_id",[](const application::Source &v){return v.id().value;})
        .def_prop_ro("name",&application::Source::name).def_prop_ro("instance",&application::Source::instance)
        .def("operate",&application::Source::operate,nb::arg("action"),nb::arg("generation")=nb::none(),nb::call_guard<nb::gil_scoped_release>())
        .def("renew",[](application::Source &v,std::uint64_t generation,const Snapshot &basis,std::optional<double> ttl){return v.renew(generation,*basis.value,ttl);},nb::call_guard<nb::gil_scoped_release>())
        .def("submit",[](application::Source &v,comm::MotionRequest motion,std::uint64_t generation,double ttl,const Snapshot &basis,std::optional<bool> gripper){
            application::Submission result; {nb::gil_scoped_release release;result=v.submit(std::move(motion),generation,ttl,*basis.value,gripper);}
            return nb::make_tuple(result.command_id,result.queued,fitReport(result.fit));
        });
    nb::class_<NativeRuntime>(m,"Runtime")
        .def(nb::init<const application::DeploymentSpecification &,const std::string &,bool,std::optional<double>>(),nb::call_guard<nb::gil_scoped_release>())
        .def("source",[](NativeRuntime &v,const std::string &name,const std::string &instance){return v.runtime->source(name,instance);},nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("operator_source",[](NativeRuntime &v){return v.runtime->operatorSource();})
        .def("frames",[](NativeRuntime &v,const Snapshot &basis){
            robot::resources::Workspace workspace{v.runtime->communication()->model()};nb::dict result;
            if(basis.value->backend.state){const auto transforms=[&]{nb::gil_scoped_release release;return workspace.frames(basis.value->backend.state->joints.positions());}();
                for(const auto &[name,transform]:transforms)result[nb::str(name.c_str())]=pose(transform);}
            return result;
        })
        .def("snapshot",[](NativeRuntime &v){return Snapshot{v.runtime->snapshot()};},nb::call_guard<nb::gil_scoped_release>())
        .def("operate",[](NativeRuntime &v,comm::OperatorAction action){return v.runtime->operate(action);},nb::call_guard<nb::gil_scoped_release>())
        .def("step",[](NativeRuntime &v,std::size_t n){v.runtime->step(n);},nb::call_guard<nb::gil_scoped_release>())
        .def("tick",[](NativeRuntime &v){v.runtime->tick();},nb::call_guard<nb::gil_scoped_release>())
        .def("reset",[](NativeRuntime &v){v.runtime->reset();},nb::call_guard<nb::gil_scoped_release>())
        .def("wait",[](NativeRuntime &v,std::uint64_t epoch,std::uint64_t sequence,double timeout){v.runtime->waitChange(epoch,sequence,timeout);},nb::call_guard<nb::gil_scoped_release>())
        .def("publish_capture",[](NativeRuntime &v,const Snapshot &basis,const std::vector<ImagePlane> &planes,nb::bytes raw){
            std::vector<application::ImageSample> images;std::size_t expected=0;
            for(const auto &plane:planes){
                const bool depth=plane.dtype=="float32";
                if((!depth && plane.dtype!="uint8") || plane.shape.size()!=(depth?2:3) || (!depth && plane.shape[2]!=3) ||
                   plane.shape[0]==0 || plane.shape[1]==0 || plane.shape[0]>UINT32_MAX || plane.shape[1]>UINT32_MAX ||
                   plane.offset!=expected || plane.bytes>raw.size()-std::min(raw.size(),expected))throw std::invalid_argument("invalid image plane layout");
                auto storage=std::make_shared<std::vector<std::byte>>(plane.bytes);std::memcpy(storage->data(),raw.c_str()+plane.offset,plane.bytes);
                robot::backend::SampleStamp stamp{plane.sequence,basis.value->control_time,plane.epoch,plane.host_received_ns?
                    core::HostClock::time_point{core::duration{*plane.host_received_ns}}:core::HostClock::now(),plane.source_time_ns,plane.source_clock};
                const auto width=static_cast<std::uint32_t>(plane.shape[1]),height=static_cast<std::uint32_t>(plane.shape[0]);
                if(std::uint64_t(width)*(depth?4:3)>UINT32_MAX)throw std::invalid_argument("image stride overflow");
                images.push_back({plane.name,plane.frame,plane.calibration,std::move(stamp),{width,height,width*(depth?4u:3u),depth?"32FC1":"rgb8",std::move(storage)},plane.intrinsics,plane.world_from_camera});
                expected+=plane.bytes;
            }
            if(expected!=raw.size())throw std::invalid_argument("unused capture bytes");
            nb::gil_scoped_release release;v.runtime->publishCapture(*basis.value,std::move(images));
        })
        .def("capture",[](NativeRuntime &v)->nb::object {
            auto capture=v.runtime->capture();if(!capture)return nb::none();nb::dict result;
            result["observation"]=nb::cast(Snapshot{*capture->observation});result["sequence"]=capture->sequence;nb::dict images;
            for(const auto &sample:capture->images){nb::dict image;image["frame"]=sample.frame;image["calibration"]=sample.calibration;stamp(image,sample.stamp);
                image["dtype"]=sample.image.encoding=="rgb8"?"uint8":"float32";image["width"]=sample.image.width;image["height"]=sample.image.height;
                image["pixels"]=nb::bytes(reinterpret_cast<const char *>(sample.image.bytes->data()),sample.image.bytes->size());
                image["intrinsics"]=nb::cast(sample.intrinsics);image["world_from_camera"]=nb::cast(sample.world_from_camera);images[nb::str(sample.name.c_str())]=image;}
            result["images"]=images;return result;
        })
        .def("publish_numeric",[](NativeRuntime &v,std::string name,std::vector<double> values,std::uint64_t sequence,std::string clock,std::optional<std::int64_t> source_ns,std::uint64_t epoch){
            robot::backend::SampleStamp stamp{sequence,{},epoch,core::HostClock::now(),source_ns,std::move(clock)};
            v.runtime->publishSensor({std::move(name),{},{},std::move(stamp),robot::backend::NumericSensor{std::move(values),{}}});
        },nb::call_guard<nb::gil_scoped_release>())
        .def("close",[](NativeRuntime &v){v.runtime->close();},nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("description",[](NativeRuntime &v){
            const auto &c=*v.runtime->communication();const auto &model=c.model().description();nb::dict d;
            d["instance"]=v.runtime->instance();d["frames"]=nb::cast(model.frames);d["auxiliary_joints"]=nb::cast(model.auxiliary_joints);
            d["name"]=model.name;d["model_id"]=model.identity;d["base"]=model.base;d["tip"]=model.tip;d["joints"]=nb::cast(model.joints);
            d["joint_units"]=nb::cast(model.units);d["lower"]=nb::cast(vector(model.lower));d["upper"]=nb::cast(vector(model.upper));d["speed"]=nb::cast(vector(model.speed));d["period_ns"]=c.controlPeriod().value().count();
            d["configuration_size"]=model.joints.size();d["tangent_size"]=model.joints.size();
            d["sdk_features"]=nb::cast(std::vector<std::string>{"sources","explicit_renewal","timed_joint_targets","capture"});
            d["motion"]=nb::cast(std::vector<std::string>{"joint_position","joint_velocity","pose","twist","trajectory","joint_schedule","timed_joint_targets"});
            const auto &cap=c.capabilities();nb::dict caps;caps["backend"]=cap.name;caps["feedback"]=cap.feedback_source;caps["binary_tool"]=cap.binary_tool;
            caps["homing"]=cap.homing_operation;caps["power_off"]=cap.inhibit==robot::backend::InhibitBehavior::ActuatorForcesOff;
            caps["disable_behavior"]=cap.inhibit==robot::backend::InhibitBehavior::ActuatorForcesOff?"actuator_forces_off":"firmware_motion_inhibit";
            caps["required_period_ns"]=cap.required_period?nb::cast(cap.required_period->value().count()):nb::object(nb::none());d["capabilities"]=caps;
            nb::dict limits;limits["velocity"]=nb::cast(vector(c.controllerConfig().velocity()));limits["acceleration"]=nb::cast(vector(c.controllerConfig().acceleration()));limits["jerk"]=nb::cast(vector(c.controllerConfig().jerk()));d["execution_limits"]=limits;
            nb::dict schedule;schedule["max_horizon_ns"]=robot::execution::ScheduledMotion::maximum_horizon.count();schedule["minimum_lead_ticks"]=2;schedule["pending_capacity"]=1;schedule["maximum_target_samples"]=256;d["schedule"]=schedule;
            nb::dict authority;authority["renewal"]="explicit";authority["takeover"]=true;authority["lease_ns"]=c.description().authority_lease.count();authority["buffer_underrun"]="brake_to_owned_hold";d["authority"]=authority;
            nb::dict timeline;const auto timing=c.description().timing;timeline["owner"]=timing==comm::TimingProfile::Simulation?"simulation":"host";
            timeline["interval"]=timing==comm::TimingProfile::Simulation?"fixed":"actual";timeline["explicit_stepping"]=timing==comm::TimingProfile::Simulation;d["timeline"]=timeline;
            nb::dict conventions;conventions["pose"]="base_from_tool";conventions["quaternion"]="wxyz";conventions["twist"]="linear_then_angular";d["conventions"]=conventions;
            nb::list sensors;for(const auto &sensor:c.description().sensors){nb::dict item;item["name"]=sensor.name;item["frame"]=sensor.frame;item["calibration"]=sensor.calibration;item["units"]=nb::cast(sensor.units);sensors.append(item);}d["sensor_descriptions"]=sensors;return d;
        });
}
