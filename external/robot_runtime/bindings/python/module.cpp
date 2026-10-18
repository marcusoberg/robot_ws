#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include "robot_runtime/simulation.hpp"
namespace nb=nanobind;
using namespace robot_runtime;
using namespace nb::literals;
NB_MODULE(_native,m) {
    nb::class_<StopProfile>(m,"StopProfile").def(nb::init<>())
        .def_rw("velocity_threshold",&StopProfile::velocity_threshold)
        .def_rw("settled_duration",&StopProfile::settled_duration).def_rw("timeout",&StopProfile::timeout);
    nb::class_<RobotSpec>(m,"RobotSpec").def(nb::init<>())
        .def_rw("joints",&RobotSpec::joints).def_rw("units",&RobotSpec::units)
        .def_rw("lower",&RobotSpec::lower).def_rw("upper",&RobotSpec::upper)
        .def_rw("backend",&RobotSpec::backend).def_rw("command_mode",&RobotSpec::command_mode)
        .def("validate",&RobotSpec::validate);
    nb::class_<Measurement>(m,"Measurement").def_ro("position",&Measurement::position)
        .def_ro("velocity",&Measurement::velocity).def_ro("available",&Measurement::available).def_ro("fault",&Measurement::fault);
    nb::class_<Status>(m,"Status").def_prop_ro("mode",[](const Status& s){return mode_name(s.mode);})
        .def_ro("measured",&Status::measured).def_ro("reference",&Status::reference)
        .def_ro("session",&Status::session).def_ro("sequence",&Status::sequence)
        .def_ro("delivered_sequence",&Status::delivered_sequence).def_ro("time",&Status::time)
        .def_ro("source",&Status::source).def_ro("reason",&Status::reason);
    nb::class_<Trace>(m,"Trace").def_ro("begin",&Trace::begin).def_ro("end",&Trace::end)
        .def_ro("reference",&Trace::reference).def_ro("measured",&Trace::measured).def_ro("sequence",&Trace::sequence);
    nb::class_<Contact>(m,"Contact").def_ro("geom1",&Contact::geom1).def_ro("geom2",&Contact::geom2)
        .def_ro("ancestors1",&Contact::ancestors1).def_ro("ancestors2",&Contact::ancestors2).def_ro("normal_force",&Contact::normal_force);
    nb::class_<Snapshot>(m,"Snapshot").def_ro("status",&Snapshot::status).def_ro("time",&Snapshot::time)
        .def_ro("episode",&Snapshot::episode).def_ro("observation",&Snapshot::observation)
        .def_ro("integration",&Snapshot::integration).def_ro("trace",&Snapshot::trace)
        .def_ro("bodies",&Snapshot::bodies).def_ro("contacts",&Snapshot::contacts).def_ro("wall_lag_seconds",&Snapshot::wall_lag_seconds)
        .def_ro("host_acquired_ns",&Snapshot::host_acquired_ns);
    nb::class_<Simulation>(m,"Simulation")
        .def(nb::init<RobotSpec,const std::string&,Positions,Time,Time,StopProfile,Time>(),"spec"_a,"scene"_a,"initial"_a,"control_period"_a=10'000'000,"action_period"_a=50'000'000,"stop"_a=StopProfile{},"physics_period"_a=1'000'000)
        .def("reset",&Simulation::reset,nb::call_guard<nb::gil_scoped_release>())
        .def("enable",&Simulation::enable,nb::call_guard<nb::gil_scoped_release>())
        .def("activate",&Simulation::activate,"source"_a,nb::call_guard<nb::gil_scoped_release>())
        .def("submit",&Simulation::submit,"goal"_a,"session"_a,"sequence"_a,"valid_for"_a=300'000'000,nb::call_guard<nb::gil_scoped_release>())
        .def("step",&Simulation::step,nb::call_guard<nb::gil_scoped_release>())
        .def("advance",&Simulation::advance,nb::call_guard<nb::gil_scoped_release>())
        .def("observe",&Simulation::observe,nb::call_guard<nb::gil_scoped_release>())
        .def("latest_boundary",&Simulation::latest_boundary,nb::call_guard<nb::gil_scoped_release>())
        .def("schedule",&Simulation::schedule,"goal"_a,"session"_a,"sequence"_a,"application_time"_a,"interval_only"_a=false,nb::call_guard<nb::gil_scoped_release>())
        .def("pause",&Simulation::pause,"reason"_a="paused",nb::call_guard<nb::gil_scoped_release>())
        .def("disable",&Simulation::disable,nb::call_guard<nb::gil_scoped_release>())
        .def("start_paced",&Simulation::start_paced,nb::call_guard<nb::gil_scoped_release>())
        .def("stop_paced",&Simulation::stop_paced,nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("paced",&Simulation::paced,nb::call_guard<nb::gil_scoped_release>());
}
