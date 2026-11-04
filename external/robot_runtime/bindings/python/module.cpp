#include "robot_runtime/simulation.hpp"
#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#ifdef ROBOT_RUNTIME_WITH_PAROL6
#include "robot_runtime/parol6_serial.hpp"
#endif
namespace nb = nanobind;
using namespace robot_runtime;
using namespace nb::literals;
NB_MODULE(_native, m) {
    nb::class_<StopProfile>(m, "StopProfile")
        .def(nb::init<>())
        .def_rw("velocity_threshold", &StopProfile::velocity_threshold)
        .def_rw("settled_duration", &StopProfile::settled_duration)
        .def_rw("timeout", &StopProfile::timeout);
    nb::class_<RobotSpec>(m, "RobotSpec")
        .def(nb::init<>())
        .def_rw("joints", &RobotSpec::joints)
        .def_rw("units", &RobotSpec::units)
        .def_rw("lower", &RobotSpec::lower)
        .def_rw("upper", &RobotSpec::upper)
        .def_rw("backend", &RobotSpec::backend)
        .def_rw("command_mode", &RobotSpec::command_mode)
        .def("validate", &RobotSpec::validate);
    nb::class_<Measurement>(m, "Measurement")
        .def_ro("position", &Measurement::position)
        .def_ro("velocity", &Measurement::velocity)
        .def_ro("available", &Measurement::available)
        .def_ro("fault", &Measurement::fault);
    nb::class_<Status>(m, "Status")
        .def_prop_ro("mode", [](const Status &s) { return mode_name(s.mode); })
        .def_ro("measured", &Status::measured)
        .def_ro("reference", &Status::reference)
        .def_ro("session", &Status::session)
        .def_ro("sequence", &Status::sequence)
        .def_ro("delivered_sequence", &Status::delivered_sequence)
        .def_ro("time", &Status::time)
        .def_ro("source", &Status::source)
        .def_ro("reason", &Status::reason);
    nb::class_<Trace>(m, "Trace")
        .def_ro("begin", &Trace::begin)
        .def_ro("end", &Trace::end)
        .def_ro("reference", &Trace::reference)
        .def_ro("measured", &Trace::measured)
        .def_ro("sequence", &Trace::sequence);
    nb::class_<Contact>(m, "Contact")
        .def_ro("geom1", &Contact::geom1)
        .def_ro("geom2", &Contact::geom2)
        .def_ro("ancestors1", &Contact::ancestors1)
        .def_ro("ancestors2", &Contact::ancestors2)
        .def_ro("normal_force", &Contact::normal_force);
    nb::class_<Snapshot>(m, "Snapshot")
        .def_ro("status", &Snapshot::status)
        .def_ro("time", &Snapshot::time)
        .def_ro("episode", &Snapshot::episode)
        .def_ro("observation", &Snapshot::observation)
        .def_ro("integration", &Snapshot::integration)
        .def_ro("trace", &Snapshot::trace)
        .def_ro("bodies", &Snapshot::bodies)
        .def_ro("contacts", &Snapshot::contacts)
        .def_ro("wall_lag_seconds", &Snapshot::wall_lag_seconds)
        .def_ro("host_acquired_ns", &Snapshot::host_acquired_ns);
    nb::class_<Simulation>(m, "Simulation")
        .def(nb::init<RobotSpec, const std::string &, Positions, Time, Time, StopProfile, Time>(),
             "spec"_a, "scene"_a, "initial"_a, "control_period"_a = 10'000'000,
             "action_period"_a = 50'000'000, "stop"_a = StopProfile{},
             "physics_period"_a = 1'000'000)
        .def("reset", &Simulation::reset, nb::call_guard<nb::gil_scoped_release>())
        .def("enable", &Simulation::enable, nb::call_guard<nb::gil_scoped_release>())
        .def("activate", &Simulation::activate, "source"_a,
             nb::call_guard<nb::gil_scoped_release>())
        .def("submit", &Simulation::submit, "goal"_a, "session"_a, "sequence"_a,
             "valid_for"_a = 300'000'000, nb::call_guard<nb::gil_scoped_release>())
        .def("step", &Simulation::step, nb::call_guard<nb::gil_scoped_release>())
        .def("advance", &Simulation::advance, nb::call_guard<nb::gil_scoped_release>())
        .def("observe", &Simulation::observe, nb::call_guard<nb::gil_scoped_release>())
        .def("latest_boundary", &Simulation::latest_boundary,
             nb::call_guard<nb::gil_scoped_release>())
        .def("schedule", &Simulation::schedule, "goal"_a, "session"_a, "sequence"_a,
             "application_time"_a, "interval_only"_a = false,
             nb::call_guard<nb::gil_scoped_release>())
        .def("pause", &Simulation::pause, "reason"_a = "paused",
             nb::call_guard<nb::gil_scoped_release>())
        .def("disable", &Simulation::disable, nb::call_guard<nb::gil_scoped_release>())
        .def("start_paced", &Simulation::start_paced, nb::call_guard<nb::gil_scoped_release>())
        .def("stop_paced", &Simulation::stop_paced, nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("paced", &Simulation::paced, nb::call_guard<nb::gil_scoped_release>());
#ifdef ROBOT_RUNTIME_WITH_PAROL6
    nb::class_<Parol6Telemetry>(m, "Parol6Telemetry")
        .def_ro("mode", &Parol6Telemetry::mode)
        .def_ro("reason", &Parol6Telemetry::reason)
        .def_ro("command_error", &Parol6Telemetry::command_error)
        .def_ro("position", &Parol6Telemetry::position)
        .def_ro("velocity", &Parol6Telemetry::velocity)
        .def_ro("reference", &Parol6Telemetry::reference)
        .def_ro("transmitted_position", &Parol6Telemetry::transmitted_position)
        .def_ro("transmitted_velocity", &Parol6Telemetry::transmitted_velocity)
        .def_ro("sample_time_ns", &Parol6Telemetry::sample_time_ns)
        .def_ro("sample_wall_time_ns", &Parol6Telemetry::sample_wall_time_ns)
        .def_ro("receive_time_ns", &Parol6Telemetry::receive_time_ns)
        .def_ro("transmit_time_ns", &Parol6Telemetry::transmit_time_ns)
        .def_ro("round_trip_seconds", &Parol6Telemetry::round_trip_seconds)
        .def_ro("timing_uncertainty_seconds", &Parol6Telemetry::timing_uncertainty_seconds)
        .def_ro("motion_elapsed_seconds", &Parol6Telemetry::motion_elapsed_seconds)
        .def_ro("motion_id", &Parol6Telemetry::motion_id)
        .def_ro("reference_frame", &Parol6Telemetry::reference_frame)
        .def_ro("transmitted_motion_id", &Parol6Telemetry::transmitted_motion_id)
        .def_ro("reference_motion_id", &Parol6Telemetry::reference_motion_id)
        .def_ro("write_complete_time_ns", &Parol6Telemetry::write_complete_time_ns)
        .def_ro("evaluation_time_ns", &Parol6Telemetry::evaluation_time_ns)
        .def_ro("reference_time_ns", &Parol6Telemetry::reference_time_ns)
        .def_ro("reference_valid", &Parol6Telemetry::reference_valid)
        .def_ro("motion_complete", &Parol6Telemetry::motion_complete)
        .def_ro("steps", &Parol6Telemetry::steps)
        .def_ro("speed_steps", &Parol6Telemetry::speed_steps)
        .def_ro("payload", &Parol6Telemetry::payload)
        .def_ro("frames", &Parol6Telemetry::frames)
        .def_ro("age_seconds", &Parol6Telemetry::age_seconds)
        .def_ro("cycle_seconds", &Parol6Telemetry::cycle_seconds)
        .def_ro("goal_interval_seconds", &Parol6Telemetry::goal_interval_seconds)
        .def_ro("connected", &Parol6Telemetry::connected)
        .def_ro("estop", &Parol6Telemetry::estop)
        .def_ro("estop_verified", &Parol6Telemetry::estop_verified)
        .def_ro("homed_this_session", &Parol6Telemetry::homed_this_session)
        .def_ro("homed_bits", &Parol6Telemetry::homed_bits)
        .def_ro("io_bits", &Parol6Telemetry::io_bits)
        .def_ro("temperature_bits", &Parol6Telemetry::temperature_bits)
        .def_ro("position_error_bits", &Parol6Telemetry::position_error_bits)
        .def_ro("timing_ticks", &Parol6Telemetry::timing_ticks)
        .def_ro("timeout_value", &Parol6Telemetry::timeout_value)
        .def_ro("command_echo", &Parol6Telemetry::command_echo)
        .def_ro("gripper_id", &Parol6Telemetry::gripper_id)
        .def_ro("gripper_position", &Parol6Telemetry::gripper_position)
        .def_ro("gripper_speed", &Parol6Telemetry::gripper_speed)
        .def_ro("gripper_current", &Parol6Telemetry::gripper_current)
        .def_ro("gripper_status", &Parol6Telemetry::gripper_status)
        .def_ro("object_detection", &Parol6Telemetry::object_detection);
    nb::class_<Parol6Serial>(m, "Parol6Serial")
        .def(nb::init<const std::string &, int>(), "port"_a, "baud"_a = 3'000'000)
        .def_static("standby", &Parol6Serial::standby)
        .def("observe", &Parol6Serial::observe, nb::call_guard<nb::gil_scoped_release>())
        .def("drain_samples", &Parol6Serial::drain_samples, nb::call_guard<nb::gil_scoped_release>())
        .def("move_joint", &Parol6Serial::move_joint, nb::call_guard<nb::gil_scoped_release>())
        .def("follow_path", &Parol6Serial::follow_path, nb::call_guard<nb::gil_scoped_release>())
        .def("live_target", &Parol6Serial::live_target, nb::call_guard<nb::gil_scoped_release>())
        .def("heartbeat", &Parol6Serial::heartbeat, nb::call_guard<nb::gil_scoped_release>())
        .def("home", &Parol6Serial::home, nb::call_guard<nb::gil_scoped_release>())
        .def("enable", &Parol6Serial::enable, nb::call_guard<nb::gil_scoped_release>())
        .def("reset", &Parol6Serial::reset, nb::call_guard<nb::gil_scoped_release>())
        .def("keepalive", &Parol6Serial::keepalive, nb::call_guard<nb::gil_scoped_release>())
        .def("start", &Parol6Serial::start, nb::call_guard<nb::gil_scoped_release>())
        .def("submit", &Parol6Serial::submit, "positions"_a, "velocities"_a = Positions{},
             nb::call_guard<nb::gil_scoped_release>())
        .def("pause", &Parol6Serial::pause, nb::call_guard<nb::gil_scoped_release>())
        .def("stop", &Parol6Serial::stop, nb::call_guard<nb::gil_scoped_release>())
        .def("close", &Parol6Serial::close, nb::call_guard<nb::gil_scoped_release>());
#endif
}
