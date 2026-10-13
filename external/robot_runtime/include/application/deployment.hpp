#pragma once
#include <yaml-cpp/yaml.h>

#include "robot/resources/model_loader.hpp"
#include <filesystem>
#include <set>

#include "adapters/backends/mujoco.hpp"
#include "parol6/config.hpp"
#include "robot/boundary/joints.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/execution/drivers/host.hpp"

namespace application {
struct Deployment {
    robot::Model model;
    robot::resources::Positions initial;
    std::optional<robot::resources::Positions> standby;
    robot::execution::ControllerConfig controller;
    core::Period control_period;
    core::Period standby_duration;
    adapters::backends::MujocoConfig mujoco;
    parol6::config::SerialConfig serial;
    std::array<std::string, parol6::kJointCount> wire_joints;
    std::optional<robot::execution::drivers::HostTiming> host_timing;
};

// Unloaded input: overrides are applied before any URDF or scene is loaded.
struct RobotSpecification {
    std::string description, base, tip;
    std::vector<std::string> auxiliary_joints;
    std::vector<robot::resources::ControlLimits> control_limits;
    std::vector<double> initial;
    std::optional<std::vector<double>> standby;
};
struct ControllerSpecification {
    double period{0.01}, standby_duration{4}, speed_fraction{0.35};
    double acceleration_time{1.0 / 3}, jerk_time{0.1}, position_gain{8}, damping{0.04};
    std::optional<double> orientation_gain;
    std::optional<std::vector<double>> tracking_error;
};
struct DeploymentSpecification {
    std::vector<robot::resources::SensorDescription> external_sensors;
    RobotSpecification robot;
    ControllerSpecification controller;
    adapters::backends::MujocoConfig mujoco;
    parol6::config::SerialConfig serial;
    std::vector<std::string> wire_joints;
    std::optional<robot::execution::drivers::HostTiming> host_timing;
};
inline core::duration seconds(double value) {
    const auto positive = robot::boundary::PositiveNumber::read(value).value();
    if (positive >= std::chrono::duration<double>(core::duration::max()).count())
        throw std::invalid_argument("duration exceeds clock range");
    const auto duration = std::chrono::duration_cast<core::duration>(std::chrono::duration<double>{positive});
    if (duration == core::duration::zero()) throw std::invalid_argument("duration is below clock resolution");
    return duration;
}
inline Deployment loadDeployment(const DeploymentSpecification &spec) {
    const auto &r = spec.robot;
    const auto &c = spec.controller;
    auto model = robot::resources::ModelLoader::load(r.description, r.base, r.tip, r.auxiliary_joints, r.control_limits);
    auto initial = robot::boundary::readJoints<robot::resources::Positions>(model, r.initial);
    std::optional<robot::resources::Positions> standby;
    if (r.standby)
        standby = robot::boundary::readJoints<robot::resources::Positions>(model, *r.standby);
    using Positive = robot::boundary::PositiveNumber;
    std::optional<robot::execution::TrackingTolerance> tracking;
    if (c.tracking_error) {
        robot::boundary::Sequence source{*c.tracking_error};
        tracking = model.joints().map([&](auto) { return Positive::read(source.next()); });
        source.finish();
    }
    robot::execution::ControllerConfig config{
        model, {robot::boundary::UnitFraction::read(c.speed_fraction), Positive::read(c.acceleration_time), Positive::read(c.jerk_time),
                Positive::read(c.position_gain), Positive::read(c.damping),
                c.orientation_gain ? std::optional{Positive::read(*c.orientation_gain)} : std::nullopt, std::move(tracking)}};
    if (!config.velocity().allFinite() || !config.acceleration().allFinite() || !config.jerk().allFinite())
        throw std::invalid_argument("deployment execution limits overflow");
    const core::Period period{seconds(c.period)};
    if (spec.host_timing && spec.host_timing->maximum_interval.value() < period.value())
        throw std::invalid_argument("host maximum interval must cover its configured cadence");
    std::set<std::string> sensor_names;
    const auto registerSensors = [&](const auto &descriptions) {
        for (const auto &sensor : descriptions)
            if (sensor.name.empty() || !sensor_names.insert(sensor.name).second)
                throw std::invalid_argument("sensor names must be unique and nonempty");
    };
    registerSensors(spec.mujoco.sensors);
    registerSensors(spec.external_sensors);
    std::array<std::string, parol6::kJointCount> wire{};
    if (!spec.wire_joints.empty())
        wire = robot::boundary::readArray<std::string, parol6::kJointCount>(spec.wire_joints);
    return {model, std::move(initial), std::move(standby), std::move(config), period, core::Period{seconds(c.standby_duration)},
            spec.mujoco, spec.serial, std::move(wire), spec.host_timing};
}
inline DeploymentSpecification loadSpecification(const std::filesystem::path &path) {
    const auto root = YAML::LoadFile(path.string());
    const auto resolve = [&](const YAML::Node &entry) { return (path.parent_path() / entry.as<std::string>()).lexically_normal().string(); };
    DeploymentSpecification spec;
    auto &r = spec.robot;
    const auto robot = root["robot"];
    r.description = resolve(robot["description"]);
    r.base = robot["base"].as<std::string>();
    r.tip = robot["tip"].as<std::string>();
    r.auxiliary_joints = robot["auxiliary_joints"].as<std::vector<std::string>>(std::vector<std::string>{});
    r.initial = robot["initial"].as<std::vector<double>>();
    if (robot["standby"])
        r.standby = robot["standby"].as<std::vector<double>>();
    if (robot["control_limits"])
        for (const auto &entry : robot["control_limits"])
            r.control_limits.push_back({entry.first.as<std::string>(), entry.second["lower"].as<double>(), entry.second["upper"].as<double>(),
                                        entry.second["velocity"].as<double>()});
    auto &c = spec.controller;
    const auto controller = root["controller"];
    c.period = controller["period_ms"].as<double>() / 1000;
    c.standby_duration = controller["standby_duration_ms"].as<double>() / 1000;
    c.speed_fraction = controller["speed_fraction"].as<double>();
    c.acceleration_time = controller["acceleration_time"].as<double>();
    c.jerk_time = controller["jerk_time"].as<double>();
    c.position_gain = controller["position_gain"].as<double>(8);
    c.damping = controller["damping"].as<double>(0.04);
    if (controller["orientation_gain"])
        c.orientation_gain = controller["orientation_gain"].as<double>();
    if (controller["tracking_error"])
        c.tracking_error = controller["tracking_error"].as<std::vector<double>>();
    if (const auto mj = root["mujoco"]) {
        spec.mujoco.file = resolve(mj["model"]);
        spec.mujoco.base_frame = mj["base_frame"].as<std::string>("");
        spec.mujoco.tip_frame = mj["tip_frame"].as<std::string>("");
        if (mj["binary_actuator"]) {
            spec.mujoco.binary_actuator = mj["binary_actuator"].as<std::string>();
            spec.mujoco.open_position = mj["open_position"].as<double>();
            spec.mujoco.closed_position = mj["closed_position"].as<double>();
        }
    }
    const auto sensors = [](const YAML::Node &source, auto &destination) {
        if (!source) return;
        for (const auto &entry : source)
            destination.push_back(robot::resources::SensorDescription{entry["name"].as<std::string>(), entry["frame"].as<std::string>(),
                entry["calibration"].as<std::string>(""), entry["units"].as<std::vector<std::string>>(std::vector<std::string>{})});
    };
    sensors(root["sensors"], spec.mujoco.sensors);
    sensors(root["external_sensors"], spec.external_sensors);
    if (const auto serial = root["serial"]) {
        auto &s = spec.serial;
        if (serial["device"]) s.device = serial["device"].as<std::string>();
        if (serial["baud"]) s.baud = serial["baud"].as<int>();
        if (serial["startup_outputs"]) s.startup_outputs = serial["startup_outputs"].as<std::array<bool, 2>>();
        if (serial["first_response_timeout_seconds"]) s.first_response_timeout_seconds = serial["first_response_timeout_seconds"].as<double>();
        if (serial["transmission_timeout_seconds"]) s.transmission_timeout_seconds = serial["transmission_timeout_seconds"].as<double>();
        if (serial["telemetry_timeout_seconds"]) s.telemetry_timeout_seconds = serial["telemetry_timeout_seconds"].as<double>();
        if (serial["pneumatic_open_high"]) s.pneumatic_open_high = serial["pneumatic_open_high"].as<bool>();
        spec.wire_joints = serial["joints"].as<std::vector<std::string>>();
    }
    if (const auto timing = root["host_timing"]) {
        using Policy = robot::execution::drivers::OverrunPolicy;
        const auto policy = timing["overrun"].as<std::string>("fault");
        if (policy != "fault" && policy != "brake") throw std::invalid_argument("host_timing.overrun must be fault or brake");
        spec.host_timing = robot::execution::drivers::HostTiming{
            core::Period{std::chrono::milliseconds{timing["maximum_interval_ms"].as<int>()}}, policy == "fault" ? Policy::Fault : Policy::Brake};
    }
    return spec;
}
inline Deployment loadDeployment(const std::filesystem::path &path) { return loadDeployment(loadSpecification(path)); }
} // namespace application
