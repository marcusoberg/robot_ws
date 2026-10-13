#include <mujoco/mujoco.h>

#include <cmath>
#include <filesystem>
#include <mutex>
#include <stdexcept>

#include "adapters/backends/mujoco.hpp"
#include "robot/resources/kinematics.hpp"

namespace adapters::backends {
void MujocoBackend::DeleteModel::operator()(mjModel *model) const { mj_deleteModel(model); }
void MujocoBackend::DeleteData::operator()(mjData *data) const { mj_deleteData(data); }
MujocoBackend::~MujocoBackend() = default;
MujocoBackend::MujocoBackend(robot::Model model, const MujocoConfig &config, const robot::resources::Positions &initial)
    : model_(std::move(model)), initial_(initial), open_(config.open_position), closed_(config.closed_position) {
#if mjVERSION_HEADER >= 230
    // Release/wheel distributions place their plugins beside libmujoco.
    // In MuJoCo 3.6 these also provide the standard STL/OBJ decoders.
    // Registrations belong to MuJoCo's process-wide registry, not a model instance.
    static std::once_flag plugins;
    std::call_once(plugins, [] {
        // An embedding application (including the Python wheel) may already
        // have populated MuJoCo's process-wide registry from its own SDK path.
        if (mjp_pluginCount() == 0 && std::filesystem::is_directory(ROBOT_MUJOCO_PLUGIN_DIRECTORY))
            mj_loadAllPluginLibraries(ROBOT_MUJOCO_PLUGIN_DIRECTORY, nullptr);
    });
#endif
    char error[1024]{};
    mj_model_.reset(mj_loadXML(config.file.c_str(), nullptr, error, sizeof(error)));
    if (!mj_model_)
        throw std::runtime_error(std::string("MuJoCo: ") + error);
    data_.reset(mj_makeData(mj_model_.get()));
    if (!data_)
        throw std::runtime_error("MuJoCo data allocation failed");
    auto *m = mj_model_.get();
    auto *d = data_.get();
    for (int sensor = 0; sensor < m->nsensor; ++sensor) {
        const char *name = mj_id2name(m, mjOBJ_SENSOR, sensor);
        if (name && std::string_view{name}.starts_with("mujoco/"))
            throw std::invalid_argument("MuJoCo sensor namespace 'mujoco/' is reserved for scene coordinates");
    }
    period_ = core::Period{core::duration{std::llround(m->opt.timestep * 1e9)}};
    if (std::abs(std::chrono::duration<double>(period_.value()).count() - m->opt.timestep) > 1e-12)
        throw std::invalid_argument("MuJoCo timestep must be representable in nanoseconds");
    for (Eigen::Index i = 0; i < model_.dof(); ++i) {
        const auto &name = model_.description().joints[i];
        const int joint = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        if (joint < 0 || (m->jnt_type[joint] != mjJNT_HINGE && m->jnt_type[joint] != mjJNT_SLIDE))
            throw std::invalid_argument("MuJoCo missing scalar joint: " + name);
        const bool angular = model_.description().units[i] == "rad";
        if (angular != (m->jnt_type[joint] == mjJNT_HINGE))
            throw std::invalid_argument("MuJoCo/URDF joint units disagree: " + name);
        const double range_tolerance = angular ? 1e-5 : 1e-6; // Asset export rounding, in SI units.
        if (m->jnt_limited[joint] && (model_.description().lower[i] < m->jnt_range[2 * joint] - range_tolerance ||
                                      model_.description().upper[i] > m->jnt_range[2 * joint + 1] + range_tolerance))
            throw std::invalid_argument("MuJoCo joint range cannot cover control limits: " + name);
        int actuator = -1;
        for (int a = 0; a < m->nu; ++a)
            if (m->actuator_trntype[a] == mjTRN_JOINT && m->actuator_trnid[2 * a] == joint) {
                if (actuator >= 0)
                    throw std::invalid_argument("multiple actuators require explicit mapping: " + name);
                actuator = a;
            }
        if (actuator < 0 || m->actuator_gaintype[actuator] != mjGAIN_FIXED || m->actuator_biastype[actuator] != mjBIAS_AFFINE ||
            m->actuator_biasprm[actuator * mjNBIAS + 1] >= 0)
            throw std::invalid_argument("joint needs a position actuator: " + name);
        if (std::abs(m->actuator_gear[6 * actuator] - 1.0) > 1e-9)
            throw std::invalid_argument("position actuator mapping requires unit gear: " + name);
        if (m->actuator_ctrllimited[actuator] && (model_.description().lower[i] < m->actuator_ctrlrange[2 * actuator] - range_tolerance ||
                                                  model_.description().upper[i] > m->actuator_ctrlrange[2 * actuator + 1] + range_tolerance))
            throw std::invalid_argument("MuJoCo actuator range cannot cover control limits: " + name);
        joints_.push_back(joint);
        actuators_.push_back(actuator);
        d->qpos[m->jnt_qposadr[joint]] = initial.values()[i];
        d->ctrl[actuator] = initial.values()[i];
    }
    gain_.assign(m->actuator_gainprm, m->actuator_gainprm + m->nu * mjNGAIN);
    bias_.assign(m->actuator_biasprm, m->actuator_biasprm + m->nu * mjNBIAS);
    if (!config.binary_actuator.empty()) {
        binary_actuator_ = mj_name2id(m, mjOBJ_ACTUATOR, config.binary_actuator.c_str());
        if (binary_actuator_ < 0)
            throw std::invalid_argument("missing configured binary actuator");
        d->ctrl[binary_actuator_] = open_;
    }
    mj_forward(m, d);
    const auto frame = [&](const std::string &name) -> Eigen::Isometry3d {
        const int body = mj_name2id(m, mjOBJ_BODY, name.c_str());
        const int site = mj_name2id(m, mjOBJ_SITE, name.c_str());
        if ((body < 0) == (site < 0))
            throw std::invalid_argument("MuJoCo frame must name one body or site: " + name);
        const auto *position = body >= 0 ? d->xpos + 3 * body : d->site_xpos + 3 * site;
        const auto *rotation = body >= 0 ? d->xmat + 9 * body : d->site_xmat + 9 * site;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Map<const Eigen::Vector3d>(position);
        pose.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation);
        return pose;
    };
    const auto base = frame(config.base_frame.empty() ? model_.description().base : config.base_frame);
    const auto tip = frame(config.tip_frame.empty() ? model_.description().tip : config.tip_frame);
    const Eigen::Isometry3d actual = base.inverse() * tip;
    robot::resources::Workspace kinematics(model_);
    const auto &expected = kinematics.calculate(initial).pose;
    if ((actual.translation() - expected.translation()).norm() > 1e-4 ||
        std::abs(Eigen::AngleAxisd(actual.linear() * expected.linear().transpose()).angle()) > 1e-3)
        throw std::invalid_argument("MuJoCo/URDF initial tool frames disagree; check deployment mapping");
    for (int i = 0; i < m->nsensor; ++i) {
        const char *name = mj_id2name(m, mjOBJ_SENSOR, i);
        const char *attached = mj_id2name(m, m->sensor_objtype[i], m->sensor_objid[i]);
        sensors_.push_back({{name ? name : "sensor_" + std::to_string(i), attached ? attached : "", {}, {}}, SensorRange{m->sensor_adr[i], m->sensor_dim[i]}});
    }
    for (int joint = 0; joint < m->njnt; ++joint) {
        const char *named = mj_id2name(m, mjOBJ_JOINT, joint);
        const std::string name = named ? named : "@" + std::to_string(joint);
        const int begin = m->jnt_qposadr[joint], end = joint + 1 < m->njnt ? m->jnt_qposadr[joint + 1] : m->nq;
        sensors_.push_back({{"mujoco/joint_position/" + name, name, "mujoco:qpos", {}}, JointRange{begin, end - begin}});
    }
    for (int body = 0; body < m->nbody; ++body)
        if (m->body_mocapid[body] >= 0) {
            const char *named = mj_id2name(m, mjOBJ_BODY, body);
            const std::string name = named ? named : "@" + std::to_string(body);
            sensors_.push_back({{"mujoco/mocap/" + name, name, "mujoco:xyz-wxyz", {}}, MocapRange{m->body_mocapid[body]}});
        }
    for (const auto &configured : config.sensors) {
        const auto found = std::ranges::find(sensors_, configured.name, [](const auto &sensor) -> const std::string & { return sensor.description.name; });
        if (found == sensors_.end())
            throw std::invalid_argument("configured MuJoCo sensor missing: " + configured.name);
        found->description = configured;
    }
    for (const auto &binding : sensors_)
        sensor_descriptions_.push_back(binding.description);
    sensor_descriptions_.push_back({"mujoco/integration_state", "", "mujoco:integration-v1", {}});
    // Idle preserves the configured initial actuator hold. Runtime authority remains disarmed.
    enabled_ = true;
}
robot::backend::Delivery MujocoBackend::write(const robot::backend::ActuatorCommand &command) {
    auto *m = mj_model_.get();
    auto *d = data_.get();
    if (command.actuation == robot::backend::Actuation::Position || command.actuation == robot::backend::Actuation::Inhibit) {
        enabled_ = command.actuation == robot::backend::Actuation::Position;
        for (std::size_t i = 0; i < actuators_.size(); ++i) {
            const auto a = actuators_[i];
            for (int j = 0; j < mjNGAIN; ++j)
                m->actuator_gainprm[a * mjNGAIN + j] = enabled_ ? gain_[a * mjNGAIN + j] : 0;
            for (int j = 0; j < mjNBIAS; ++j)
                m->actuator_biasprm[a * mjNBIAS + j] = enabled_ ? bias_[a * mjNBIAS + j] : 0;
            if (enabled_)
                d->ctrl[a] = command.position.values()[static_cast<Eigen::Index>(i)];
        }
    }
    if (command.gripper_open && binary_actuator_ >= 0)
        d->ctrl[binary_actuator_] = *command.gripper_open ? open_ : closed_;
    return robot::backend::SimulatorDelivery{command.sequence};
}
void MujocoBackend::reset() {
    auto *m = mj_model_.get();
    auto *d = data_.get();
    mj_resetData(m, d);
    std::copy(gain_.begin(), gain_.end(), m->actuator_gainprm);
    std::copy(bias_.begin(), bias_.end(), m->actuator_biasprm);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        d->qpos[m->jnt_qposadr[joints_[i]]] = initial_.values()[i];
        d->ctrl[actuators_[i]] = initial_.values()[i];
    }
    if (binary_actuator_ >= 0)
        d->ctrl[binary_actuator_] = open_;
    enabled_ = true;
    ++sequence_;
    mj_forward(m, d);
}

core::Time MujocoBackend::time() const { return core::Time{core::duration{std::llround(data_->time * 1e9)}}; }

void MujocoBackend::advance(std::size_t steps) {
    for (std::size_t i = 0; i < steps; ++i)
        mj_step(mj_model_.get(), data_.get());
    mj_forward(mj_model_.get(), data_.get());
    ++sequence_;
}
robot::backend::BackendObservation MujocoBackend::read(core::HostClock::time_point now) {
    const auto *m = mj_model_.get();
    const auto *d = data_.get();
    auto measurement =
        model_.joints().measure([&](auto i) { return d->qpos[m->jnt_qposadr[joints_[i]]]; }, [&](auto i) { return d->qvel[m->jnt_dofadr[joints_[i]]]; },
                                [&](auto i) { return d->qfrc_actuator[m->jnt_dofadr[joints_[i]]]; });
    robot::backend::RobotState state{std::move(measurement)};
    state.homed = true;
    state.enabled = enabled_;
    state.stamp.sequence = sequence_;
    state.stamp.received = now;
    state.stamp.source_time_ns = std::llround(d->time * 1e9);
    state.stamp.logical_time = time();
    state.stamp.source_clock = "simulation";
    robot::backend::BackendObservation result{std::move(state), d->ncon > 0, {}};
    robot::backend::NumericSensor integration;
    integration.values.resize(mj_stateSize(m, mjSTATE_INTEGRATION));
    mj_getState(m, d, integration.values.data(), mjSTATE_INTEGRATION);
    result.sensors.push_back({"mujoco/integration_state", "", "mujoco:integration-v1", result.state->stamp, std::move(integration)});
    for (const auto &sensor : sensors_) {
        robot::backend::NumericSensor values;
        std::visit(
            [&](const auto &range) {
                using T = std::remove_cvref_t<decltype(range)>;
                if constexpr (std::same_as<T, MocapRange>) {
                    values.values.assign(d->mocap_pos + 3 * range.index, d->mocap_pos + 3 * range.index + 3);
                    values.values.insert(values.values.end(), d->mocap_quat + 4 * range.index, d->mocap_quat + 4 * range.index + 4);
                } else {
                    const double *data;
                    if constexpr (std::same_as<T, SensorRange>)
                        data = d->sensordata;
                    else
                        data = d->qpos;
                    values.values.assign(data + range.address, data + range.address + range.count);
                }
            },
            sensor.range);
        values.units = sensor.description.units;
        result.sensors.push_back({sensor.description.name, sensor.description.frame, sensor.description.calibration, result.state->stamp, std::move(values)});
    }
    return result;
}
} // namespace adapters::backends
