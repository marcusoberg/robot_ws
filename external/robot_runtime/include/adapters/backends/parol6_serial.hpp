#pragma once
#include <algorithm>

#include "parol6/serial_backend.hpp"
#include "robot/backend/concepts.hpp"

namespace adapters::backends {
// Six-axis wire format and calibration belong exclusively to this device adapter.
class Parol6SerialBackend {
public:
    Parol6SerialBackend(robot::Model model, const parol6::config::SerialConfig &config, std::array<std::string, parol6::kJointCount> wire_joints)
        : model_(std::move(model)), mapping_(mapping(model_, wire_joints)), wire_(config) {}
    [[nodiscard]] robot::backend::BackendCapabilities capabilities() const {
        return {"parol6_serial",
                "firmware_step_counter",
                robot::backend::InhibitBehavior::FirmwareMotionInhibit,
                true,
                false,
                core::Period{std::chrono::milliseconds(10)}};
    }
    [[nodiscard]] const robot::Model &model() const { return model_; }
    [[nodiscard]] robot::backend::BackendObservation read(core::HostClock::time_point now) {
        wire_.readAvailable(now);
        const auto observation = wire_.observation();
        if (!observation.robot.valid)
            return {};
        Eigen::VectorXd position(model_.dof()), velocity(model_.dof());
        for (std::size_t i = 0; i < mapping_.size(); ++i) {
            position[mapping_[i]] = observation.robot.q[i];
            velocity[mapping_[i]] = observation.robot.dq[i];
        }
        auto measurement = model_.joints().measure([&](auto i) { return position[i]; }, [&](auto i) { return velocity[i]; });
        robot::backend::RobotState state{std::move(measurement)};
        state.homed = observation.robot.homed;
        state.physical_estop = observation.robot.physical_estop;
        state.fault = observation.robot.fault;
        // Firmware adapter currently has no confirmed torque, gripper-position,
        // enabled-state or collision measurement. Leave them unavailable.
        state.stamp.sequence = observation.robot.sequence;
        state.stamp.received = wire_.receivedAt();
        const auto &status = observation.status;
        robot::backend::SensorSample raw{
            "parol6/device_status", "", "parol6:firmware-45a7502-layout", state.stamp,
            robot::backend::NumericSensor{{double(status.homed_bits), double(status.io_bits), double(status.temperature_bits),
                                           double(status.position_error_bits), double(status.timing_ticks), double(status.timeout_error),
                                           double(status.echoed_command), double(status.gripper_device_id), double(status.gripper_position),
                                           double(status.gripper_speed), double(status.gripper_current), double(status.gripper_status),
                                           double(status.gripper_object_detection), double(status.raw_tool_byte)},
                                          {}}};
        return {std::move(state), {}, {std::move(raw)}};
    }
    robot::backend::Delivery write(const robot::backend::ActuatorCommand &command) {
        parol6::JointSetpoint point;
        point.valid = true;
        point.sequence = command.sequence;
        point.enable = command.actuation == robot::backend::Actuation::Position;
        point.inhibit_motion = command.actuation == robot::backend::Actuation::Inhibit;
        if (point.enable)
            for (std::size_t i = 0; i < mapping_.size(); ++i) {
                point.q[i] = command.position.values()[mapping_[i]];
                point.dq[i] = command.velocity.values()[mapping_[i]];
            }
        if (command.gripper_open) {
            grip_.open = *command.gripper_open;
            ++grip_.sequence;
        }
        wire_.write(point, grip_);
        const auto status = wire_.transmission();
        robot::backend::TransportDelivery delivery{{}, status.pending, status.fault};
        if (status.transmitted)
            delivery.transmitted = robot::backend::TransmittedFrame{status.transmitted->request_sequence, status.transmitted->at, status.transmitted->command};
        return delivery;
    }

private:
    static std::array<Eigen::Index, parol6::kJointCount> mapping(const robot::Model &model, const std::array<std::string, parol6::kJointCount> &wire_joints) {
        std::array<Eigen::Index, parol6::kJointCount> mapping{};
        if (model.dof() != static_cast<Eigen::Index>(parol6::kJointCount))
            throw std::invalid_argument("PAROL6 serial requires six explicitly mapped joints");
        for (std::size_t i = 0; i < wire_joints.size(); ++i) {
            const auto &names = model.description().joints;
            auto found = std::find(names.begin(), names.end(), wire_joints[i]);
            if (found == names.end())
                throw std::invalid_argument("wire joint absent from model: " + wire_joints[i]);
            mapping[i] = std::distance(names.begin(), found);
            const double max_speed =
                parol6::protocol::stepsPerSecondToRadiansPerSecond(static_cast<std::int32_t>(parol6::config::kMaxSpeedStepsPerSecond[i]), i);
            if (model.description().units[mapping[i]] != "rad" || model.description().speed[mapping[i]] > max_speed + 1e-9)
                throw std::invalid_argument("PAROL6 model limits exceed calibrated device speed");
            (void)parol6::protocol::radiansToSteps(model.description().lower[mapping[i]], i);
            (void)parol6::protocol::radiansToSteps(model.description().upper[mapping[i]], i);
            for (std::size_t j = 0; j < i; ++j)
                if (mapping[j] == mapping[i])
                    throw std::invalid_argument("duplicate wire joint mapping");
        }
        return mapping;
    }
    robot::Model model_;
    std::array<Eigen::Index, parol6::kJointCount> mapping_{};
    parol6::Parol6SerialBackend wire_;
    parol6::GripperCommand grip_;
};
} // namespace adapters::backends
