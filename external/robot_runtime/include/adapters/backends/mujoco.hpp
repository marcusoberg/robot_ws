#pragma once
#include <memory>
#include <string>
#include <vector>

#include "robot/backend/concepts.hpp"
#include "robot/resources/sensors.hpp"

typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;
namespace adapters::backends {
struct MujocoConfig {
    std::string file;
    // Body/site names; empty uses the corresponding URDF frame name.
    std::string base_frame, tip_frame;
    // Optional deployment mapping for a binary tool; absent means no binary tool.
    std::string binary_actuator;
    double open_position{}, closed_position{};
    std::vector<robot::resources::SensorDescription> sensors;
};
class MujocoBackend {
public:
    MujocoBackend(robot::Model model, const MujocoConfig &config, const robot::resources::Positions &initial);
    ~MujocoBackend();
    MujocoBackend(const MujocoBackend &) = delete;
    MujocoBackend &operator=(const MujocoBackend &) = delete;
    [[nodiscard]] const robot::Model &model() const { return model_; }
    [[nodiscard]] core::Period physicsPeriod() const { return period_; }
    [[nodiscard]] robot::backend::BackendObservation read(core::HostClock::time_point now);
    [[nodiscard]] robot::backend::BackendCapabilities capabilities() const {
        return {"mujoco", "simulation_ground_truth", robot::backend::InhibitBehavior::ActuatorForcesOff, binary_actuator_ >= 0, false, {}};
    }
    robot::backend::Delivery write(const robot::backend::ActuatorCommand &command);
    const std::vector<robot::resources::SensorDescription> &sensorDescriptions() const { return sensor_descriptions_; }
    void reset();
    void advance(std::size_t steps);
    [[nodiscard]] core::Time time() const;

private:
    struct DeleteModel {
        void operator()(mjModel *) const;
    };
    struct DeleteData {
        void operator()(mjData *) const;
    };
    robot::Model model_;
    robot::resources::Positions initial_;
    std::unique_ptr<mjModel, DeleteModel> mj_model_;
    std::unique_ptr<mjData, DeleteData> data_;
    core::Period period_{std::chrono::milliseconds(1)};
    struct SensorRange {
        int address, count;
    };
    struct JointRange {
        int address, count;
    };
    struct MocapRange {
        int index;
    };
    struct SensorBinding {
        robot::resources::SensorDescription description;
        std::variant<SensorRange, JointRange, MocapRange> range;
    };
    std::vector<SensorBinding> sensors_;
    std::vector<robot::resources::SensorDescription> sensor_descriptions_;
    std::vector<int> joints_, actuators_;
    std::vector<double> gain_, bias_;
    int binary_actuator_{-1};
    double open_{}, closed_{};
    bool enabled_{false};
    std::uint64_t sequence_{1};
};
} // namespace adapters::backends
