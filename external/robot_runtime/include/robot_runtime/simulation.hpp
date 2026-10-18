#pragma once
#include "executor.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <chrono>
#include <memory>
#include <map>
#include <mutex>
#include <thread>

namespace robot_runtime {
struct Contact { std::string geom1,geom2; std::vector<std::string> ancestors1,ancestors2; double normal_force{}; };
struct MujocoBackend final : Backend {
    MujocoBackend(const RobotSpec&, const std::string& scene, const Positions& initial);
    Measurement read() override;
    void enable(const Positions&) override;
    void write(const Positions&) override;
    void hold(const Positions&) override;
    void inhibit() override;
    void reset();
    void advance(std::size_t steps);
    Positions integration_state() const;
    std::map<std::string, Positions> body_state() const;
    std::vector<Contact> contacts() const;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
private:
    RobotSpec spec_; Positions initial_;
    std::vector<int> joints_, actuators_;
    std::vector<std::array<double,mjNGAIN>> gains_;
    std::vector<std::array<double,mjNBIAS>> biases_;
};
struct Trace { Time begin{}, end{}; Positions reference, measured; std::uint64_t sequence{}; };
struct Snapshot {
    Status status;
    Time time{};
    std::uint64_t episode{}, observation{};
    Positions integration;
    std::vector<Trace> trace;
    std::map<std::string, Positions> bodies;
    std::vector<Contact> contacts;
    double wall_lag_seconds{};
    Time host_acquired_ns{};
};
class Simulation {
public:
    Simulation(RobotSpec spec, const std::string& scene, Positions initial, Time control_period=10'000'000, Time action_period=50'000'000, StopProfile stop={}, Time physics_period=1'000'000);
    ~Simulation();
    Snapshot reset();
    void enable();
    std::uint64_t activate(const std::string& source);
    void submit(const Positions& goal, std::uint64_t session, std::uint64_t sequence, Time valid_for=300'000'000);
    Snapshot step(const Positions& goal, std::uint64_t session, std::uint64_t sequence);
    Snapshot advance(std::size_t ticks);
    Snapshot observe();
    Snapshot latest_boundary();
    void schedule(const Positions&, std::uint64_t session, std::uint64_t sequence, Time application_time, bool interval_only=false);
    void pause(const std::string& reason="paused");
    void disable();
    void start_paced();
    void stop_paced();
    bool paced();
private:
    Time now() const;
    void tick();
    Snapshot snapshot();
    MujocoBackend backend_; Executor executor_; PositionHold controller_;
    Time control_period_, action_period_, time_{};
    std::size_t substeps_;
    std::uint64_t episode_{}, observation_{};
    std::vector<Trace> trace_;
    std::mutex mutex_;
    std::jthread runner_;
    bool paced_{};
    std::chrono::steady_clock::time_point origin_;
    Time origin_time_{};
    double lag_{};
    std::optional<JointReference> pending_;
    Time pending_time_{};
    bool pending_interval_only_{};
    std::optional<Time> interval_end_;
    Snapshot boundary_;
    bool needs_reset_{};
};
}
