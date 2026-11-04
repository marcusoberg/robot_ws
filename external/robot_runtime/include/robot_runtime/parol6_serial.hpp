#pragma once
#include "robot_runtime/executor.hpp"
#include <memory>

namespace robot_runtime {
// Stock PAROL6 reports software step counters, not encoder measurements.
struct Parol6Telemetry {
    std::string mode{"Connecting"}, reason, command_error;
    Positions position, velocity, reference, transmitted_position, transmitted_velocity;
    std::vector<int> steps, speed_steps, payload;
    std::uint64_t frames{};
    double age_seconds{};
    double cycle_seconds{}, goal_interval_seconds{};
    // Steady-clock times; wall time is captured from the same feedback estimate.
    std::int64_t sample_time_ns{}, sample_wall_time_ns{}, receive_time_ns{}, transmit_time_ns{};
    std::int64_t write_complete_time_ns{}, evaluation_time_ns{}, reference_time_ns{};
    double round_trip_seconds{}, timing_uncertainty_seconds{}, motion_elapsed_seconds{};
    std::uint64_t motion_id{}, reference_frame{};
    std::uint64_t transmitted_motion_id{}, reference_motion_id{};
    bool reference_valid{}, motion_complete{};
    bool connected{}, estop{true}, estop_verified{}, homed_this_session{};
    int homed_bits{}, io_bits{}, temperature_bits{}, position_error_bits{};
    int timing_ticks{}, timeout_value{}, command_echo{};
    int gripper_id{}, gripper_position{}, gripper_speed{}, gripper_current{}, gripper_status{},
        object_detection{};
};

// Owns the port and a 100 Hz polling thread independently of Python/ROS/IK.
// Faults latch for this connection. No automatic reconnect or motion restart.
class Parol6Serial {
  public:
    explicit Parol6Serial(const std::string &port, int baud = 3'000'000);
    ~Parol6Serial();
    Parol6Serial(const Parol6Serial &) = delete;
    Parol6Serial &operator=(const Parol6Serial &) = delete;
    Parol6Telemetry observe() const;
    std::vector<Parol6Telemetry> drain_samples();
    bool home();      // requires a fresh disabled-state E-stop press/release
    bool enable();    // reuse live controller homing flags and counters
    bool reset();     // clear software faults without reopening the serial connection
    void keepalive(); // renews homing/idle holding leases, never a motion target
    bool start();     // requires a valid controller position and enabled holding
    // Empty velocities retain position-only behavior; otherwise six rad/s values.
    bool submit(const Positions &six_joint_radians, const Positions &velocities = {});
    // Plans are evaluated by the serial clock. Heartbeats never restart a plan.
    bool move_joint(const Positions &target, double duration, std::uint64_t motion_id);
    bool follow_path(const Positions &times, const Positions &flattened_joints, std::uint64_t motion_id);
    bool live_target(const Positions &target, double period, std::uint64_t motion_id);
    bool heartbeat(std::uint64_t motion_id);
    void pause(); // zero velocity while retaining motor holding torque after homing
    void stop();  // terminal for this run; repeated command 102
    void close(); // bounded best-effort disable before closing
    static Positions standby();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robot_runtime
