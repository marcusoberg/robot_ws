#pragma once
#include <array>
#include <numbers>
#include <string>

#include "parol6/types.hpp"

namespace parol6::config {
// Firmware calibration, independent of the robot model/kinematic/controller layer.
inline constexpr JointVector kGearRatio{6.4, 20.0, 20.0 * (38.0 / 42.0), 4.0, 4.0, 10.0};
inline constexpr JointVector kMaxSpeedStepsPerSecond{15000, 25000, 32000, 10000, 10000, 27000};
inline constexpr double kRadiansPerMotorStep = 2.0 * std::numbers::pi / (32.0 * 200.0);
struct SerialConfig {
    std::string device{"/dev/ttyACM0"};
    int baud{3000000};
    // Required when opening this adapter: the first request necessarily drives both outputs.
    std::optional<std::array<bool, 2>> startup_outputs;
    double first_response_timeout_seconds{1.0};
    double telemetry_timeout_seconds{0.10};
    bool pneumatic_open_high{true};
    double transmission_timeout_seconds{0.05};
};
}  // namespace parol6::config
