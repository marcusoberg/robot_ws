#pragma once
#include <string>
#include <vector>
namespace robot::resources {
// Resource metadata loaded once. Calibration identifies an external calibration asset.
struct SensorDescription {
    std::string name, frame, calibration;
    std::vector<std::string> units;
};
} // namespace robot::resources
