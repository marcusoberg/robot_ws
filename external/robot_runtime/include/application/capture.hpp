#pragma once
#include "robot/execution/state.hpp"
#include <array>
#include <memory>

namespace application {
struct ImageSample {
    std::string name,frame,calibration;
    robot::backend::SampleStamp stamp;
    robot::backend::ImageSensor image;
    std::array<double,9> intrinsics{};
    std::array<double,16> world_from_camera{};
};
struct Capture {
    std::string runtime_instance;
    std::shared_ptr<const robot::execution::Snapshot> observation;
    std::vector<ImageSample> images;
    std::uint64_t sequence{};
};
} // namespace application
