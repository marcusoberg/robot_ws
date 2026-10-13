#pragma once
#include "robot/resources/model.hpp"
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>

namespace robot::resources {
// URDF and deployment constraints enter once here. Successful loading produces an
// immutable model and a single coordinate space used by every component.
class ModelLoader {
public:
    [[nodiscard]] static Model load(const std::filesystem::path &file, std::string base, std::string tip, std::span<const std::string> auxiliary = {},
                                    std::span<const ControlLimits> overrides = {});
};

} // namespace robot::resources
