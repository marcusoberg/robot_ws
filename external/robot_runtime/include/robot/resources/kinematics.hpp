#pragma once
#include "robot/resources/model.hpp"
#include <memory>

namespace robot::resources {
// Each calculation owner gets its own Pinocchio workspace. The model is shared
// immutably; neither model loading nor shape validation occurs during calculation.
class Workspace {
public:
    explicit Workspace(Model model);
    ~Workspace();
    Workspace(const Workspace &) = delete;
    Workspace &operator=(const Workspace &) = delete;
    [[nodiscard]] const Kinematics &calculate(const Positions &position);
    // All URDF link frames, expressed in the configured base. This workspace is
    // independent of the execution workspace and is owned by its visualization caller.
    [[nodiscard]] std::vector<std::pair<std::string, Eigen::Isometry3d>> frames(const Positions &position);

private:
    struct Storage;
    std::unique_ptr<Storage> workspace_;
};
} // namespace robot::resources
