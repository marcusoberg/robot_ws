#pragma once
#include "robot/resources/joints.hpp"
#include <memory>

namespace robot {

// Immutable, validated robot description. ModelLoader is its only construction boundary.
class Model {
public:
    [[nodiscard]] const resources::Description &description() const noexcept { return *data_; }
    [[nodiscard]] const resources::JointSpace &joints() const noexcept { return joints_; }
    [[nodiscard]] Eigen::Index dof() const noexcept { return joints_.extent(); }

private:
    friend class resources::ModelLoader;
    friend class resources::Workspace;

    Model(std::shared_ptr<const resources::Description> data, std::shared_ptr<const resources::Implementation> implementation)
        : data_(std::move(data)), implementation_(std::move(implementation)), joints_(data_) {}

    std::shared_ptr<const resources::Description> data_;
    std::shared_ptr<const resources::Implementation> implementation_;
    resources::JointSpace joints_;
};

} // namespace robot
