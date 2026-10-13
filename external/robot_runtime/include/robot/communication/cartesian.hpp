#pragma once
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include <string>

namespace robot::communication {
// Base-frame Cartesian values. External adapters establish a normalized rotation
// before construction; numerical producers use the same fixed-size interfaces.
class Pose {
public:
    struct AuxiliaryPosition { Eigen::Index index; std::string name; double position; };
    explicit Pose(Eigen::Isometry3d value, std::vector<AuxiliaryPosition> auxiliary = {})
        : value_(std::move(value)), auxiliary_(std::move(auxiliary)) {}
    [[nodiscard]] const auto &auxiliary() const noexcept { return auxiliary_; }
    [[nodiscard]] const Eigen::Isometry3d &value() const { return value_; }

private:
    Eigen::Isometry3d value_;
    std::vector<AuxiliaryPosition> auxiliary_;
};
class Twist {
public:
    explicit Twist(Eigen::Matrix<double, 6, 1> value) : value_(std::move(value)) {}
    [[nodiscard]] const Eigen::Matrix<double, 6, 1> &value() const { return value_; }

private:
    Eigen::Matrix<double, 6, 1> value_;
};
} // namespace robot::communication
