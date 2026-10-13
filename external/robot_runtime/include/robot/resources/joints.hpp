#pragma once
#include <Eigen/Geometry>
#include <concepts>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robot {
class Model;
}

namespace robot::resources {

struct ControlLimits {
    std::string joint;
    double lower, upper, velocity;
};

struct Description {
    std::string name, base, tip, identity;
    std::vector<std::string> joints, frames, units, auxiliary_joints;
    Eigen::VectorXd lower, upper, speed;
    std::size_t configuration_size{}, tangent_size{};
};

struct PositionTag {};
struct VelocityTag {};
struct AccelerationTag {};
struct EffortTag {};

template <class T>
concept QuantityTag = std::same_as<T, PositionTag> || std::same_as<T, VelocityTag> || std::same_as<T, AccelerationTag> || std::same_as<T, EffortTag>;
template <class Producer>
concept CoordinateProducer = std::invocable<Producer &, Eigen::Index> && std::convertible_to<std::invoke_result_t<Producer &, Eigen::Index>, double>;

class JointSpace;

template <class T> class JointMap {
public:
    [[nodiscard]] const T &operator[](Eigen::Index index) const { return entries_[index]; }

private:
    friend class JointSpace;

    explicit JointMap(std::vector<T> entries) : entries_(std::move(entries)) {}

    std::vector<T> entries_;
};

// Fixed shape for the lifetime of this value. No resize, default constructor or
// conversion from an arbitrary Eigen vector. Only its owning JointSpace creates it.
template <QuantityTag Tag> class JointValues {
public:
    using quantity = Tag;

    template <CoordinateProducer Producer> void assign(Producer &&coordinate) {
        for (Eigen::Index i = 0; i < values_.size(); ++i)
            values_[i] = coordinate(i);
    }

    [[nodiscard]] const Eigen::VectorXd &values() const noexcept { return values_; }

private:
    friend class JointSpace;

    JointValues(std::shared_ptr<const Description> owner, Eigen::VectorXd values) : owner_(std::move(owner)), values_(std::move(values)) {}

    std::shared_ptr<const Description> owner_;
    Eigen::VectorXd values_;
};

using Positions = JointValues<PositionTag>;
using Velocities = JointValues<VelocityTag>;
using Accelerations = JointValues<AccelerationTag>;
using Efforts = JointValues<EffortTag>;
template <class T>
concept JointQuantity = requires { typename T::quantity; } && std::same_as<T, JointValues<typename T::quantity>>;

class Measurement {
public:
    [[nodiscard]] const Eigen::VectorXd &position() const noexcept { return position_.values(); }
    [[nodiscard]] const Eigen::VectorXd &velocity() const noexcept { return velocity_.values(); }
    [[nodiscard]] const Positions &positions() const noexcept { return position_; }
    [[nodiscard]] const Velocities &velocities() const noexcept { return velocity_; }
    [[nodiscard]] const std::optional<Efforts> &effort() const noexcept { return effort_; }

private:
    friend class JointSpace;

    Measurement(Positions position, Velocities velocity, std::optional<Efforts> effort)
        : position_(std::move(position)), velocity_(std::move(velocity)), effort_(std::move(effort)) {}

    Positions position_;
    Velocities velocity_;
    std::optional<Efforts> effort_;
};

// A joint layout is created once by ModelLoader. Producers write one coordinate
// for each model index; no producer can supply a differently sized container.
class JointSpace {
public:
    [[nodiscard]] Eigen::Index extent() const noexcept { return owner_->lower.size(); }

    template <class Producer>
        requires std::invocable<Producer &, Eigen::Index>
    [[nodiscard]] auto map(Producer &&entry) const {
        using T = std::remove_cvref_t<std::invoke_result_t<Producer &, Eigen::Index>>;
        std::vector<T> entries;
        entries.reserve(extent());
        for (Eigen::Index i = 0; i < extent(); ++i)
            entries.push_back(entry(i));
        return JointMap<T>{std::move(entries)};
    }

    template <JointQuantity T, CoordinateProducer Producer> [[nodiscard]] T generate(Producer &&coordinate) const {
        Eigen::VectorXd values(extent());
        for (Eigen::Index i = 0; i < extent(); ++i)
            values[i] = coordinate(i);
        return T{owner_, std::move(values)};
    }

    template <JointQuantity T> [[nodiscard]] T zero() const {
        return generate<T>([](Eigen::Index) { return 0.0; });
    }

    template <CoordinateProducer Position, CoordinateProducer Velocity> [[nodiscard]] Measurement measure(Position &&position, Velocity &&velocity) const {
        return {generate<Positions>(position), generate<Velocities>(velocity), {}};
    }

    template <CoordinateProducer Position, CoordinateProducer Velocity, CoordinateProducer Effort>
    [[nodiscard]] Measurement measure(Position &&position, Velocity &&velocity, Effort &&effort) const {
        return {generate<Positions>(position), generate<Velocities>(velocity), generate<Efforts>(effort)};
    }

private:
    friend class robot::Model;

    explicit JointSpace(std::shared_ptr<const Description> owner) : owner_(std::move(owner)) {}

    std::shared_ptr<const Description> owner_;
};

struct Implementation;
class ModelLoader;
class Workspace;

struct Kinematics {
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
};

} // namespace robot::resources
