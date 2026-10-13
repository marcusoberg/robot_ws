#include "robot/resources/kinematics.hpp"
#include "robot/resources/model_loader.hpp"

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <boost/uuid/detail/sha1.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>
#include <unordered_set>

namespace robot::resources {
struct Implementation {
    pinocchio::Model model;
    pinocchio::FrameIndex base{}, tip{};
    struct Coordinate {
        int configuration, tangent, configuration_size;
    };
    std::vector<Coordinate> coordinates;
};

Model ModelLoader::load(const std::filesystem::path &file, std::string base, std::string tip, std::span<const std::string> auxiliary,
                        std::span<const ControlLimits> overrides) {
    std::ifstream input(file);
    if (!input)
        throw std::invalid_argument("cannot read URDF: " + file.string());
    const std::string xml{std::istreambuf_iterator<char>{input}, {}};
    const auto tree = urdf::parseURDF(xml);
    if (!tree)
        throw std::invalid_argument("URDF parser rejected description");
    if (!tree->getLink(base) || !tree->getLink(tip))
        throw std::invalid_argument("base and tip must name URDF links");
    auto data = std::make_shared<Description>();
    data->name = tree->getName();
    data->base = std::move(base);
    data->tip = std::move(tip);
    data->auxiliary_joints.assign(auxiliary.begin(), auxiliary.end());
    auto implementation = std::make_shared<Implementation>();
    auto &native = implementation->model;
    // Mimic joints remain dependent coordinates, not independent actuators.
    pinocchio::urdf::buildModel(tree, native, false, true);
    implementation->base = native.getFrameId(data->base, pinocchio::BODY);
    implementation->tip = native.getFrameId(data->tip, pinocchio::BODY);
    data->configuration_size = native.nq;
    data->tangent_size = native.nv;
    for (const auto &[name, _] : tree->links_)
        data->frames.push_back(name);

    std::vector<std::string> chain;
    std::unordered_set<std::string> visited;
    auto link = tree->getLink(data->tip);
    while (link->name != data->base) {
        if (!visited.insert(link->name).second || !link->parent_joint)
            throw std::invalid_argument("tip must descend from base");
        if (link->parent_joint->type != urdf::Joint::FIXED)
            chain.push_back(link->parent_joint->name);
        link = tree->getLink(link->parent_joint->parent_link_name);
    }
    std::reverse(chain.begin(), chain.end());
    for (const auto &name : chain) {
        auto joint = tree->getJoint(name);
        std::unordered_set<std::string> dependencies;
        while (joint->mimic) {
            if (!dependencies.insert(joint->name).second)
                throw std::invalid_argument("cyclic mimic relationship");
            joint = tree->getJoint(joint->mimic->joint_name);
            if (!joint)
                throw std::invalid_argument("unknown mimic source");
        }
        if (std::ranges::find(data->joints, joint->name) == data->joints.end())
            data->joints.push_back(joint->name);
    }
    for (const auto &name : auxiliary) {
        auto joint = tree->getJoint(name);
        if (!joint || joint->mimic || joint->type == urdf::Joint::FIXED || std::ranges::find(data->joints, name) != data->joints.end())
            throw std::invalid_argument("auxiliary must name an independent joint: " + name);
        data->joints.push_back(name);
    }
    if (data->joints.empty())
        throw std::invalid_argument("empty actuated control group");
    data->lower.resize(data->joints.size());
    data->upper.resize(data->joints.size());
    data->speed.resize(data->joints.size());
    std::unordered_set<std::string> overridden;
    for (const auto &limit : overrides)
        if (!overridden.insert(limit.joint).second || std::ranges::find(data->joints, limit.joint) == data->joints.end())
            throw std::invalid_argument("unknown/duplicate joint limit override: " + limit.joint);
    for (std::size_t i = 0; i < data->joints.size(); ++i) {
        const auto &name = data->joints[i];
        const auto joint = tree->getJoint(name);
        const auto id = native.getJointId(name);
        const auto &coordinate = native.joints[id];
        const bool continuous = joint->type == urdf::Joint::CONTINUOUS;
        if ((joint->type != urdf::Joint::REVOLUTE && !continuous && joint->type != urdf::Joint::PRISMATIC) || coordinate.nv() != 1 ||
            coordinate.nq() != (continuous ? 2 : 1))
            throw std::invalid_argument("control group supports independent scalar joints: " + name);
        implementation->coordinates.push_back({coordinate.idx_q(), coordinate.idx_v(), coordinate.nq()});
        const auto limit = std::ranges::find(overrides, name, &ControlLimits::joint);
        if (limit != overrides.end()) {
            data->lower[i] = limit->lower;
            data->upper[i] = limit->upper;
            data->speed[i] = limit->velocity;
            if (joint->limits && !continuous && (limit->lower < joint->limits->lower || limit->upper > joint->limits->upper))
                throw std::invalid_argument("deployment bounds exceed URDF limits: " + name);
            if (joint->limits && joint->limits->velocity > 0 && limit->velocity > joint->limits->velocity)
                throw std::invalid_argument("deployment velocity exceeds URDF limit: " + name);
        } else {
            if (continuous || !joint->limits)
                throw std::invalid_argument("explicit deployment control_limits required: " + name);
            data->lower[i] = joint->limits->lower;
            data->upper[i] = joint->limits->upper;
            data->speed[i] = joint->limits->velocity;
        }
        if (!std::isfinite(data->lower[i]) || !std::isfinite(data->upper[i]) || !std::isfinite(data->speed[i]) || data->lower[i] >= data->upper[i] ||
            data->speed[i] <= 0)
            throw std::invalid_argument("invalid finite control limits: " + name);
        data->units.push_back(joint->type == urdf::Joint::PRISMATIC ? "m" : "rad");
    }
    // Dependent joint travel/speed constraints also constrain its driving coordinate.
    for (const auto &[name, joint] : tree->joints_) {
        if (!joint->mimic)
            continue;
        std::shared_ptr<const urdf::Joint> source = joint;
        double multiplier = 1.0, offset = 0.0;
        std::unordered_set<std::string> dependencies;
        while (source->mimic) {
            if (!dependencies.insert(source->name).second)
                throw std::invalid_argument("cyclic mimic relationship: " + name);
            offset += multiplier * source->mimic->offset;
            multiplier *= source->mimic->multiplier;
            source = tree->getJoint(source->mimic->joint_name);
            if (!source)
                throw std::invalid_argument("unknown mimic source: " + name);
        }
        if (!std::isfinite(multiplier) || !std::isfinite(offset))
            throw std::invalid_argument("nonfinite mimic relationship: " + name);
        const auto found = std::ranges::find(data->joints, source->name);
        if (found == data->joints.end() || !joint->limits)
            continue;
        const auto index = std::distance(data->joints.begin(), found);
        if (multiplier == 0.0) {
            if (offset < joint->limits->lower || offset > joint->limits->upper)
                throw std::invalid_argument("constant mimic position violates its limits: " + name);
            continue;
        }
        if (joint->type != urdf::Joint::CONTINUOUS) {
            const auto lo = (joint->limits->lower - offset) / multiplier;
            const auto hi = (joint->limits->upper - offset) / multiplier;
            data->lower[index] = std::max(data->lower[index], std::min(lo, hi));
            data->upper[index] = std::min(data->upper[index], std::max(lo, hi));
        }
        data->speed[index] = std::min(data->speed[index], joint->limits->velocity / std::abs(multiplier));
        if (data->lower[index] >= data->upper[index] || data->speed[index] <= 0)
            throw std::invalid_argument("mimic constraints leave no feasible control range: " + name);
    }
    // A reproducible description/control-group fingerprint, not authentication.
    std::ostringstream selection;
    selection << xml << '\0' << data->base << '\0' << data->tip << std::setprecision(17);
    for (std::size_t i = 0; i < data->joints.size(); ++i)
        selection << '\0' << data->joints[i] << ':' << data->lower[i] << ':' << data->upper[i] << ':' << data->speed[i];
    boost::uuids::detail::sha1 digest;
    const auto bytes = selection.str();
    digest.process_bytes(bytes.data(), bytes.size());
    boost::uuids::detail::sha1::digest_type hash;
    digest.get_digest(hash);
    std::ostringstream fingerprint;
    fingerprint << "sha1:" << std::hex << std::setfill('0');
    for (const auto word : hash)
        fingerprint << std::setw(2 * sizeof(word)) << static_cast<unsigned>(word);
    data->identity = fingerprint.str();
    return Model{std::move(data), std::move(implementation)};
}

struct Workspace::Storage {
    Model owner;
    pinocchio::Data data;
    Eigen::VectorXd configuration;
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_jacobian, tip_jacobian;
    Kinematics result;
    explicit Storage(Model model)
        : owner(std::move(model)), data(owner.implementation_->model), configuration(pinocchio::neutral(owner.implementation_->model)),
          base_jacobian(6, owner.implementation_->model.nv), tip_jacobian(6, owner.implementation_->model.nv) {
        result.jacobian.resize(6, owner.dof());
    }
};
Workspace::Workspace(Model model) : workspace_(std::make_unique<Storage>(std::move(model))) {}
Workspace::~Workspace() = default;
const Kinematics &Workspace::calculate(const Positions &coordinates) {
    const auto &position = coordinates.values();
    auto &work = *workspace_;
    const auto &implementation = *work.owner.implementation_;
    const auto &native = implementation.model;
    for (std::size_t i = 0; i < implementation.coordinates.size(); ++i) {
        const auto &c = implementation.coordinates[i];
        if (c.configuration_size == 2) {
            work.configuration[c.configuration] = std::cos(position[i]);
            work.configuration[c.configuration + 1] = std::sin(position[i]);
        } else
            work.configuration[c.configuration] = position[i];
    }
    pinocchio::computeJointJacobians(native, work.data, work.configuration);
    pinocchio::updateFramePlacements(native, work.data);
    work.base_jacobian.setZero();
    work.tip_jacobian.setZero();
    pinocchio::getFrameJacobian(native, work.data, implementation.base, pinocchio::LOCAL_WORLD_ALIGNED, work.base_jacobian);
    pinocchio::getFrameJacobian(native, work.data, implementation.tip, pinocchio::LOCAL_WORLD_ALIGNED, work.tip_jacobian);
    const auto &base = work.data.oMf[implementation.base];
    const auto &tip = work.data.oMf[implementation.tip];
    const auto relative = base.actInv(tip);
    work.result.pose.matrix() = relative.toHomogeneousMatrix();
    const Eigen::Matrix3d rotation = base.rotation().transpose();
    const Eigen::Vector3d displacement = tip.translation() - base.translation();
    for (std::size_t i = 0; i < implementation.coordinates.size(); ++i) {
        const auto column = implementation.coordinates[i].tangent;
        work.result.jacobian.col(i).head<3>() = rotation * (work.tip_jacobian.col(column).head<3>() - work.base_jacobian.col(column).head<3>() +
                                                            displacement.cross(work.base_jacobian.col(column).tail<3>()));
        work.result.jacobian.col(i).tail<3>() = rotation * (work.tip_jacobian.col(column).tail<3>() - work.base_jacobian.col(column).tail<3>());
    }
    return work.result;
}
std::vector<std::pair<std::string, Eigen::Isometry3d>> Workspace::frames(const Positions &position) {
    (void)calculate(position);
    const auto &work = *workspace_;
    const auto &impl = *work.owner.implementation_;
    const auto &base = work.data.oMf[impl.base];
    std::vector<std::pair<std::string, Eigen::Isometry3d>> result;
    result.reserve(work.owner.description().frames.size());
    for (const auto &name : work.owner.description().frames) {
        const auto id = impl.model.getFrameId(name, pinocchio::BODY);
        Eigen::Isometry3d transform;
        transform.matrix() = base.actInv(work.data.oMf[id]).toHomogeneousMatrix();
        result.emplace_back(name, transform);
    }
    return result;
}
} // namespace robot::resources
