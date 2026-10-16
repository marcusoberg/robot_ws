#pragma once
#include <optional>
#include <memory>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "robot/communication/authority.hpp"
#include "robot/execution/trajectory.hpp"

namespace robot::execution {
// A prepared rolling path and the identity of the schedule it replaces.
struct GripperPoint { core::duration offset; bool open; };
class ScheduledMotion {
public:
    static constexpr auto maximum_horizon = std::chrono::seconds(5);

    [[nodiscard]] core::Time start() const { return start_; }
    [[nodiscard]] core::Time end() const { return start_ + path_.duration(); }
    [[nodiscard]] std::uint64_t epoch() const { return epoch_; }
    [[nodiscard]] std::optional<communication::Id> replaces() const { return replaces_; }
    [[nodiscard]] const JointPath &path() const { return path_; }

    [[nodiscard]] bool hasGripper() const { return gripper_ && !gripper_->empty(); }
    [[nodiscard]] std::optional<bool> gripper(core::Time time) const {
        if (!hasGripper() || time < start_ || time > end()) return {};
        const auto elapsed = time - start_;
        const auto next = std::upper_bound(gripper_->begin(), gripper_->end(), elapsed,
            [](core::duration t, const GripperPoint &p) { return t < p.offset; });
        if (next == gripper_->begin()) return {};
        return std::prev(next)->open;
    }
private:
    friend class Planner;

    ScheduledMotion(core::Time start, std::uint64_t epoch, std::optional<communication::Id> replaces, JointPath path, std::vector<GripperPoint> gripper = {})
        : start_(start), epoch_(epoch), replaces_(replaces), path_(std::move(path)) {
        if (gripper.size() > 256) throw std::invalid_argument("too many scheduled gripper samples");
        std::optional<core::duration> previous;
        for (const auto &point : gripper) {
            if (point.offset < core::duration{} || (previous && point.offset <= *previous) || point.offset > path_.duration())
                throw std::invalid_argument("gripper timestamps must increase within the motion horizon");
            previous = point.offset;
        }
        if (!gripper.empty()) gripper_ = std::make_shared<const std::vector<GripperPoint>>(std::move(gripper));
    }

    core::Time start_;
    std::uint64_t epoch_;
    std::optional<communication::Id> replaces_;
    JointPath path_;
    std::shared_ptr<const std::vector<GripperPoint>> gripper_;
};

} // namespace robot::execution
