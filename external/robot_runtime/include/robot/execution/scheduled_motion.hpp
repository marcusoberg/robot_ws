#pragma once
#include <optional>

#include "robot/communication/authority.hpp"
#include "robot/execution/trajectory.hpp"

namespace robot::execution {
// A prepared rolling path and the identity of the schedule it replaces.
class ScheduledMotion {
public:
    static constexpr auto maximum_horizon = std::chrono::seconds(5);

    [[nodiscard]] core::Time start() const { return start_; }
    [[nodiscard]] core::Time end() const { return start_ + path_.duration(); }
    [[nodiscard]] std::uint64_t epoch() const { return epoch_; }
    [[nodiscard]] std::optional<communication::Id> replaces() const { return replaces_; }
    [[nodiscard]] const JointPath &path() const { return path_; }

private:
    friend class Planner;

    ScheduledMotion(core::Time start, std::uint64_t epoch, std::optional<communication::Id> replaces, JointPath path)
        : start_(start), epoch_(epoch), replaces_(replaces), path_(std::move(path)) {}

    core::Time start_;
    std::uint64_t epoch_;
    std::optional<communication::Id> replaces_;
    JointPath path_;
};

} // namespace robot::execution
