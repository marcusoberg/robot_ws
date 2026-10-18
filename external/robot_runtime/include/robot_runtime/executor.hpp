#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_runtime {
using Time = std::int64_t;
using Positions = std::vector<double>;
enum class Mode { Disabled, Ready, Running, Stopping, Fault };

inline std::string mode_name(Mode mode) {
    switch (mode) {
    case Mode::Disabled:
        return "Disabled";
    case Mode::Ready:
        return "Ready";
    case Mode::Running:
        return "Running";
    case Mode::Stopping:
        return "Stopping";
    case Mode::Fault:
        return "Fault";
    }
    throw std::logic_error("unknown mode");
}

struct RobotSpec {
    std::vector<std::string> joints, units;
    Positions lower, upper;
    std::string command_mode{"position"}, backend;
    void validate() const {
        const auto n = joints.size();
        if (!n || units.size() != n || lower.size() != n || upper.size() != n || command_mode != "position" || backend.empty())
            throw std::invalid_argument("invalid position robot specification");
        std::set<std::string> names;
        for (std::size_t i = 0; i < n; ++i)
            if (joints[i].empty() || !names.insert(joints[i]).second || (units[i] != "rad" && units[i] != "m") || !std::isfinite(lower[i]) ||
                !std::isfinite(upper[i]) || lower[i] >= upper[i])
                throw std::invalid_argument("invalid joint specification");
    }
    void validate_positions(const Positions &q) const {
        if (q.size() != joints.size())
            throw std::invalid_argument("joint cardinality mismatch");
        for (std::size_t i = 0; i < q.size(); ++i)
            if (!std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i])
                throw std::invalid_argument("invalid goal for joint " + joints[i]);
    }
};

struct Measurement {
    Positions position, velocity;
    bool available{true}, fault{false};
};

struct JointReference {
    Positions position;
    std::uint64_t session{}, sequence{};
    Time issued_at{}, valid_until{};
};

struct StopProfile {
    double velocity_threshold{0.02};
    Time settled_duration{100'000'000}, timeout{5'000'000'000};
};
struct Status {
    Mode mode{Mode::Disabled};
    Measurement measured;
    Positions reference;
    std::uint64_t session{}, sequence{}, delivered_sequence{};
    Time time{};
    std::string source, reason;
};
class Backend {
public:
    virtual ~Backend() = default;
    virtual Measurement read() = 0;
    virtual void enable(const Positions &) = 0;
    virtual void write(const Positions &) = 0;
    virtual void hold(const Positions &) = 0;
    virtual void inhibit() = 0;
};
class PositionHold {
public:
    JointReference reference(const Positions &goal, std::uint64_t session, std::uint64_t sequence, Time now, Time expiry) const {
        return {goal, session, sequence, now, expiry};
    }
};
class Executor {
public:
    Executor(RobotSpec spec, Backend &backend, StopProfile stop = {}) : spec_(std::move(spec)), backend_(backend), stop_(stop) {
        spec_.validate();
        if (!std::isfinite(stop.velocity_threshold) || stop.velocity_threshold <= 0 || stop.settled_duration <= 0 || stop.timeout <= stop.settled_duration)
            throw std::invalid_argument("invalid stop profile");
    }
    const RobotSpec &spec() const { return spec_; }
    const Status &status() const { return status_; }
    void reset(Time now) {
        backend_.inhibit();
        ++generation_;
        reference_.reset();
        stopped_since_.reset();
        status_ = {};
        status_.time = now;
        refresh();
    }
    void enable(Time now) {
        if (status_.mode != Mode::Disabled)
            throw std::logic_error("enable requires Disabled; reset faults explicitly");
        status_.time = now;
        if (!refresh())
            throw std::runtime_error(status_.reason);
        try {
            backend_.enable(status_.measured.position);
        } catch (const std::exception &e) {
            fault(e.what());
            throw;
        }
        status_.reference = status_.measured.position;
        status_.mode = Mode::Ready;
        status_.reason.clear();
    }
    std::uint64_t activate(const std::string &source) {
        if (source.empty() || status_.mode != Mode::Ready || status_.session)
            throw std::logic_error("activation requires unowned Ready state");
        status_.source = source;
        status_.session = ++generation_;
        status_.sequence = 0;
        status_.delivered_sequence = 0;
        return status_.session;
    }
    void submit(const JointReference &value, Time now) {
        if ((status_.mode != Mode::Ready && status_.mode != Mode::Running) || !status_.session || value.session != status_.session)
            throw std::invalid_argument("reference has no active session");
        if (now < status_.time || !value.sequence || value.sequence <= status_.sequence || value.issued_at < 0 || value.issued_at > now ||
            value.valid_until <= now || value.issued_at > value.valid_until)
            throw std::invalid_argument("reference is stale, reordered, or future dated");
        spec_.validate_positions(value.position);
        reference_ = value;
        status_.sequence = value.sequence;
        status_.mode = Mode::Running;
        status_.reason.clear();
    }
    void stop(Time now, std::string reason = "paused") {
        if (status_.mode == Mode::Disabled || status_.mode == Mode::Fault || status_.mode == Mode::Stopping)
            return;
        status_.time = now;
        revoke();
        if (!refresh())
            return;
        status_.reference = status_.measured.position;
        try {
            backend_.hold(status_.reference);
        } catch (const std::exception &e) {
            fault(e.what());
            return;
        }
        status_.mode = Mode::Stopping;
        status_.reason = std::move(reason);
        stop_started_ = now;
        stopped_since_.reset();
    }
    void disable() {
        revoke();
        try {
            backend_.inhibit();
        } catch (const std::exception &e) {
            fault(e.what());
            return;
        }
        if (status_.mode != Mode::Fault)
            status_.mode = Mode::Disabled;
    }
    void tick(Time now) {
        if (now < status_.time)
            throw std::invalid_argument("execution clock moved backwards");
        status_.time = now;
        if (!refresh() || status_.mode == Mode::Disabled || status_.mode == Mode::Fault)
            return;
        if (status_.mode == Mode::Running && (!reference_ || now >= reference_->valid_until))
            stop(now, "reference_expired");
        if (status_.mode == Mode::Fault)
            return;
        if (status_.mode == Mode::Stopping) {
            const bool stationary = std::all_of(status_.measured.velocity.begin(), status_.measured.velocity.end(),
                                                [&](double v) { return std::abs(v) < stop_.velocity_threshold; });
            if (!stationary)
                stopped_since_.reset();
            else if (!stopped_since_)
                stopped_since_ = now;
            if (stopped_since_ && now - *stopped_since_ >= stop_.settled_duration)
                status_.mode = Mode::Ready;
            else if (now - stop_started_ >= stop_.timeout) {
                fault("stop_timeout");
                return;
            }
        }
        if (status_.mode == Mode::Running)
            status_.reference = reference_->position;
        try {
            backend_.write(status_.reference);
            if (status_.mode == Mode::Running)
                status_.delivered_sequence = reference_->sequence;
        } catch (const std::exception &e) {
            fault(e.what());
        }
    }
    void observe(Time now) {
        status_.time = now;
        refresh();
    }
    void fault(const std::string &reason) noexcept {
        revoke();
        status_.mode = Mode::Fault;
        status_.reason = reason;
        try {
            backend_.inhibit();
        } catch (...) {
        }
    }

private:
    void revoke() {
        reference_.reset();
        ++generation_;
        status_.session = 0;
        status_.source.clear();
        status_.delivered_sequence = 0;
    }
    bool refresh() {
        try {
            auto measured = backend_.read();
            const auto n = spec_.joints.size();
            if (!measured.available || measured.fault || measured.position.size() != n || measured.velocity.size() != n ||
                !std::all_of(measured.position.begin(), measured.position.end(), [](double v) { return std::isfinite(v); }) ||
                !std::all_of(measured.velocity.begin(), measured.velocity.end(), [](double v) { return std::isfinite(v); }))
                throw std::runtime_error("backend feedback unavailable or invalid");
            status_.measured = std::move(measured);
            return true;
        } catch (const std::exception &e) {
            fault(e.what());
            return false;
        }
    }
    RobotSpec spec_;
    Backend &backend_;
    StopProfile stop_;
    Status status_;
    std::optional<JointReference> reference_;
    static std::uint64_t new_identity() {
        std::random_device random;
        return (static_cast<std::uint64_t>(random())<<32) | random();
    }
    std::uint64_t generation_{new_identity()};
    Time stop_started_{};
    std::optional<Time> stopped_since_;
};
} // namespace robot_runtime
