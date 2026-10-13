#pragma once
#include <compare>
#include <cstdint>
#include <variant>
#include <optional>

#include "core/types.hpp"

namespace robot {
class Communication;
}
namespace robot::communication {
// Authority identities and operator requests are independent of motion payloads.
struct SourceId {
    std::uint64_t value{};
    auto operator<=>(const SourceId &) const = default;
};

struct Session {
    SourceId source;
    std::uint64_t generation{};
    auto operator<=>(const Session &) const = default;
};

struct Id {
    std::uint64_t value{};
    auto operator<=>(const Id &) const = default;
};
struct HostExpiry {
    core::HostClock::time_point at;
};

struct LogicalExpiry {
    core::Time at;
    std::uint64_t epoch;
};

struct AuthorityRenewal {
    Session session;
    std::uint64_t request_id;
    HostExpiry expiry;
};
// Only the receive boundary can establish a continuous host-time renewal chain.
class RenewalWindow {
public:
    Session session() const { return session_; }
    core::HostClock::time_point since() const { return since_; }
    HostExpiry through() const { return through_; }

private:
    friend class robot::Communication;
    RenewalWindow(Session session, core::HostClock::time_point since, HostExpiry through) : session_(session), since_(since), through_(through) {}
    Session session_;
    core::HostClock::time_point since_;
    HostExpiry through_;
};
using Expiry = std::variant<HostExpiry, LogicalExpiry>;
inline bool expired(const Expiry &expiry, const core::Context &ctx, core::HostClock::time_point now) {
    return std::visit(
        [&](const auto &deadline) {
            if constexpr (std::same_as<std::decay_t<decltype(deadline)>, HostExpiry>)
                return now >= deadline.at;
            else
                return ctx.epoch != deadline.epoch || ctx.end >= deadline.at;
        },
        expiry);
}
enum class OperatorAction { Enable, Disable, Stop, Hold, Standby, OpenGripper, CloseGripper, Acquire, Release, Takeover, Pause };

struct OperatorRequest {
    OperatorAction action;
    SourceId source{};
    std::uint64_t request_id{};
    std::optional<Session> session_guard;
};
struct Reference {
    Session session;
    Id id;
    auto operator<=>(const Reference &) const = default;
};
enum class OperatorDisposition { Rejected, Accepted, Completed, Canceled };

enum class OperatorReason { None, InvalidState, NoAuthority, NotHomed, Unavailable, Interrupted };

struct OperatorResult {
    std::uint64_t sequence{};
    OperatorRequest request;
    OperatorDisposition disposition{OperatorDisposition::Rejected};
    OperatorReason reason{OperatorReason::None};
    std::optional<Session> granted_session;
};
} // namespace robot::communication
