#pragma once
#include <chrono>
#include <concepts>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace core {
using HostClock = std::chrono::steady_clock;
using duration = std::chrono::nanoseconds;
using ms = std::chrono::milliseconds;
struct TimelineClock {
    using duration = core::duration;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<TimelineClock, duration>;
    static constexpr bool is_steady = false;
};
using Time = TimelineClock::time_point;
class Period {
public:
    explicit constexpr Period(duration value) : value_(value) {
        if (value <= duration::zero())
            throw std::invalid_argument("period must be positive");
    }
    [[nodiscard]] constexpr duration value() const noexcept { return value_; }

private:
    duration value_;
};
// The caller supplies an interval. Graph evaluation does not progress this timeline.
struct Context {
    Time begin, end;
    std::uint64_t seq, epoch;
    [[nodiscard]] duration elapsed() const noexcept { return end - begin; }
    template <std::floating_point T = double> [[nodiscard]] T getDt() const noexcept { return std::chrono::duration<T>(elapsed()).count(); }
};
template <class T>
concept SignalData = std::copyable<T>;
template <class Out, class L, class R>
concept LeftMultipliesTo = requires(const L &l, const R &r) {
    { l * r } -> std::convertible_to<Out>;
};
template <class Out, class L, class R>
concept RightMultipliesTo = requires(const L &l, const R &r) {
    { r * l } -> std::convertible_to<Out>;
};
template <class Out, class L, class R>
concept MultipliesTo = LeftMultipliesTo<Out, L, R> || RightMultipliesTo<Out, L, R>;
template <class T, class... Ts> inline constexpr std::size_t type_count = (0 + ... + std::size_t(std::same_as<T, Ts>));
template <class... Ts>
concept UniqueTypes = ((type_count<Ts, Ts...> == 1) && ...);
template <class... Ts>
    requires UniqueTypes<Ts...>
struct Inputs {};
template <class... Ts>
    requires UniqueTypes<Ts...>
struct Outputs {};
class IBlock;
class IComponent;
class IInPort;
} // namespace core
