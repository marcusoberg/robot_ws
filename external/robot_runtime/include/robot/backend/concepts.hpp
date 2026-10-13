#pragma once
#include <concepts>

#include "robot/backend/types.hpp"

namespace robot::backend {
// The adapter presents model-ordered joint values. It owns hardware mapping and
// reports missing measurements as unavailable, not fabricated zero readings.
template <class T>
concept Device = requires(T &device, const ActuatorCommand &command, core::HostClock::time_point now) {
    { device.model() } -> std::same_as<const Model &>;
    { device.capabilities() } -> std::same_as<BackendCapabilities>;
    { device.read(now) } -> std::same_as<BackendObservation>;
    { device.write(command) } -> std::same_as<Delivery>;
};
template <class T>
concept Stepped = Device<T> && requires(T &device, std::size_t count) {
    { device.time() } -> std::same_as<core::Time>;
    { device.physicsPeriod() } -> std::same_as<core::Period>;
    { device.advance(count) } -> std::same_as<void>;
};
} // namespace robot::backend
