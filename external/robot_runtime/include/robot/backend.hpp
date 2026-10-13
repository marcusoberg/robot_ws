#pragma once
#include "robot/backend/concepts.hpp"
#include "robot/resources/sensors.hpp"
#include <memory>
namespace robot {
// Device ownership only. Physics advancement belongs to a capability-specific driver.
class Backend final {
public:
    template <backend::Device Adapter> explicit Backend(std::unique_ptr<Adapter> adapter) : device_(std::make_unique<Device<Adapter>>(std::move(adapter))) {}
    Backend(Backend &&) = default;
    Backend &operator=(Backend &&) = default;
    const Model &model() const { return device_->model(); }
    backend::BackendCapabilities capabilities() const { return device_->capabilities(); }
    const std::vector<resources::SensorDescription> &sensorDescriptions() const { return device_->sensorDescriptions(); }
    backend::BackendObservation read(core::HostClock::time_point now) { return device_->read(now); }
    backend::Delivery write(const backend::ActuatorCommand &command) { return device_->write(command); }

private:
    struct Interface {
        virtual ~Interface() = default;
        virtual const Model &model() const = 0;
        virtual backend::BackendCapabilities capabilities() const = 0;
        virtual const std::vector<resources::SensorDescription> &sensorDescriptions() const = 0;
        virtual backend::BackendObservation read(core::HostClock::time_point) = 0;
        virtual backend::Delivery write(const backend::ActuatorCommand &) = 0;
    };
    template <backend::Device Adapter> struct Device final : Interface {
        explicit Device(std::unique_ptr<Adapter> value) : adapter(std::move(value)) {}
        const Model &model() const override { return adapter->model(); }
        backend::BackendCapabilities capabilities() const override { return adapter->capabilities(); }
        const std::vector<resources::SensorDescription> &sensorDescriptions() const override {
            if constexpr (requires(const Adapter &device) { device.sensorDescriptions(); })
                return adapter->sensorDescriptions();
            else {
                static const std::vector<resources::SensorDescription> empty;
                return empty;
            }
        }
        backend::BackendObservation read(core::HostClock::time_point now) override { return adapter->read(now); }
        backend::Delivery write(const backend::ActuatorCommand &command) override { return adapter->write(command); }
        std::unique_ptr<Adapter> adapter;
    };
    std::unique_ptr<Interface> device_;
};
} // namespace robot
