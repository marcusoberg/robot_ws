#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "parol6/config.hpp"
#include "parol6/serial_protocol.hpp"
#include "parol6/types.hpp"

namespace parol6 {

class Parol6SerialBackend final {
   public:
    explicit Parol6SerialBackend(const config::SerialConfig& cfg);
    ~Parol6SerialBackend();

    Parol6SerialBackend(const Parol6SerialBackend&) = delete;
    Parol6SerialBackend& operator=(const Parol6SerialBackend&) = delete;

    void readAvailable(std::chrono::steady_clock::time_point now);
    void write(const JointSetpoint& setpoint, const GripperCommand& gripper) {
        transmit(setpoint, gripper);
    }
    [[nodiscard]] TransmissionStatus transmission() const {
        return {transmitted_, pending_ ? std::optional{pending_->request_sequence} : std::nullopt,
                io_fault_};
    }
    [[nodiscard]] auto receivedAt() const { return last_rx_; }
    [[nodiscard]] BackendObservation observation() const { return observation_; }

   private:
    void configureSerial(int baud);
    void drainReceive();
    void applyTelemetry(const protocol::TelemetryDecoded& decoded);
    void progressTransmit(std::chrono::steady_clock::time_point now);
    void transmit(const JointSetpoint& setpoint, const GripperCommand& gripper);

    struct FileDescriptor {
        int value{-1};
        ~FileDescriptor();
        operator int() const noexcept { return value; }
    } fd_;
    bool io_fault_{false};
    std::string device_;
    protocol::TelemetryStreamParser parser_;
    BackendObservation observation_{};
    bool backend_enabled_{false};
    const std::array<bool, 2> startup_outputs_;
    std::chrono::steady_clock::time_point first_response_deadline_;
    struct PendingFrame {
        std::array<std::uint8_t, protocol::kCommandFrameSize> bytes;
        std::size_t offset{};
        std::uint64_t request_sequence{};
        protocol::FirmwareCommand command;
        std::chrono::steady_clock::time_point deadline;
    };
    std::optional<PendingFrame> pending_;
    std::optional<Transmission> transmitted_;
    std::chrono::steady_clock::duration transmission_timeout_;
    std::uint64_t state_sequence_{0};
    std::chrono::steady_clock::time_point last_rx_{};
    double telemetry_timeout_seconds_{0.10};
    bool pneumatic_open_high_{true};
};

}  // namespace parol6
