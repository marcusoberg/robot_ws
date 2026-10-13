#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "parol6/types.hpp"

namespace parol6::protocol {

inline constexpr std::uint8_t kStartByte = 0xFF;
inline constexpr std::uint8_t kEnd0 = 0x01;
inline constexpr std::uint8_t kEnd1 = 0x02;

// Host -> firmware. LEN is every byte after the length byte, including CRC and END.
inline constexpr std::size_t kCommandLengthField = 52;
inline constexpr std::size_t kCommandFrameSize = 4 + kCommandLengthField;

// Firmware -> host. Current PAROL6 firmware sends LEN=56. The first 52 bytes match
// the Python status decoder; byte 52 is the firmware's extra tool byte, then
// a placeholder CRC and two END markers. Only this identified frame layout is accepted.
inline constexpr std::size_t kCurrentTelemetryLengthField = 56;
inline constexpr std::size_t kTelemetryPayloadSize = 53;

inline constexpr int kPneumaticOutputArrayIndex = 2;  // firmware OUT1 / Python io_port=1

enum class FirmwareCommand : std::uint8_t {
    Home = 100,
    Enable = 101,
    InhibitMotion = 102,
    Jog = 123,
    Move = 156,
    Teleport = 200,
    Idle = 255,
};

struct TelemetryDecoded {
    RobotState robot{};
    DeviceStatus status;
};

[[nodiscard]] bool bitMsbFirst(std::uint8_t value, int array_index);
[[nodiscard]] std::int32_t decodeSigned24(const std::uint8_t* p);
void encodeSigned24(std::uint8_t* p, std::int32_t value);
[[nodiscard]] std::int16_t decodeSigned16(const std::uint8_t* p);
[[nodiscard]] std::int32_t radiansToSteps(double radians, std::size_t joint);
[[nodiscard]] std::int32_t radiansPerSecondToStepsPerSecond(double radians_per_second,
                                                            std::size_t joint);
[[nodiscard]] double stepsToRadians(std::int32_t steps, std::size_t joint);
[[nodiscard]] double stepsPerSecondToRadiansPerSecond(std::int32_t steps_per_second,
                                                      std::size_t joint);

[[nodiscard]] std::array<std::uint8_t, kCommandFrameSize> encodeCommandFrame(
    const JointSetpoint& setpoint, FirmwareCommand command, const std::array<bool, 2>& outputs);

// `payload` is the 53 status bytes, not the framing bytes. This mirrors
// parol6.protocol.wire.unpack_rx_frame_into in the current official Python API.
[[nodiscard]] bool decodeTelemetryPayload(std::span<const std::uint8_t> payload,
                                          TelemetryDecoded& out);

class TelemetryStreamParser {
   public:
    TelemetryStreamParser();

    // Appends arbitrary serial chunks, tolerates fragmentation/noise, and returns
    // the number of valid telemetry frames consumed. `latest` receives the newest.
    std::size_t push(std::span<const std::uint8_t> bytes, TelemetryDecoded& latest);
    void clear();

   private:
    std::array<std::uint8_t, 260> buffer_{};
    std::size_t size_{};
};

}  // namespace parol6::protocol
