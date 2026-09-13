#include "eae/can_messages.hpp"

#include <algorithm>
#include <cmath>

namespace eae {

namespace {

// Temperatures are carried as a signed 16-bit value at 0.1 C per bit, giving
// a resolution finer than the sensor itself and a range far wider than the
// coolant will ever reach.
constexpr double kTempScale = 10.0;

void pack_int16(uint8_t* dest, int16_t value) {
    dest[0] = static_cast<uint8_t>(value & 0xFF);
    dest[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

int16_t unpack_int16(const uint8_t* src) {
    return static_cast<int16_t>(static_cast<uint16_t>(src[0]) |
                                (static_cast<uint16_t>(src[1]) << 8));
}

// Converts a temperature to raw counts, clamping to what an int16 can hold so
// that an out-of-range value wraps to a plausible-looking wrong number.
int16_t temp_to_raw(double celsius) {
    const double scaled = std::round(celsius * kTempScale);
    const double clamped = std::clamp(scaled, -32768.0, 32767.0);
    return static_cast<int16_t>(clamped);
}

// Confirms a frame is the expected message before its contents are trusted.
bool frame_matches(const CanFrame& frame, uint32_t id, uint8_t min_dlc) {
    return frame.id == id && frame.dlc >= min_dlc;
}

}  // namespace

// --------------------------------------------------------------------------

CanFrame encode(const CoolantStatus& status) {
    CanFrame frame;
    frame.id = can_id::kCoolantStatus;
    frame.dlc = 3;
    pack_int16(frame.data, temp_to_raw(status.temperature_c));
    frame.data[2] = status.sensor_valid ? 1 : 0;
    return frame;
}

std::optional<CoolantStatus> decode_coolant_status(const CanFrame& frame) {
    if (!frame_matches(frame, can_id::kCoolantStatus, 3)) {
        return std::nullopt;
    }
    CoolantStatus status;
    status.temperature_c = unpack_int16(frame.data) / kTempScale;
    status.sensor_valid = frame.data[2] != 0;
    return status;
}

// --------------------------------------------------------------------------

CanFrame encode(const ActuatorStatus& status) {
    CanFrame frame;
    frame.id = can_id::kActuatorStatus;
    frame.dlc = 2;
    frame.data[0] = status.pump_enabled ? 1 : 0;
    frame.data[1] = std::min<uint8_t>(status.fan_pwm_percent, 100);
    return frame;
}

std::optional<ActuatorStatus> decode_actuator_status(const CanFrame& frame) {
    if (!frame_matches(frame, can_id::kActuatorStatus, 2)) {
        return std::nullopt;
    }
    ActuatorStatus status;
    status.pump_enabled = frame.data[0] != 0;
    status.fan_pwm_percent = std::min<uint8_t>(frame.data[1], 100);
    return status;
}

// --------------------------------------------------------------------------

CanFrame encode(const SystemStatus& status) {
    CanFrame frame;
    frame.id = can_id::kSystemStatus;
    frame.dlc = 2;
    frame.data[0] = static_cast<uint8_t>(status.state);
    frame.data[1] = static_cast<uint8_t>(status.fault);
    return frame;
}

std::optional<SystemStatus> decode_system_status(const CanFrame& frame) {
    if (!frame_matches(frame, can_id::kSystemStatus, 2)) {
        return std::nullopt;
    }

    // Reject enum values this build does not know about rather than casting
    // an arbitrary byte into an enum and acting on it.
    if (frame.data[0] > static_cast<uint8_t>(State::Shutdown) ||
        frame.data[1] > static_cast<uint8_t>(FaultCode::Overtemperature)) {
        return std::nullopt;
    }

    SystemStatus status;
    status.state = static_cast<State>(frame.data[0]);
    status.fault = static_cast<FaultCode>(frame.data[1]);
    return status;
}

// --------------------------------------------------------------------------

CanFrame encode(const SetpointCommand& command) {
    CanFrame frame;
    frame.id = can_id::kSetpointCommand;
    frame.dlc = 2;
    pack_int16(frame.data, temp_to_raw(command.setpoint_c));
    return frame;
}

std::optional<SetpointCommand> decode_setpoint_command(const CanFrame& frame) {
    if (!frame_matches(frame, can_id::kSetpointCommand, 2)) {
        return std::nullopt;
    }

    const double value = unpack_int16(frame.data) / kTempScale;

    // A setpoint outside the sensible operating band is rejected outright. It
    // is safer to keep running on the previous value than to accept a command
    // that would either overcool or allow the coolant to run far too hot.
    if (value < kSetpointMinC || value > kSetpointMaxC) {
        return std::nullopt;
    }

    SetpointCommand command;
    command.setpoint_c = value;
    return command;
}

}  // namespace eae
