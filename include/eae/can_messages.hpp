#pragma once

#include "eae/can_bus.hpp"
#include "eae/state_machine.hpp"

#include <optional>

namespace eae {

/**
 * Application message set for the cooling loop.
 *
 * Three status frames travel from the PLC to the PV450 display, and one
 * command frame travels back from the display, whose keypad allows the
 * operator to adjust the target temperature.
 *
 * Signals are packed little-endian, which is the convention used throughout
 * this message set.
 */
namespace can_id {

constexpr uint32_t kCoolantStatus = 0x100;  // PLC -> display
constexpr uint32_t kActuatorStatus = 0x101; // PLC -> display
constexpr uint32_t kSystemStatus = 0x102;   // PLC -> display
constexpr uint32_t kSetpointCommand = 0x200;// display -> PLC

}  // namespace can_id

// ---------------------------------------------------------------------------
// 0x100 Coolant status
//
//   bytes 0-1 : coolant temperature, int16, 0.1 C per bit
//   byte  2   : sensor valid flag, 0 or 1
// ---------------------------------------------------------------------------

struct CoolantStatus {
    double temperature_c = 0.0;
    bool sensor_valid = false;
};

CanFrame encode(const CoolantStatus& status);
std::optional<CoolantStatus> decode_coolant_status(const CanFrame& frame);

// ---------------------------------------------------------------------------
// 0x101 Actuator status
//
//   byte 0 : pump enabled, 0 or 1
//   byte 1 : fan PWM command, 0 to 100 percent
// ---------------------------------------------------------------------------

struct ActuatorStatus {
    bool pump_enabled = false;
    uint8_t fan_pwm_percent = 0;
};

CanFrame encode(const ActuatorStatus& status);
std::optional<ActuatorStatus> decode_actuator_status(const CanFrame& frame);

// ---------------------------------------------------------------------------
// 0x102 System status
//
//   byte 0 : controller state
//   byte 1 : active fault code
//
// Sending the fault code rather than a single fault bit lets the display show
// which condition tripped instead of a generic warning lamp.
// ---------------------------------------------------------------------------

struct SystemStatus {
    State state = State::Init;
    FaultCode fault = FaultCode::None;
};

CanFrame encode(const SystemStatus& status);
std::optional<SystemStatus> decode_system_status(const CanFrame& frame);

// ---------------------------------------------------------------------------
// 0x200 Setpoint command
//
//   bytes 0-1 : requested setpoint, int16, 0.1 C per bit
//
// Received rather than transmitted. The value is range-checked on arrival,
// because a frame from another node is untrusted input: a corrupted or
// mis-scaled setpoint must not be allowed to drive the control loop.
// ---------------------------------------------------------------------------

struct SetpointCommand {
    double setpoint_c = 0.0;
};

constexpr double kSetpointMinC = 20.0;
constexpr double kSetpointMaxC = 80.0;

CanFrame encode(const SetpointCommand& command);
std::optional<SetpointCommand> decode_setpoint_command(const CanFrame& frame);

}  // namespace eae
