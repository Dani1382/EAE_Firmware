#pragma once

#include <string_view>

namespace eae {

/**
 * Operating states of the cooling controller.
 *
 * The controller is always in exactly one of these. Every transition is
 * driven by the inputs sampled during update(), so the state at any moment
 * is a pure function of the input history.
 */
enum class State {
    Init,      // Power-on. Nothing has been sampled yet.
    Idle,      // Ignition off. Pump and fan are commanded off.
    Starting,  // Ignition on. Pump priming before closed-loop control begins.
    Running,   // Normal closed-loop cooling.
    Fault,     // A safety condition is active. Maximum cooling is commanded.
    Shutdown   // Ignition has gone off. Pump runs on briefly to purge heat.
};

/** Human-readable name for a state, for logging and CAN reporting. */
std::string_view to_string(State state);

/**
 * The reason the controller entered Fault. Reported over CAN so the display
 * can show which condition tripped rather than a generic fault light.
 */
enum class FaultCode {
    None,
    SensorInvalid,   // Coolant temperature could not be measured.
    LowCoolant,      // Level switch reports low coolant.
    Overtemperature  // Coolant exceeded the trip threshold.
};

std::string_view to_string(FaultCode code);

/** Inputs sampled once per control cycle. */
struct MachineInputs {
    bool ignition_on = false;
    bool sensor_valid = false;
    bool low_coolant = false;
    double coolant_temp_c = 0.0;
};

/** Commands and status produced by one control cycle. */
struct MachineOutputs {
    State state = State::Init;
    FaultCode fault = FaultCode::None;
    bool pump_enabled = false;
    bool request_max_cooling = false;  // Fault response: bypass the PID.
    bool closed_loop_active = false;   // True only when the PID should run.
};

/** Tunable timing and thresholds. */
struct MachineConfig {
    // How long the pump primes before closed-loop control begins. Circulating
    // coolant before trusting the temperature reading avoids acting on a
    // stagnant pocket of fluid sitting at the sensor.
    double prime_seconds = 2.0;

    // How long the pump runs after ignition off, to move residual heat out of
    // the inverter and DC-DC rather than letting it soak.
    double purge_seconds = 5.0;

    // Overtemperature trip and clear points. The gap provides hysteresis so a
    // temperature hovering at the threshold does not chatter the fault.
    double overtemp_trip_c = 65.0;
    double overtemp_clear_c = 60.0;
};

/**
 * Cooling loop state machine.
 *
 * This is the same control intent as the Section 7 PLC logic, restructured
 * so that mode is explicit rather than implied by a chain of if statements.
 * The practical gain is that Starting and Shutdown become expressible: the
 * PLC version had no way to say "pump running, but do not trust the sensor
 * yet".
 */
class StateMachine {
public:
    explicit StateMachine(const MachineConfig& config = {});

    /**
     * Advance the machine by one cycle.
     *
     * @param in  Inputs sampled this cycle.
     * @param dt  Seconds since the previous call.
     */
    MachineOutputs update(const MachineInputs& in, double dt);

    State state() const { return state_; }

private:
    // Evaluates the safety conditions and updates the latched overtemperature
    // flag. Returns the active fault, or None.
    FaultCode evaluate_faults(const MachineInputs& in);

    MachineConfig config_;
    State state_ = State::Init;
    double time_in_state_ = 0.0;
    bool overtemp_latched_ = false;

    void transition_to(State next);
};

}  // namespace eae
