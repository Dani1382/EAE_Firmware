#include "eae/state_machine.hpp"

namespace eae {

std::string_view to_string(State state) {
    switch (state) {
        case State::Init:     return "INIT";
        case State::Idle:     return "IDLE";
        case State::Starting: return "STARTING";
        case State::Running:  return "RUNNING";
        case State::Fault:    return "FAULT";
        case State::Shutdown: return "SHUTDOWN";
    }
    return "UNKNOWN";
}

std::string_view to_string(FaultCode code) {
    switch (code) {
        case FaultCode::None:            return "NONE";
        case FaultCode::SensorInvalid:   return "SENSOR_INVALID";
        case FaultCode::LowCoolant:      return "LOW_COOLANT";
        case FaultCode::Overtemperature: return "OVERTEMPERATURE";
    }
    return "UNKNOWN";
}

StateMachine::StateMachine(const MachineConfig& config) : config_(config) {}

void StateMachine::transition_to(State next) {
    if (next != state_) {
        state_ = next;
        time_in_state_ = 0.0;
    }
}

FaultCode StateMachine::evaluate_faults(const MachineInputs& in) {
    // A sensor that cannot be read is checked first. Without a trustworthy
    // temperature there is no basis for evaluating overtemperature at all.
    if (!in.sensor_valid) {
        return FaultCode::SensorInvalid;
    }

    if (in.low_coolant) {
        return FaultCode::LowCoolant;
    }

    // Latched overtemperature with hysteresis: trips at the high threshold and
    // only releases once the coolant has come back down to the clear point.
    if (in.coolant_temp_c >= config_.overtemp_trip_c) {
        overtemp_latched_ = true;
    } else if (in.coolant_temp_c <= config_.overtemp_clear_c) {
        overtemp_latched_ = false;
    }

    return overtemp_latched_ ? FaultCode::Overtemperature : FaultCode::None;
}

MachineOutputs StateMachine::update(const MachineInputs& in, double dt) {
    time_in_state_ += dt;

    const FaultCode fault = evaluate_faults(in);

    switch (state_) {
        case State::Init:
            // One cycle to settle, then the machine is live.
            transition_to(State::Idle);
            break;

        case State::Idle:
            // Ignition off clears the overtemperature latch, so a new key
            // cycle never inherits a stale fault.
            overtemp_latched_ = false;
            if (in.ignition_on) {
                transition_to(State::Starting);
            }
            break;

        case State::Starting:
            if (!in.ignition_on) {
                transition_to(State::Shutdown);
            } else if (fault != FaultCode::None) {
                transition_to(State::Fault);
            } else if (time_in_state_ >= config_.prime_seconds) {
                transition_to(State::Running);
            }
            break;

        case State::Running:
            if (!in.ignition_on) {
                transition_to(State::Shutdown);
            } else if (fault != FaultCode::None) {
                transition_to(State::Fault);
            }
            break;

        case State::Fault:
            // Ignition off always wins, even with a fault active.
            if (!in.ignition_on) {
                transition_to(State::Shutdown);
            } else if (fault == FaultCode::None) {
                // The condition cleared. Return through Starting rather than
                // straight to Running so the pump re-primes and the PID is
                // re-entered from a known state.
                transition_to(State::Starting);
            }
            break;

        case State::Shutdown:
            if (in.ignition_on) {
                // Key turned back on mid-purge. Go straight back to priming.
                transition_to(State::Starting);
            } else if (time_in_state_ >= config_.purge_seconds) {
                transition_to(State::Idle);
            }
            break;
    }

    // Build the output commands from whichever state is now current.
    MachineOutputs out;
    out.state = state_;
    out.fault = (state_ == State::Fault) ? fault : FaultCode::None;

    switch (state_) {
        case State::Init:
        case State::Idle:
            out.pump_enabled = false;
            break;

        case State::Starting:
            // Pump on to circulate, but the PID stays out until Running.
            out.pump_enabled = true;
            break;

        case State::Running:
            out.pump_enabled = true;
            out.closed_loop_active = true;
            break;

        case State::Fault:
            // Fail safe: maximum cooling, PID bypassed. A production system
            // might derate or shut down instead, which is a vehicle safety
            // decision rather than a control one.
            out.pump_enabled = true;
            out.request_max_cooling = true;
            break;

        case State::Shutdown:
            // Pump continues to purge residual heat; the fan is not driven.
            out.pump_enabled = true;
            break;
    }

    return out;
}

}  // namespace eae
