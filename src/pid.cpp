#include "eae/pid.hpp"

#include <algorithm>

namespace eae {

Pid::Pid(const PidConfig& config) : config_(config) {}

void Pid::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_ = false;
}

double Pid::update(double setpoint, double measured, double dt) {
    // A non-positive timestep would divide by zero in the derivative term.
    if (dt <= 0.0) {
        return 0.0;
    }

    // Reverse action inverts the error, so a measurement above the setpoint
    // produces a positive error and therefore more cooling.
    const double error = config_.reverse_acting ? (measured - setpoint)
                                                : (setpoint - measured);

    // Proportional term.
    const double p_term = config_.kp * error;

    // Derivative term. Skipped on the very first call, because there is no
    // previous error to difference against and the resulting spike would be
    // meaningless.
    double d_term = 0.0;
    if (has_previous_) {
        d_term = config_.kd * (error - previous_error_) / dt;
    }

    // Provisionally integrate, then decide whether to keep it.
    const double candidate_integral = integral_ + error * dt;
    const double candidate_output =
        p_term + config_.ki * candidate_integral + d_term;

    // Anti-windup: only commit the new integral if the resulting output is
    // within limits, or if the error is pushing the output back into range.
    const bool saturated_high = candidate_output > config_.output_max;
    const bool saturated_low = candidate_output < config_.output_min;
    const bool unwinding = (saturated_high && error < 0.0) ||
                           (saturated_low && error > 0.0);

    if ((!saturated_high && !saturated_low) || unwinding) {
        integral_ = candidate_integral;
    }

    const double output = p_term + config_.ki * integral_ + d_term;

    previous_error_ = error;
    has_previous_ = true;

    return std::clamp(output, config_.output_min, config_.output_max);
}

}  // namespace eae
