#pragma once

namespace eae {

/**
 * Gains and limits for a PID controller.
 */
struct PidConfig {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;

    // The controller output is clamped to this range. For the cooling loop
    // this is fan PWM, so 0 to 100 percent.
    double output_min = 0.0;
    double output_max = 100.0;
};

/**
 * A PID controller with output clamping and integral anti-windup.
 *
 * Anti-windup matters here. If the fan is already at 100 percent and the
 * coolant is still too hot, an unguarded integral term keeps accumulating.
 * When the temperature finally drops, that stored error takes a long time to
 * unwind and the controller overshoots badly. This implementation stops
 * accumulating once the output has saturated.
 */
class Pid {
public:
    explicit Pid(const PidConfig& config);

    /**
     * Run one control step.
     *
     * @param setpoint  Target value.
     * @param measured  Current measured value.
     * @param dt        Time elapsed since the previous call, in seconds.
     * @return          Clamped controller output.
     */
    double update(double setpoint, double measured, double dt);

    /** Clear accumulated state. Call when the controller is disabled. */
    void reset();

private:
    PidConfig config_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    bool has_previous_ = false;
};

}  // namespace eae
