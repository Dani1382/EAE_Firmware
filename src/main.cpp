/*
 * EAE_Firmware
 *
 * Cooling loop controller simulation.
 *
 * Each cycle the controller samples its inputs, advances the state machine,
 * runs the PID when closed-loop control is permitted, drives a simulated
 * thermal plant, and publishes status onto a simulated CAN bus for the
 * PV450 display. Setpoint commands arriving from the display are read back
 * off the same bus.
 */

#include "eae/can_bus.hpp"
#include "eae/can_messages.hpp"
#include "eae/pid.hpp"
#include "eae/state_machine.hpp"
#include "eae/thermal_plant.hpp"

#include <CLI/CLI.hpp>

#include <cstdio>
#include <string>

namespace {

/*
 * Scripted events for the demonstration run.
 *
 * The timeline exercises every branch of the controller: warm-up under light
 * load, a step increase in load, a setpoint change arriving over CAN, a low
 * coolant fault and its recovery, and finally key-off with the shutdown
 * purge.
 */
struct Scenario {
    double load_step_s = 60.0;      // Heat input rises.
    double setpoint_msg_s = 100.0;  // Display requests a new setpoint.
    double fault_start_s = 125.0;   // Low coolant asserted.
    double fault_end_s = 145.0;     // Low coolant clears.
    double ignition_off_s = 165.0;  // Key off.

    double light_load_w = 4000.0;
    double heavy_load_w = 9000.0;
    double commanded_setpoint_c = 50.0;
};

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"EAE cooling loop firmware simulation"};

    double setpoint = 45.0;
    double kp = 8.0;
    double ki = 0.4;
    double kd = 2.0;
    double duration = 180.0;
    double dt = 0.1;
    double print_interval = 5.0;
    bool quiet = false;

    app.add_option("--setpoint", setpoint, "Target coolant temperature in C");
    app.add_option("--kp", kp, "Proportional gain");
    app.add_option("--ki", ki, "Integral gain");
    app.add_option("--kd", kd, "Derivative gain");
    app.add_option("--duration", duration, "Simulated run time in seconds");
    app.add_option("--dt", dt, "Control cycle period in seconds");
    app.add_option("--print-interval", print_interval,
                   "Seconds between printed rows");
    app.add_flag("--quiet", quiet, "Suppress the per-cycle table");

    CLI11_PARSE(app, argc, argv);

    const Scenario scenario;

    eae::PidConfig pid_config;
    pid_config.kp = kp;
    pid_config.ki = ki;
    pid_config.kd = kd;
    pid_config.output_min = 0.0;
    pid_config.output_max = 100.0;

    // Cooling: more fan lowers the measured temperature.
    pid_config.reverse_acting = true;

    eae::Pid pid(pid_config);
    eae::StateMachine machine;
    eae::ThermalPlant plant;
    eae::SimulatedCanBus bus;

    std::printf("EAE cooling loop simulation\n");
    std::printf("  setpoint %.1f C   gains %.2f / %.2f / %.2f   dt %.2f s\n\n",
                setpoint, kp, ki, kd, dt);

    if (!quiet) {
        std::printf("%8s | %-9s | %7s | %8s | %5s | %4s | %-15s\n",
                    "time", "state", "coolant", "setpoint", "fan", "pump",
                    "fault");
        std::printf("---------|-----------|---------|----------|-------|"
                    "------|----------------\n");
    }

    double next_print = 0.0;
    double heat_input_w = scenario.light_load_w;
    bool setpoint_sent = false;

    for (double t = 0.0; t <= duration; t += dt) {
        // ------------------------------------------------------------------
        // Scripted environment changes.
        // ------------------------------------------------------------------
        if (t >= scenario.load_step_s) {
            heat_input_w = scenario.heavy_load_w;
        }

        // The display transmits a new target temperature once, partway
        // through the run. This is the inbound half of the CAN traffic.
        if (!setpoint_sent && t >= scenario.setpoint_msg_s) {
            bus.inject(eae::encode(
                eae::SetpointCommand{scenario.commanded_setpoint_c}));
            setpoint_sent = true;
        }

        // ------------------------------------------------------------------
        // Read anything waiting on the bus.
        //
        // Frames from another node are untrusted: the decoder validates the
        // identifier, length, and value range, and anything that fails is
        // ignored rather than acted on.
        // ------------------------------------------------------------------
        while (const auto frame = bus.receive()) {
            if (const auto command = eae::decode_setpoint_command(*frame)) {
                setpoint = command->setpoint_c;
                std::printf("%8.1f | CAN: setpoint command accepted, "
                            "now %.1f C\n", t, setpoint);
            }
        }

        // ------------------------------------------------------------------
        // Sample inputs.
        // ------------------------------------------------------------------
        eae::MachineInputs inputs;
        inputs.ignition_on = (t < scenario.ignition_off_s);
        inputs.sensor_valid = true;
        inputs.low_coolant = (t >= scenario.fault_start_s &&
                              t < scenario.fault_end_s);
        inputs.coolant_temp_c = plant.temperature_c();

        // ------------------------------------------------------------------
        // Advance the controller.
        // ------------------------------------------------------------------
        const auto outputs = machine.update(inputs, dt);

        double fan_pwm = 0.0;
        if (outputs.request_max_cooling) {
            // Fault response bypasses the PID entirely.
            fan_pwm = 100.0;
            pid.reset();
        } else if (outputs.closed_loop_active) {
            fan_pwm = pid.update(setpoint, inputs.coolant_temp_c, dt);
        } else {
            // Priming, idle, or purging: the loop is not under control, so
            // the accumulated integral would be stale by the time it resumes.
            pid.reset();
        }

        // ------------------------------------------------------------------
        // Drive the plant and publish status.
        // ------------------------------------------------------------------
        const double load_w = inputs.ignition_on ? heat_input_w : 0.0;
        plant.update(load_w, fan_pwm, outputs.pump_enabled, dt);

        bus.send(eae::encode(eae::CoolantStatus{inputs.coolant_temp_c,
                                                inputs.sensor_valid}));
        bus.send(eae::encode(eae::ActuatorStatus{
            outputs.pump_enabled, static_cast<uint8_t>(fan_pwm)}));
        bus.send(eae::encode(eae::SystemStatus{outputs.state, outputs.fault}));

        // The display consumes frames as they arrive; draining the transmit
        // record here keeps the simulated buffer from filling.
        while (bus.pop_transmitted()) {
        }

        // ------------------------------------------------------------------
        // Report.
        // ------------------------------------------------------------------
        if (!quiet && t >= next_print) {
            std::printf("%8.1f | %-9s | %6.1fC | %7.1fC | %4.0f%% | %4s | %-15s\n",
                        t,
                        std::string(eae::to_string(outputs.state)).c_str(),
                        inputs.coolant_temp_c,
                        setpoint,
                        fan_pwm,
                        outputs.pump_enabled ? "ON" : "OFF",
                        std::string(eae::to_string(outputs.fault)).c_str());
            next_print += print_interval;
        }
    }

    std::printf("\nSimulation complete.\n");
    std::printf("  final coolant temperature %.1f C\n", plant.temperature_c());
    std::printf("  frames dropped by the bus: %zu\n", bus.dropped_count());

    return 0;
}
