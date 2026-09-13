#include "eae/pid.hpp"

#include <CLI/CLI.hpp>

#include <iostream>

int main(int argc, char** argv) {
    CLI::App app{"EAE cooling loop firmware"};

    double setpoint = 45.0;
    double kp = 2.0;
    double ki = 0.1;
    double kd = 0.05;

    app.add_option("--setpoint", setpoint, "Target coolant temperature in C");
    app.add_option("--kp", kp, "Proportional gain");
    app.add_option("--ki", ki, "Integral gain");
    app.add_option("--kd", kd, "Derivative gain");

    CLI11_PARSE(app, argc, argv);

    eae::PidConfig config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;

    eae::Pid pid(config);

    std::cout << "EAE firmware skeleton\n"
              << "  setpoint = " << setpoint << " C\n"
              << "  gains    = " << kp << ", " << ki << ", " << kd << "\n\n";

    // Placeholder: one control step against a fake measurement, proving the
    // library, the argument parsing, and the build are all wired together.
    const double fan = pid.update(setpoint, setpoint + 10.0, 0.1);
    std::cout << "Measured 10 C above setpoint -> fan command " << fan << "%\n";

    return 0;
}
