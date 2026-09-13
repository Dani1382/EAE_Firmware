#include "eae/thermal_plant.hpp"

#include <algorithm>

namespace eae {

ThermalPlant::ThermalPlant(const PlantConfig& config)
    : config_(config), temperature_c_(config.ambient_c) {}

double ThermalPlant::update(double heat_input_w,
                            double fan_pwm_percent,
                            bool pump_running,
                            double dt) {
    if (dt <= 0.0) {
        return temperature_c_;
    }

    const double fan_fraction =
        std::clamp(fan_pwm_percent, 0.0, 100.0) / 100.0;

    // Heat rejection scales with how far above ambient the coolant is, which
    // is why a hot loop sheds heat faster than a warm one.
    double loss_coefficient =
        config_.passive_loss_w_per_c + config_.fan_loss_w_per_c * fan_fraction;

    // A radiator with no flow through it cannot reject much, regardless of
    // how hard the fan is working.
    if (!pump_running) {
        loss_coefficient *= config_.stagnant_flow_factor;
    }

    const double delta_to_ambient = temperature_c_ - config_.ambient_c;
    const double heat_out_w = loss_coefficient * delta_to_ambient;

    // Net power into the coolant, converted to a rate of temperature change
    // by the thermal mass, then integrated over the timestep.
    const double net_w = heat_input_w - heat_out_w;
    temperature_c_ += (net_w / config_.thermal_mass_j_per_c) * dt;

    return temperature_c_;
}

}  // namespace eae
