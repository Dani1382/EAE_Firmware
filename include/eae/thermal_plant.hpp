#pragma once

namespace eae {

/**
 * Physical parameters of the cooling loop being simulated.
 *
 * These are plausible figures for a loop of this size, chosen so the
 * simulation moves at a watchable rate. They are not measured values from
 * any real vehicle.
 */
struct PlantConfig {
    double ambient_c = 25.0;

    // Effective thermal mass of coolant plus the metal it flows through,
    // expressed in joules per degree.
    double thermal_mass_j_per_c = 10000.0;

    // Heat rejected per degree of temperature difference above ambient, with
    // the fan stopped. Represents passive loss from hoses and the radiator.
    double passive_loss_w_per_c = 15.0;

    // Additional heat rejection per degree, at full fan. Forced airflow does
    // most of the work.
    double fan_loss_w_per_c = 500.0;

    // With the pump off, coolant is not circulating, so the radiator sees
    // almost no flow. Rejection collapses to this fraction of normal.
    double stagnant_flow_factor = 0.08;
};

/**
 * First-order thermal model of the coolant loop.
 *
 * Heat in comes from the inverter and DC-DC. Heat out depends on the
 * temperature difference to ambient, scaled by fan speed and by whether the
 * pump is circulating. Temperature integrates the difference over time.
 *
 * This exists so the PID has something real to act against. Without a plant
 * the controller cannot be demonstrated, only unit tested.
 */
class ThermalPlant {
public:
    explicit ThermalPlant(const PlantConfig& config = {});

    /** Set the starting coolant temperature. */
    void set_temperature(double celsius) { temperature_c_ = celsius; }

    /**
     * Advance the simulation by one timestep.
     *
     * @param heat_input_w     Heat being dumped into the loop, in watts.
     * @param fan_pwm_percent  Fan command, 0 to 100.
     * @param pump_running     Whether coolant is circulating.
     * @param dt               Timestep in seconds.
     * @return                 New coolant temperature.
     */
    double update(double heat_input_w,
                  double fan_pwm_percent,
                  bool pump_running,
                  double dt);

    double temperature_c() const { return temperature_c_; }

private:
    PlantConfig config_;
    double temperature_c_ = 25.0;
};

}  // namespace eae
