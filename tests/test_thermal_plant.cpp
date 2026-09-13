#include "eae/thermal_plant.hpp"

#include <gtest/gtest.h>

using namespace eae;

TEST(ThermalPlantTest, StartsAtAmbient) {
    PlantConfig config;
    config.ambient_c = 22.0;
    ThermalPlant plant(config);
    EXPECT_DOUBLE_EQ(plant.temperature_c(), 22.0);
}

TEST(ThermalPlantTest, HeatInputRaisesTemperature) {
    ThermalPlant plant;
    const double before = plant.temperature_c();
    plant.update(10000.0, 0.0, true, 1.0);
    EXPECT_GT(plant.temperature_c(), before);
}

TEST(ThermalPlantTest, HotCoolantCoolsTowardAmbient) {
    PlantConfig config;
    config.ambient_c = 25.0;
    ThermalPlant plant(config);
    plant.set_temperature(80.0);

    // No heat input, fan at full: temperature must fall.
    plant.update(0.0, 100.0, true, 1.0);
    EXPECT_LT(plant.temperature_c(), 80.0);
}

TEST(ThermalPlantTest, FanIncreasesCoolingRate) {
    PlantConfig config;
    ThermalPlant with_fan(config);
    ThermalPlant without_fan(config);
    with_fan.set_temperature(70.0);
    without_fan.set_temperature(70.0);

    for (int i = 0; i < 10; ++i) {
        with_fan.update(0.0, 100.0, true, 1.0);
        without_fan.update(0.0, 0.0, true, 1.0);
    }

    EXPECT_LT(with_fan.temperature_c(), without_fan.temperature_c());
}

TEST(ThermalPlantTest, StoppedPumpCripplesHeatRejection) {
    PlantConfig config;
    ThermalPlant flowing(config);
    ThermalPlant stagnant(config);
    flowing.set_temperature(70.0);
    stagnant.set_temperature(70.0);

    // Same fan command, but one loop has no circulation.
    for (int i = 0; i < 10; ++i) {
        flowing.update(0.0, 100.0, true, 1.0);
        stagnant.update(0.0, 100.0, false, 1.0);
    }

    EXPECT_LT(flowing.temperature_c(), stagnant.temperature_c());
}

TEST(ThermalPlantTest, ReachesSteadyStateUnderConstantLoad) {
    ThermalPlant plant;

    // Run a constant load with a constant fan command for a simulated hour.
    for (int i = 0; i < 3600; ++i) {
        plant.update(8000.0, 50.0, true, 1.0);
    }

    const double first = plant.temperature_c();
    plant.update(8000.0, 50.0, true, 1.0);

    // At equilibrium heat in equals heat out, so temperature stops moving.
    EXPECT_NEAR(plant.temperature_c(), first, 0.01);
}

TEST(ThermalPlantTest, NonPositiveTimestepLeavesTemperatureUnchanged) {
    ThermalPlant plant;
    plant.set_temperature(50.0);
    plant.update(10000.0, 0.0, true, 0.0);
    EXPECT_DOUBLE_EQ(plant.temperature_c(), 50.0);
}
