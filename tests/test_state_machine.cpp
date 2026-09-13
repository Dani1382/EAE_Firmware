#include "eae/state_machine.hpp"

#include <gtest/gtest.h>

using namespace eae;

namespace {

// A healthy, ignition-on input set. Individual tests mutate one field so the
// intent of each test stays visible.
MachineInputs healthy() {
    MachineInputs in;
    in.ignition_on = true;
    in.sensor_valid = true;
    in.low_coolant = false;
    in.coolant_temp_c = 40.0;
    return in;
}

// Steps the machine until it reaches Running, or gives up.
void run_until_running(StateMachine& sm) {
    for (int i = 0; i < 100 && sm.state() != State::Running; ++i) {
        sm.update(healthy(), 0.1);
    }
}

}  // namespace

TEST(StateMachineTest, StartsInInit) {
    StateMachine sm;
    EXPECT_EQ(sm.state(), State::Init);
}

TEST(StateMachineTest, LeavesInitOnFirstUpdate) {
    StateMachine sm;
    MachineInputs in;  // ignition off
    in.sensor_valid = true;
    EXPECT_EQ(sm.update(in, 0.1).state, State::Idle);
}

TEST(StateMachineTest, PumpIsOffWhileIdle) {
    StateMachine sm;
    MachineInputs in;
    in.sensor_valid = true;
    const auto out = sm.update(in, 0.1);
    EXPECT_EQ(out.state, State::Idle);
    EXPECT_FALSE(out.pump_enabled);
    EXPECT_FALSE(out.closed_loop_active);
}

TEST(StateMachineTest, IgnitionOnMovesToStarting) {
    StateMachine sm;
    sm.update(MachineInputs{}, 0.1);  // Init -> Idle
    const auto out = sm.update(healthy(), 0.1);
    EXPECT_EQ(out.state, State::Starting);
    EXPECT_TRUE(out.pump_enabled);
}

TEST(StateMachineTest, ClosedLoopIsInactiveWhilePriming) {
    StateMachine sm;
    sm.update(MachineInputs{}, 0.1);
    const auto out = sm.update(healthy(), 0.1);
    ASSERT_EQ(out.state, State::Starting);
    // The pump circulates, but the PID must not act on a reading taken from
    // coolant that has not started moving yet.
    EXPECT_FALSE(out.closed_loop_active);
}

TEST(StateMachineTest, ReachesRunningAfterPrimeDelay) {
    StateMachine sm;
    run_until_running(sm);
    EXPECT_EQ(sm.state(), State::Running);
}

TEST(StateMachineTest, RunningEnablesClosedLoop) {
    StateMachine sm;
    run_until_running(sm);
    const auto out = sm.update(healthy(), 0.1);
    EXPECT_TRUE(out.pump_enabled);
    EXPECT_TRUE(out.closed_loop_active);
    EXPECT_FALSE(out.request_max_cooling);
}

TEST(StateMachineTest, InvalidSensorTripsFault) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.sensor_valid = false;
    const auto out = sm.update(in, 0.1);

    EXPECT_EQ(out.state, State::Fault);
    EXPECT_EQ(out.fault, FaultCode::SensorInvalid);
    EXPECT_TRUE(out.request_max_cooling);
    EXPECT_TRUE(out.pump_enabled);
}

TEST(StateMachineTest, LowCoolantTripsFault) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.low_coolant = true;
    const auto out = sm.update(in, 0.1);

    EXPECT_EQ(out.state, State::Fault);
    EXPECT_EQ(out.fault, FaultCode::LowCoolant);
}

TEST(StateMachineTest, OvertemperatureTripsFault) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.coolant_temp_c = 70.0;
    const auto out = sm.update(in, 0.1);

    EXPECT_EQ(out.state, State::Fault);
    EXPECT_EQ(out.fault, FaultCode::Overtemperature);
    EXPECT_TRUE(out.request_max_cooling);
}

TEST(StateMachineTest, OvertemperatureLatchHoldsAboveClearPoint) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.coolant_temp_c = 70.0;
    sm.update(in, 0.1);  // trip

    // 62 C is below the 65 C trip but above the 60 C clear point, so the
    // latch must hold.
    in.coolant_temp_c = 62.0;
    const auto out = sm.update(in, 0.1);

    EXPECT_EQ(out.state, State::Fault);
    EXPECT_EQ(out.fault, FaultCode::Overtemperature);
}

TEST(StateMachineTest, OvertemperatureClearsAtClearPoint) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.coolant_temp_c = 70.0;
    sm.update(in, 0.1);
    ASSERT_EQ(sm.state(), State::Fault);

    in.coolant_temp_c = 58.0;
    const auto out = sm.update(in, 0.1);

    // Recovery goes back through Starting so the pump re-primes.
    EXPECT_EQ(out.state, State::Starting);
}

TEST(StateMachineTest, IgnitionOffFromRunningEntersShutdown) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.ignition_on = false;
    const auto out = sm.update(in, 0.1);

    EXPECT_EQ(out.state, State::Shutdown);
    // The pump keeps running to purge residual heat.
    EXPECT_TRUE(out.pump_enabled);
}

TEST(StateMachineTest, IgnitionOffOverridesActiveFault) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.low_coolant = true;
    sm.update(in, 0.1);
    ASSERT_EQ(sm.state(), State::Fault);

    in.ignition_on = false;
    EXPECT_EQ(sm.update(in, 0.1).state, State::Shutdown);
}

TEST(StateMachineTest, ShutdownReturnsToIdleAfterPurge) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.ignition_on = false;

    // Purge defaults to 5 seconds; step well past it.
    for (int i = 0; i < 100; ++i) {
        sm.update(in, 0.1);
    }

    EXPECT_EQ(sm.state(), State::Idle);
}

TEST(StateMachineTest, KeyCycleClearsLatchedOvertemperature) {
    StateMachine sm;
    run_until_running(sm);

    MachineInputs in = healthy();
    in.coolant_temp_c = 70.0;
    sm.update(in, 0.1);
    ASSERT_EQ(sm.state(), State::Fault);

    // Key off, wait out the purge, then key back on at a temperature that is
    // hot but below the trip point.
    in.ignition_on = false;
    for (int i = 0; i < 100; ++i) {
        sm.update(in, 0.1);
    }
    ASSERT_EQ(sm.state(), State::Idle);

    in.ignition_on = true;
    in.coolant_temp_c = 62.0;
    const auto out = sm.update(in, 0.1);

    // A stale latch would drop it straight back into Fault.
    EXPECT_EQ(out.state, State::Starting);
    EXPECT_EQ(out.fault, FaultCode::None);
}
