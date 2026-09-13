#include "eae/can_bus.hpp"
#include "eae/can_messages.hpp"

#include <gtest/gtest.h>

using namespace eae;

// ---------------------------------------------------------------------------
// Bus simulation
// ---------------------------------------------------------------------------

TEST(CanBusTest, ReceiveOnEmptyBusReturnsNothing) {
    SimulatedCanBus bus;
    EXPECT_FALSE(bus.receive().has_value());
}

TEST(CanBusTest, TransmittedFramesAreRecorded) {
    SimulatedCanBus bus;
    CanFrame frame;
    frame.id = 0x123;
    frame.dlc = 1;
    frame.data[0] = 0x42;

    bus.send(frame);

    ASSERT_EQ(bus.transmitted_count(), 1u);
    const auto sent = bus.pop_transmitted();
    ASSERT_TRUE(sent.has_value());
    EXPECT_EQ(sent->id, 0x123u);
    EXPECT_EQ(sent->data[0], 0x42);
}

TEST(CanBusTest, SendingDoesNotLoopBackToReceive) {
    SimulatedCanBus bus;
    CanFrame frame;
    frame.id = 0x123;
    bus.send(frame);

    // A node does not receive its own transmissions in this model.
    EXPECT_FALSE(bus.receive().has_value());
}

TEST(CanBusTest, InjectedFramesAreReceived) {
    SimulatedCanBus bus;
    CanFrame frame;
    frame.id = 0x200;
    frame.dlc = 2;

    bus.inject(frame);

    const auto received = bus.receive();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->id, 0x200u);
    EXPECT_EQ(bus.pending_count(), 0u);
}

TEST(CanBusTest, FramesAreReceivedInOrder) {
    SimulatedCanBus bus;
    for (uint32_t i = 0; i < 3; ++i) {
        CanFrame frame;
        frame.id = 0x300 + i;
        bus.inject(frame);
    }

    EXPECT_EQ(bus.receive()->id, 0x300u);
    EXPECT_EQ(bus.receive()->id, 0x301u);
    EXPECT_EQ(bus.receive()->id, 0x302u);
}

TEST(CanBusTest, OverflowDropsOldestAndIsCounted) {
    SimulatedCanBus bus;
    const std::size_t over = SimulatedCanBus::kQueueLimit + 5;

    for (std::size_t i = 0; i < over; ++i) {
        CanFrame frame;
        frame.id = static_cast<uint32_t>(i);
        bus.inject(frame);
    }

    EXPECT_EQ(bus.pending_count(), SimulatedCanBus::kQueueLimit);
    EXPECT_EQ(bus.dropped_count(), 5u);
    // The five oldest were discarded, so the queue now starts at frame 5.
    EXPECT_EQ(bus.receive()->id, 5u);
}

// ---------------------------------------------------------------------------
// Message encoding
// ---------------------------------------------------------------------------

TEST(CanMessageTest, CoolantStatusRoundTrips) {
    CoolantStatus original;
    original.temperature_c = 47.3;
    original.sensor_valid = true;

    const auto decoded = decode_coolant_status(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_NEAR(decoded->temperature_c, 47.3, 0.05);
    EXPECT_TRUE(decoded->sensor_valid);
}

TEST(CanMessageTest, CoolantStatusHandlesNegativeTemperature) {
    CoolantStatus original;
    original.temperature_c = -15.5;
    original.sensor_valid = true;

    const auto decoded = decode_coolant_status(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_NEAR(decoded->temperature_c, -15.5, 0.05);
}

TEST(CanMessageTest, CoolantStatusCarriesInvalidSensorFlag) {
    CoolantStatus original;
    original.sensor_valid = false;

    const auto decoded = decode_coolant_status(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_FALSE(decoded->sensor_valid);
}

TEST(CanMessageTest, ActuatorStatusRoundTrips) {
    ActuatorStatus original;
    original.pump_enabled = true;
    original.fan_pwm_percent = 65;

    const auto decoded = decode_actuator_status(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->pump_enabled);
    EXPECT_EQ(decoded->fan_pwm_percent, 65);
}

TEST(CanMessageTest, SystemStatusRoundTrips) {
    SystemStatus original;
    original.state = State::Fault;
    original.fault = FaultCode::LowCoolant;

    const auto decoded = decode_system_status(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->state, State::Fault);
    EXPECT_EQ(decoded->fault, FaultCode::LowCoolant);
}

TEST(CanMessageTest, DecoderRejectsWrongIdentifier) {
    CanFrame frame = encode(ActuatorStatus{});
    // The payload is valid, but it is not a coolant status message.
    EXPECT_FALSE(decode_coolant_status(frame).has_value());
}

TEST(CanMessageTest, DecoderRejectsTruncatedFrame) {
    CanFrame frame = encode(CoolantStatus{});
    frame.dlc = 1;  // Too short to contain the temperature and flag.
    EXPECT_FALSE(decode_coolant_status(frame).has_value());
}

TEST(CanMessageTest, DecoderRejectsUnknownStateValue) {
    CanFrame frame = encode(SystemStatus{});
    frame.data[0] = 99;  // Not a state this build knows about.
    EXPECT_FALSE(decode_system_status(frame).has_value());
}

// ---------------------------------------------------------------------------
// Setpoint command, the one message the controller receives
// ---------------------------------------------------------------------------

TEST(CanMessageTest, SetpointCommandRoundTrips) {
    SetpointCommand original;
    original.setpoint_c = 45.0;

    const auto decoded = decode_setpoint_command(encode(original));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_NEAR(decoded->setpoint_c, 45.0, 0.05);
}

TEST(CanMessageTest, SetpointBelowRangeIsRejected) {
    SetpointCommand original;
    original.setpoint_c = kSetpointMinC - 1.0;
    EXPECT_FALSE(decode_setpoint_command(encode(original)).has_value());
}

TEST(CanMessageTest, SetpointAboveRangeIsRejected) {
    SetpointCommand original;
    original.setpoint_c = kSetpointMaxC + 1.0;
    EXPECT_FALSE(decode_setpoint_command(encode(original)).has_value());
}

TEST(CanMessageTest, SetpointBoundariesAreAccepted) {
    SetpointCommand low{kSetpointMinC};
    SetpointCommand high{kSetpointMaxC};
    EXPECT_TRUE(decode_setpoint_command(encode(low)).has_value());
    EXPECT_TRUE(decode_setpoint_command(encode(high)).has_value());
}

// ---------------------------------------------------------------------------
// End to end: controller transmits status, display transmits a setpoint
// ---------------------------------------------------------------------------

TEST(CanIntegrationTest, StatusIsPublishedAndCommandIsReceived) {
    SimulatedCanBus bus;

    // The controller publishes one cycle of status.
    bus.send(encode(CoolantStatus{52.5, true}));
    bus.send(encode(ActuatorStatus{true, 80}));
    bus.send(encode(SystemStatus{State::Running, FaultCode::None}));

    EXPECT_EQ(bus.transmitted_count(), 3u);

    const auto coolant = decode_coolant_status(*bus.pop_transmitted());
    ASSERT_TRUE(coolant.has_value());
    EXPECT_NEAR(coolant->temperature_c, 52.5, 0.05);

    // The display sends a new target temperature back.
    bus.inject(encode(SetpointCommand{50.0}));

    const auto frame = bus.receive();
    ASSERT_TRUE(frame.has_value());
    const auto command = decode_setpoint_command(*frame);
    ASSERT_TRUE(command.has_value());
    EXPECT_NEAR(command->setpoint_c, 50.0, 0.05);
}
