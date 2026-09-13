#include "eae/pid.hpp"

#include <gtest/gtest.h>

using eae::Pid;
using eae::PidConfig;

namespace {

PidConfig basic_config() {
    PidConfig c;
    c.kp = 2.0;
    c.ki = 0.0;
    c.kd = 0.0;
    c.output_min = 0.0;
    c.output_max = 100.0;
    return c;
}

}  // namespace

TEST(PidTest, ZeroErrorProducesZeroOutput) {
    Pid pid(basic_config());
    EXPECT_DOUBLE_EQ(pid.update(50.0, 50.0, 0.1), 0.0);
}

TEST(PidTest, ProportionalTermScalesWithError) {
    Pid pid(basic_config());
    // kp = 2.0, error = 5.0, so the output should be 10.0.
    EXPECT_DOUBLE_EQ(pid.update(55.0, 50.0, 0.1), 10.0);
}

TEST(PidTest, OutputIsClampedToMaximum) {
    Pid pid(basic_config());
    // A large error would give 200.0 uncapped; it must clamp to 100.0.
    EXPECT_DOUBLE_EQ(pid.update(150.0, 50.0, 0.1), 100.0);
}

TEST(PidTest, OutputIsClampedToMinimum) {
    Pid pid(basic_config());
    // Negative error must not produce a negative fan command.
    EXPECT_DOUBLE_EQ(pid.update(40.0, 50.0, 0.1), 0.0);
}

TEST(PidTest, NonPositiveTimestepIsRejected) {
    Pid pid(basic_config());
    EXPECT_DOUBLE_EQ(pid.update(55.0, 50.0, 0.0), 0.0);
}

TEST(PidTest, IntegralDoesNotWindUpWhileSaturated) {
    PidConfig c = basic_config();
    c.ki = 1.0;
    Pid pid(c);

    // Hold a large error long enough to saturate the output.
    for (int i = 0; i < 100; ++i) {
        pid.update(150.0, 50.0, 0.1);
    }

    // With the error now reversed, a wound-up integral would keep the output
    // pinned high for many steps. Anti-windup should let it fall promptly.
    const double output = pid.update(40.0, 50.0, 0.1);
    EXPECT_LT(output, 100.0);
}

TEST(PidTest, ResetClearsAccumulatedState) {
    PidConfig c = basic_config();
    c.ki = 1.0;
    Pid pid(c);

    for (int i = 0; i < 10; ++i) {
        pid.update(60.0, 50.0, 0.1);
    }
    pid.reset();

    // A reset controller must produce exactly what a freshly constructed one
    // does, given the same input.
    Pid fresh(c);
    EXPECT_DOUBLE_EQ(pid.update(55.0, 50.0, 0.1),
                     fresh.update(55.0, 50.0, 0.1));
}
