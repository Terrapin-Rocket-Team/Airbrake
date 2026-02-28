#pragma once

#include <NativeTestHelper.h>
#include <unity.h>

#include "../../src/MotorDriver/MDNative.h"

namespace test_motor_driver_native {

void test_motor_moves_toward_target_with_rate_limit()
{
    setMillis(0);
    MDNative motor("MotorSim");
    TEST_ASSERT_EQUAL(0, motor.init());

    motor.setPos(26.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, motor.getPosition());

    setMillis(100);
    TEST_ASSERT_EQUAL(0, motor.read());
    const float pos100ms = motor.getPosition();
    TEST_ASSERT_TRUE(pos100ms > 0.0f);
    TEST_ASSERT_TRUE(pos100ms <= 26.0f);

    setMillis(2000);
    TEST_ASSERT_EQUAL(0, motor.read());
    const float pos2s = motor.getPosition();
    TEST_ASSERT_TRUE(pos2s >= pos100ms);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 26.0f, pos2s);

    resetMillis();
}

void test_motor_reports_target_angle_and_can_retract()
{
    setMillis(0);
    MDNative motor("MotorSim");
    TEST_ASSERT_EQUAL(0, motor.init());

    const float commandAngleDeg = 40.0f;
    motor.setPos(motor.angleToPos(commandAngleDeg));

    setMillis(3000);
    TEST_ASSERT_EQUAL(0, motor.read());
    const float deployedPos = motor.getPosition();
    TEST_ASSERT_TRUE(deployedPos > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, commandAngleDeg, motor.posToAngle(deployedPos));

    motor.setPos(0.0f);
    setMillis(6000);
    TEST_ASSERT_EQUAL(0, motor.read());
    TEST_ASSERT_TRUE(motor.getPosition() < deployedPos);

    resetMillis();
}

void run_test_motor_driver_native_tests()
{
    RUN_TEST(test_motor_moves_toward_target_with_rate_limit);
    RUN_TEST(test_motor_reports_target_angle_and_can_retract);
}

} // namespace test_motor_driver_native

