#pragma once

#include <cmath>

#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include <unity.h>

#include "../../src/ErrorCorrectedBaro.h"

namespace test_error_corrected_baro {

namespace {

class FailingBarometer : public FakeBarometer {
public:
    int update(double currentTime = -1) override
    {
        (void)currentTime;
        return -7;
    }
};

constexpr double kTol = 1e-3;
constexpr double kPi = 3.14159265358979323846;

} // namespace

void test_begin_fails_without_inner_barometer()
{
    astra::ErrorCorrectedBaro corrected;
    TEST_ASSERT_EQUAL(-1, corrected.begin());
}

void test_set_inner_barometer_disables_inner_auto_update()
{
    FakeBarometer inner;
    inner.begin();
    inner.setAutoUpdate(true);

    astra::ErrorCorrectedBaro corrected;
    corrected.setInnerBarometer(&inner);

    TEST_ASSERT_FALSE(inner.getAutoUpdate());
}

void test_update_propagates_inner_sensor_readings()
{
    FakeBarometer inner;
    inner.begin();
    inner.set(980.0, 13.25);

    astra::ErrorCorrectedBaro corrected(&inner);
    TEST_ASSERT_EQUAL(0, corrected.begin());
    TEST_ASSERT_EQUAL(0, corrected.update());

    TEST_ASSERT_DOUBLE_WITHIN(kTol, 980.0, corrected.getPressure());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 13.25, corrected.getTemp());
}

void test_update_propagates_inner_sensor_error()
{
    FailingBarometer inner;
    inner.begin();

    astra::ErrorCorrectedBaro corrected(&inner);
    TEST_ASSERT_EQUAL(0, corrected.begin());
    TEST_ASSERT_EQUAL(-7, corrected.update());
}

void test_correction_disabled_forces_zero_delta()
{
    FakeBarometer inner;
    inner.begin();
    inner.set(1013.25, 20.0);

    astra::ErrorCorrectedBaro corrected(&inner);
    corrected.setCorrectionParams(0.1, 0.15);
    corrected.setCorrectionInputs(30.0, 500.0);
    TEST_ASSERT_EQUAL(0, corrected.begin());
    TEST_ASSERT_EQUAL(0, corrected.update());
    TEST_ASSERT_TRUE(corrected.getAltitudeDelta() > 0.0);

    corrected.setCorrectionEnabled(false);
    corrected.setCorrectionInputs(45.0, 900.0);
    TEST_ASSERT_EQUAL(0, corrected.update());

    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, corrected.getAltitudeDelta());
}

void test_correction_uses_first_order_filter()
{
    FakeBarometer inner;
    inner.begin();
    inner.set(1013.25, 20.0);

    astra::ErrorCorrectedBaro corrected(&inner);
    corrected.setCorrectionParams(0.1, 0.15); // alpha = 0.4 (dt=0.1)
    corrected.setCorrectionInputs(30.0, 500.0);
    TEST_ASSERT_EQUAL(0, corrected.begin());
    TEST_ASSERT_EQUAL(0, corrected.update());

    const double expectedAltitudeError = 0.1 * 500.0 * std::sin(30.0 * kPi / 180.0);
    const double expectedDelta = (0.1 / (0.1 + 0.15)) * expectedAltitudeError;
    TEST_ASSERT_DOUBLE_WITHIN(0.01, expectedDelta, corrected.getAltitudeDelta());
}

void test_correction_clamps_angle_to_max_limit()
{
    FakeBarometer inner;
    inner.begin();
    inner.set(1013.25, 20.0);

    astra::ErrorCorrectedBaro corrected(&inner);
    corrected.setCorrectionParams(0.1, 0.15);
    corrected.setMaxCorrectionAngle(20.0);
    corrected.setCorrectionInputs(80.0, 600.0);
    TEST_ASSERT_EQUAL(0, corrected.begin());
    TEST_ASSERT_EQUAL(0, corrected.update());

    const double expectedAltitudeError = 0.1 * 600.0 * std::sin(20.0 * kPi / 180.0);
    const double expectedDelta = (0.1 / (0.1 + 0.15)) * expectedAltitudeError;
    TEST_ASSERT_DOUBLE_WITHIN(0.01, expectedDelta, corrected.getAltitudeDelta());
}

void run_test_error_corrected_baro_tests()
{
    RUN_TEST(test_begin_fails_without_inner_barometer);
    RUN_TEST(test_set_inner_barometer_disables_inner_auto_update);
    RUN_TEST(test_update_propagates_inner_sensor_readings);
    RUN_TEST(test_update_propagates_inner_sensor_error);
    RUN_TEST(test_correction_disabled_forces_zero_delta);
    RUN_TEST(test_correction_uses_first_order_filter);
    RUN_TEST(test_correction_clamps_angle_to_max_limit);
}

} // namespace test_error_corrected_baro
