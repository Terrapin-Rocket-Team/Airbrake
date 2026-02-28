#pragma once

#include <cmath>

#include <Math/Matrix.h>
#include <Math/Vector.h>
#include <NativeTestHelper.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <UnitTestSensors.h>
#include <unity.h>

#include <RocketState.h>

#include "../../src/AirbrakeController.h"
#include "../../src/MotorDriver/MDNative.h"
#include "../mocks/MockLinearKalmanFilter.h"
#include "../mocks/MockMahony.h"

namespace test_airbrake_controller {

namespace {

class TestMotorDriver : public MDNative {
public:
    TestMotorDriver() : MDNative("TestMotor") {}

    void forceInitialized(bool value)
    {
        initialized = value;
    }
};

FakeBarometer *fakeBaro = nullptr;
FakeIMU *fakeIMU = nullptr;
astra::SensorManager *sensorManager = nullptr;
astra_mocks::MockLinearKalmanFilter *kalman = nullptr;
astra_mocks::MockMahony *mahony = nullptr;
astra_rocket::RocketState *state = nullptr;
TestMotorDriver *motor = nullptr;
astra::ErrorCorrectedBaro *correctedBaro = nullptr;
AirbrakeController *controller = nullptr;

void local_setUp()
{
    fakeBaro = new FakeBarometer();
    fakeIMU = new FakeIMU();
    fakeBaro->begin();
    fakeIMU->begin();

    sensorManager = new astra::SensorManager();
    sensorManager->setAccelSource(fakeIMU->getAccelSensor());
    sensorManager->setGyroSource(fakeIMU->getGyroSensor());
    sensorManager->setBaroSource(fakeBaro);
    sensorManager->begin();

    kalman = new astra_mocks::MockLinearKalmanFilter(6, 0, 9);
    mahony = new astra_mocks::MockMahony();
    mahony->setMockEarthAcceleration(astra::Vector<3>(0, 0, -9.81));

    state = new astra_rocket::RocketState(kalman, mahony);
    state->withSensorManager(sensorManager);
    state->begin();
    state->setGroundLevel(0.0);

    motor = new TestMotorDriver();
    correctedBaro = new astra::ErrorCorrectedBaro(fakeBaro);
    correctedBaro->begin();

    controller = new AirbrakeController(motor, state, correctedBaro, "TestAirbrake");
    controller->setTargetApogee(2000.0);
    controller->setRocketParameters(40.0, 0.012, 0.010);
    controller->setBinarySearchParams(8, 5.0, 1.0);
    controller->setAngleLimits(0.0, 65.0);
    controller->setGroundAltitude(0.0);
    controller->setSimulationParams(0.05, 20.0);
    controller->enableAdaptiveCdA(false);
    controller->enableBaroCorrection(true, 0.052, 0.15);
    controller->enable();

    setMillis(0);
}

void local_tearDown()
{
    delete controller;
    delete correctedBaro;
    delete motor;
    delete state;
    delete mahony;
    delete kalman;
    delete sensorManager;
    delete fakeIMU;
    delete fakeBaro;

    controller = nullptr;
    correctedBaro = nullptr;
    motor = nullptr;
    state = nullptr;
    mahony = nullptr;
    kalman = nullptr;
    sensorManager = nullptr;
    fakeIMU = nullptr;
    fakeBaro = nullptr;

    resetMillis();
}

void set_state_and_update(double altitudeM, double velocityMps, double accelZMps2, unsigned long timeMs)
{
    astra::Matrix kfState = kalman->getState();
    kfState(2, 0) = altitudeM;
    kfState(5, 0) = velocityMps;
    kalman->setState(kfState);

    mahony->setMockEarthAcceleration(astra::Vector<3>(0.0, 0.0, accelZMps2));
    setMillis(timeMs);

    const double tSec = static_cast<double>(timeMs) / 1000.0;
    state->predictState(tSec);
    state->update(tSec);
}

} // namespace

void test_begin_requires_motor_and_state()
{
    local_setUp();

    AirbrakeController noMotor(nullptr, state, correctedBaro, "NoMotor");
    TEST_ASSERT_EQUAL(-1, noMotor.begin());

    AirbrakeController noState(motor, nullptr, correctedBaro, "NoState");
    TEST_ASSERT_EQUAL(-1, noState.begin());

    TEST_ASSERT_EQUAL(0, controller->begin());

    local_tearDown();
}

void test_enable_disable_state_tracking()
{
    local_setUp();

    TEST_ASSERT_TRUE(controller->isEnabled());
    controller->disable();
    TEST_ASSERT_FALSE(controller->isEnabled());
    controller->enable();
    TEST_ASSERT_TRUE(controller->isEnabled());

    local_tearDown();
}

void test_update_when_disabled_returns_error_and_clears_baro_inputs()
{
    local_setUp();

    controller->disable();
    correctedBaro->setCorrectionInputs(25.0, 800.0);
    TEST_ASSERT_TRUE(correctedBaro->getDynamicPressure() > 0.0);

    TEST_ASSERT_EQUAL(-1, controller->update(0.1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, correctedBaro->getCorrectionAngle());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, correctedBaro->getDynamicPressure());

    local_tearDown();
}

void test_update_when_motor_uninitialized_returns_error()
{
    local_setUp();

    motor->forceInitialized(false);
    correctedBaro->setCorrectionInputs(10.0, 500.0);

    TEST_ASSERT_EQUAL(-1, controller->update(0.2));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, correctedBaro->getCorrectionAngle());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, correctedBaro->getDynamicPressure());

    local_tearDown();
}

void test_update_with_zero_speed_sets_predicted_apogee_to_altitude()
{
    local_setUp();

    motor->forceInitialized(true);
    set_state_and_update(1234.5, 0.0, -9.81, 1000);
    state->setFlightStage(astra_rocket::BOOST);

    TEST_ASSERT_EQUAL(0, controller->update(1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1234.5, controller->getPredictedApogee());

    local_tearDown();
}

void test_update_in_coast_computes_dynamic_pressure()
{
    local_setUp();

    motor->forceInitialized(true);
    set_state_and_update(1500.0, 120.0, -15.0, 2000);
    state->setFlightStage(astra_rocket::COAST);

    TEST_ASSERT_EQUAL(0, controller->update(2.0));
    TEST_ASSERT_TRUE(std::isfinite(controller->getPredictedApogee()));
    TEST_ASSERT_TRUE(controller->getPredictedApogee() > 1500.0);
    TEST_ASSERT_TRUE(correctedBaro->getDynamicPressure() > 10.0);

    local_tearDown();
}

void test_install_baro_wrapper_behavior()
{
    // Null manager path
    AirbrakeController ctrl(nullptr, nullptr, nullptr, "WrapTest");
    TEST_ASSERT_FALSE(ctrl.installBaroWrapper(nullptr));

    // Missing barometer source path
    astra::SensorManager emptyManager;
    TEST_ASSERT_FALSE(ctrl.installBaroWrapper(&emptyManager));

    // Happy path
    FakeBarometer rawBaro;
    rawBaro.begin();
    astra::SensorManager manager;
    manager.setBaroSource(&rawBaro);

    ctrl.enableBaroCorrection(true, 0.052, 0.15);
    TEST_ASSERT_TRUE(ctrl.installBaroWrapper(&manager));

    astra::Barometer *wrapped = manager.getBaroSource();
    TEST_ASSERT_NOT_NULL(wrapped);
    TEST_ASSERT_TRUE(wrapped != &rawBaro);
    TEST_ASSERT_TRUE(wrapped->isInitialized());

    // Idempotent re-install path (already wrapped)
    astra::Barometer *firstWrapped = wrapped;
    TEST_ASSERT_TRUE(ctrl.installBaroWrapper(&manager));
    TEST_ASSERT_EQUAL_PTR(firstWrapped, manager.getBaroSource());
}

void run_test_airbrake_controller_tests()
{
    RUN_TEST(test_begin_requires_motor_and_state);
    RUN_TEST(test_enable_disable_state_tracking);
    RUN_TEST(test_update_when_disabled_returns_error_and_clears_baro_inputs);
    RUN_TEST(test_update_when_motor_uninitialized_returns_error);
    RUN_TEST(test_update_with_zero_speed_sets_predicted_apogee_to_altitude);
    RUN_TEST(test_update_in_coast_computes_dynamic_pressure);
    RUN_TEST(test_install_baro_wrapper_behavior);
}

} // namespace test_airbrake_controller
