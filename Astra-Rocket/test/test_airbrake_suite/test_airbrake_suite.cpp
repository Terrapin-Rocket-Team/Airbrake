#include <unity.h>

#include "test_airbrake_controller.h"
#include "test_error_corrected_baro.h"
#include "test_motor_driver_native.h"

void setUp(void)
{
    // Called before each test.
}

void tearDown(void)
{
    // Called after each test.
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    test_error_corrected_baro::run_test_error_corrected_baro_tests();
    test_airbrake_controller::run_test_airbrake_controller_tests();
    test_motor_driver_native::run_test_motor_driver_native_tests();

    UNITY_END();
    return 0;
}
