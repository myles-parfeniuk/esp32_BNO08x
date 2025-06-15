/**
 * @file CallbackTests.cpp
 * @author Myles Parfeniuk
 *
 *
 * @warning YOU MUST ADD THE FOLLOWING LINE TO YOUR MAIN PROJECTS CMakeLists.txt IN ORDER FOR THIS
 * TEST SUITE TO BE BUILT WITH PROJECT: set(TEST_COMPONENTS "esp32_BNO08x" CACHE STRING "Components
 * to test.")
 */

#include "unity.h"
#include "../include/BNO08xTestHelper.hpp"

/**
 * @test Driver Creation for Read Octo Report Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] - Read Octo
 * Report Test
 *
 * 1. Create a test IMU instance for use in Read Octo Report Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Read Octo Report Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Read Octo Report Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Octo Report
 *
 * This test validates the basic functionality of the void input parameter callback flavor,
 * (one registered to no specific report that is not passed the report ID).
 *
 * 1. Attempt to register a void input parameter flavor callback and assert that it happens successfully.
 *
 * 2. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 4. Attempt to enable gravity reports and assert that it happens successfully.
 *
 * 5. Attempt to enable calibrated gyroscope reports and assert that it happens successfully.
 *
 * 6. Attempt to enable calibrated magnetometer reports and assert that it happens successfully.
 *
 * 7. Attempt to enable rotation vector reports and assert that it happens successfully.
 *
 * 8. Attempt to enable game rotation vector reports and assert that it happens successfully.
 *
 * 9. Attempt to enable geomagnetic rotation vector reports and assert that it happens successfully.
 *
 * 10. Block until test lock is released by callback or timeout occurs.
 *
 * 11. Assert that callback releases test lock successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 12. Attempt to disable all reports.
 *
 * 13. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Octo Report", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Read Octo Report";
    constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_grav = false;
    bool data_available_cal_gyro = false;
    bool data_available_cal_magnetometer = false;
    bool data_available_rv = false;
    bool data_available_rv_game = false;
    bool data_available_rv_geomagnetic = false;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;
    bno08x_magf_t data_magf;
    bno08x_quat_t data_quat;

    SemaphoreHandle_t test_lock = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true,
            imu->register_cb(
                    [&imu, &data_available_accel, &data_available_lin_accel, &data_available_grav, &data_available_cal_gyro,
                            &data_available_cal_magnetometer, &data_accel, &data_available_rv, &data_available_rv_game,
                            &data_available_rv_geomagnetic, &data_quat, &data_vel, &data_magf, &msg_buff, &test_lock]()
                    {
                        static int i = 0;
                        static bool lock_release = false;

                        i++;

                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                        {
                            // 11.
                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock));
                            lock_release = true;
                            return;
                        }

                        if (imu->rpt.accelerometer.has_new_data())
                        {
                            data_available_accel = true;
                            data_accel = imu->rpt.accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                    "accuracy: %s ",
                                    i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.linear_accelerometer.has_new_data())
                        {
                            data_available_lin_accel = true;
                            data_accel = imu->rpt.linear_accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.gravity.has_new_data())
                        {
                            data_available_grav = true;
                            data_accel = imu->rpt.gravity.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.cal_gyro.has_new_data())
                        {
                            data_available_cal_gyro = true;
                            data_vel = imu->rpt.cal_gyro.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    i, data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.cal_magnetometer.has_new_data())
                        {
                            data_available_cal_magnetometer = true;
                            data_magf = imu->rpt.cal_magnetometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f y: "
                                    "%.2f z: %.2f accuracy: %s ",
                                    i, data_magf.x, data_magf.y, data_magf.z, BNO08xAccuracy_to_str(data_magf.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.rv.has_new_data())
                        {
                            data_available_rv = true;
                            data_quat = imu->rpt.rv.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: %.2f k: "
                                    "%.2f accuracy: %s ",
                                    i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.rv_game.has_new_data())
                        {
                            data_available_rv_game = true;
                            data_quat = imu->rpt.rv_game.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f j: "
                                    "%.2f k: %.2f accuracy: %s ",
                                    i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.rv_geomagnetic.has_new_data())
                        {
                            data_available_rv_geomagnetic = true;
                            data_quat = imu->rpt.rv_geomagnetic.get_quat();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: "
                                    "%.2f j: %.2f k: %.2f accuracy: %s ",
                                    i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                    BNO08xAccuracy_to_str(data_quat.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }
                    }));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD_US));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD_US));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD_US));

    // 7.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    // 8.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD_US));

    // 9.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD_US));

    // 10.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 12.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 13.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_grav);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);
    TEST_ASSERT_EQUAL(true, data_available_cal_magnetometer);
    TEST_ASSERT_EQUAL(true, data_available_rv);
    TEST_ASSERT_EQUAL(true, data_available_rv_game);
    TEST_ASSERT_EQUAL(true, data_available_rv_geomagnetic);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Read Octo Report Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportVoidInputParam] - Read Octo Report Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportVoidInputParam] - Read Octo Report Test.
 *
 */
TEST_CASE("Driver Cleanup for Read Octo Report", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Read Octo Report";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for Register Cb After Reports Enabled Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] - Register Cb
 * After Reports Enabled Test
 *
 * 1. Create a test IMU instance for use in Register Cb After Reports Enabled Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Register Cb After Reports Enabled Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Register Cb After Reports Enabled Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Register Cb After Reports Enabled
 *
 * This test validates that a callback can be registered after reports have already been enabled.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to register a void input parameter flavor callback and assert that it happens successfully.
 *
 * 4. Block until test lock is released by callback or timeout occurs.
 *
 * 5. Assert that callback releases test lock successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 6. Attempt to disable all reports.
 *
 * 7. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Register Cb After Reports Enabled", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Register Cb After Reports Enabled";
    constexpr uint8_t ENABLED_REPORT_COUNT = 2;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;

    bno08x_accel_t data_accel;

    SemaphoreHandle_t test_lock = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true,
            imu->register_cb(
                    [&imu, &data_available_accel, &data_available_lin_accel, &data_accel, &msg_buff, &test_lock]()
                    {
                        static int i = 0;
                        static bool lock_release = false;

                        i++;

                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                        {
                            // 5.
                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock));
                            lock_release = true;
                            return;
                        }

                        if (imu->rpt.accelerometer.has_new_data())
                        {
                            data_available_accel = true;
                            data_accel = imu->rpt.accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                    "accuracy: %s ",
                                    i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }

                        if (imu->rpt.linear_accelerometer.has_new_data())
                        {
                            data_available_lin_accel = true;
                            data_accel = imu->rpt.linear_accelerometer.get();
                            sprintf(msg_buff,
                                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: "
                                    "%.2f accuracy: %s ",
                                    i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                            return;
                        }
                    }));

    // 4.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 7.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Register Cb After Reports Enabled Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportVoidInputParam] - Register Cb After
 * Reports Enabled Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportVoidInputParam] - Register Cb After Reports Enabled
 * Test.
 *
 */
TEST_CASE("Driver Cleanup for Register Cb After Reports Enabled Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Register Cb After Reports Enabled Test";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for Register Cb During Cb Execution Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] - Register Cb
 * During Cb Execution Test
 *
 * 1. Create a test IMU instance for use in Register Cb During Cb Execution Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Register Cb During Cb Execution Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Register Cb During Cb Execution Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Register Cb During Cb Execution
 *
 * This test validates that a callback can be registered after reports have been enabled and another callback has already been
 * registered.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to register a void input parameter flavor callback and assert that it happens successfully.
 *
 * 4. Block until test lock 1 is released by callback or timeout occurs.
 *
 * 5. Assert that callback releases test lock 1 successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 6. Register another void input parameter flavor callback and assert that it happens successfully.
 *
 * 7. Block until test lock 2 is released by callback or timeout occurs.
 *
 * 8. Assert that callback releases test lock 2 successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 9. Attempt to disable all reports.
 *
 * 10. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Register Cb During Cb Execution", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Register Cb During Cb Execution";
    constexpr uint8_t ENABLED_REPORT_COUNT = 2;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;

    bno08x_accel_t data_accel;

    SemaphoreHandle_t test_lock_cb_1 = xSemaphoreCreateBinary();
    SemaphoreHandle_t test_lock_cb_2 = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->register_cb(
                                    [&imu, &data_available_accel, &data_accel, &msg_buff, &test_lock_cb_1]()
                                    {
                                        static int i = 0;
                                        static bool lock_release = false;

                                        i++;

                                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                                        {
                                            // 5.
                                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock_cb_1));
                                            lock_release = true;
                                            return;
                                        }

                                        if (imu->rpt.accelerometer.has_new_data())
                                        {
                                            data_available_accel = true;
                                            data_accel = imu->rpt.accelerometer.get();
                                            sprintf(msg_buff,
                                                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                                    "accuracy: %s ",
                                                    i, data_accel.x, data_accel.y, data_accel.z,
                                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                            return;
                                        }
                                    }));
    // 4.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock_cb_1, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->register_cb(
                                    [&imu, &data_available_lin_accel, &data_accel, &msg_buff, &test_lock_cb_2]()
                                    {
                                        static int i = 0;
                                        static bool lock_release = false;

                                        i++;

                                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                                        {
                                            // 8.
                                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock_cb_2));
                                            lock_release = true;
                                            return;
                                        }

                                        if (imu->rpt.linear_accelerometer.has_new_data())
                                        {
                                            data_available_lin_accel = true;
                                            data_accel = imu->rpt.linear_accelerometer.get();
                                            sprintf(msg_buff,
                                                    "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f z: "
                                                    "%.2f accuracy: %s ",
                                                    i, data_accel.x, data_accel.y, data_accel.z,
                                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                            return;
                                        }
                                    }));

    // 7.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock_cb_2, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 9.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 10.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);

    vSemaphoreDelete(test_lock_cb_1);
    vSemaphoreDelete(test_lock_cb_2);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Register Cb During Cb Execution Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportVoidInputParam] - Register Cb During Cb
 * Execution Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportVoidInputParam] - Register Cb During Cb Execution
 * Test.
 *
 */
TEST_CASE("Driver Cleanup for Register Cb During Cb Execution Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Register Cb During Cb Execution Test";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for No Execution After Report Disable Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] - Register Cb
 * During Cb Execution Test
 *
 * 1. Create a test IMU instance for use in No Execution After Report Disable Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for No Execution After Report Disable Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for No Execution After Report Disable Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test No Execution After Report Disable
 *
 * This test validates that a callback is no longer executed if reports are disabled.
 *
 * 1. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to register a void input parameter flavor callback and assert that it happens successfully.
 *
 * 3. Block until test lock is released by callback or timeout occurs.
 *
 * 4. Assert that callback releases test lock successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 5. Attempt to disable all reports.
 *
 * 6. Block until test lock is released by callback or timeout occurs (timeout expected).
 *
 * 7. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("No Execution After Report Disable", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "No Execution After Report Disable";
    constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;

    bno08x_accel_t data_accel;

    SemaphoreHandle_t test_lock = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->register_cb(
                                    [&imu, &data_available_accel, &data_accel, &msg_buff, &test_lock]()
                                    {
                                        static int i = 0;

                                        i++;

                                        if (i > RX_REPORT_TRIAL_CNT)
                                        {
                                            // 4.
                                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock));
                                            i = 0;
                                            return;
                                        }

                                        if (imu->rpt.accelerometer.has_new_data())
                                        {
                                            data_available_accel = true;
                                            data_accel = imu->rpt.accelerometer.get();
                                            sprintf(msg_buff,
                                                    "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                                    "accuracy: %s ",
                                                    i, data_accel.x, data_accel.y, data_accel.z,
                                                    BNO08xAccuracy_to_str(data_accel.accuracy));
                                            BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                            return;
                                        }
                                    }));
    // 3.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 6.
    TEST_ASSERT_EQUAL(pdFALSE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 7.
    TEST_ASSERT_EQUAL(true, data_available_accel);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for No Execution After Report Disable Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportVoidInputParam] - Register Cb During Cb
 * Execution Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportVoidInputParam] - No Execution After Report Disable
 * Test.
 *
 */
TEST_CASE("Driver Cleanup for No Execution After Report Disable Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for No Execution After Report Disable Test";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for Execution Performance Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] - Performance
 * Test
 *
 * 1. Create a test IMU instance for use in Execution Performance Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Execution Performance Test", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Execution Performance Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Execution Performance
 *
 * This test benchmarks the performance of the internal callback dispatch engine.
 *
 * 1. Attempt to enable rotation vector reports and assert that it happens successfully.
 *
 * 2. Attempt to register a void input parameter flavor callback and assert that it happens successfully.
 *
 * 3. Block for BENCHMARK_INTERVAL_MS.
 *
 * 4. Attempt to disable all reports.
 *
 * 5. Calculate cb execution per second and compare to threshold.
 *
 */
TEST_CASE("Execution Performance", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Execution Performance";
    constexpr uint32_t REPORT_PERIOD_US = 4000UL;       // 4ms
    constexpr uint32_t BENCHMARK_INTERVAL_MS = 10000UL; // 10s
    constexpr float THRESHOLD_INVOCATION_P_MS = (static_cast<float>(REPORT_PERIOD_US - (REPORT_PERIOD_US / 80UL)) / 1000.0f) /
                                                10.0f; // minimum callback invocation per ms for test to pass

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    uint32_t cnt = 0UL;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->register_cb(
                                    [&imu, &cnt]()
                                    {
                                        if (imu->rpt.rv.has_new_data())
                                        {
                                            cnt++;
                                            return;
                                        }
                                    }));

    // 3.
    vTaskDelay(BENCHMARK_INTERVAL_MS / portTICK_PERIOD_MS);

    // 4.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 5.
    sprintf(msg_buff, "Rotation Vector Callback Invoked: %ld times in 10s", cnt);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
    float callback_p_ms = (cnt / 10.0f) / 1000.0f;
    sprintf(msg_buff, "Rotation Vector Callback Invocation/s: %.4f Vs. Threshold: %.4f ", callback_p_ms,
            THRESHOLD_INVOCATION_P_MS);
    BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);

    TEST_ASSERT_EQUAL(true, (callback_p_ms >= THRESHOLD_INVOCATION_P_MS));

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Execution Performance Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportVoidInputParam] - Execution Performance
 * Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportVoidInputParam] - Execution Performance Test.
 *
 */
TEST_CASE("Driver Cleanup for Execution Performance", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Execution Performance";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for Read Octo Report Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportIDInputParam] - Read Octo Report
 * Test
 *
 * 1. Create a test IMU instance for use in Read Octo Report Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Read Octo Report Test", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Read Octo Report Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Octo Report
 *
 * This test validates the basic functionality of the report ID input parameter callback flavor,
 * (one registered to no specific report that is passed the report ID which invoked it).
 *
 * 1. Attempt to register a report ID input parameter flavor callback and assert that it happens successfully.
 *
 * 2. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 3. Attempt to enable linear accelerometer reports and assert that it happens successfully.
 *
 * 4. Attempt to enable gravity reports and assert that it happens successfully.
 *
 * 5. Attempt to enable calibrated gyroscope reports and assert that it happens successfully.
 *
 * 6. Attempt to enable calibrated magnetometer reports and assert that it happens successfully.
 *
 * 7. Attempt to enable rotation vector reports and assert that it happens successfully.
 *
 * 8. Attempt to enable game rotation vector reports and assert that it happens successfully.
 *
 * 9. Attempt to enable geomagnetic rotation vector reports and assert that it happens successfully.
 *
 * 10. Block until test lock is released by callback or timeout occurs.
 *
 * 11. Assert that callback releases test lock successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 12. Attempt to disable all reports.
 *
 * 13. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Octo Report", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Read Octo Report";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;
    bool data_available_lin_accel = false;
    bool data_available_grav = false;
    bool data_available_cal_gyro = false;
    bool data_available_cal_magnetometer = false;
    bool data_available_rv = false;
    bool data_available_rv_game = false;
    bool data_available_rv_geomagnetic = false;

    bno08x_accel_t data_accel;
    bno08x_gyro_t data_vel;
    bno08x_magf_t data_magf;
    bno08x_quat_t data_quat;

    SemaphoreHandle_t test_lock = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true,
            imu->register_cb(
                    [&imu, &data_available_accel, &data_available_lin_accel, &data_available_grav, &data_available_cal_gyro,
                            &data_available_cal_magnetometer, &data_accel, &data_available_rv, &data_available_rv_game,
                            &data_available_rv_geomagnetic, &data_quat, &data_vel, &data_magf, &msg_buff,
                            &test_lock](uint8_t report_ID)
                    {
                        static int i = 0;
                        static bool lock_release = false;

                        i++;

                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                        {
                            // 11.
                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock));
                            lock_release = true;
                            return;
                        }

                        switch (report_ID)
                        {
                            case SH2_ACCELEROMETER:

                                data_available_accel = true;
                                data_accel = imu->rpt.accelerometer.get();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: "
                                        "%.2f accuracy: %s ",
                                        i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_LINEAR_ACCELERATION:
                                data_available_lin_accel = true;
                                data_accel = imu->rpt.linear_accelerometer.get();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: LinAccel: [m/s^2] x: %.2f y: %.2f "
                                        "z: %.2f accuracy: %s ",
                                        i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_GRAVITY:
                                data_available_grav = true;
                                data_accel = imu->rpt.gravity.get();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: Gravity: [m/s^2] x: %.2f y: %.2f z: "
                                        "%.2f accuracy: %s ",
                                        i, data_accel.x, data_accel.y, data_accel.z, BNO08xAccuracy_to_str(data_accel.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_GYROSCOPE_CALIBRATED:
                                data_available_cal_gyro = true;
                                data_vel = imu->rpt.cal_gyro.get();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: CalGyro: [rad/s] x: %.2f y: %.2f z: "
                                        "%.2f accuracy: %s ",
                                        i, data_vel.x, data_vel.y, data_vel.z, BNO08xAccuracy_to_str(data_vel.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_MAGNETIC_FIELD_CALIBRATED:
                                data_available_cal_magnetometer = true;
                                data_magf = imu->rpt.cal_magnetometer.get();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: CalMagnetometer: [uTesla] x: %.2f "
                                        "y: %.2f z: %.2f accuracy: %s ",
                                        i, data_magf.x, data_magf.y, data_magf.z, BNO08xAccuracy_to_str(data_magf.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_ROTATION_VECTOR:
                                data_available_rv = true;
                                data_quat = imu->rpt.rv.get_quat();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: RV: [n/a] real: %.2f i: %.2f j: "
                                        "%.2f k: %.2f accuracy: %s ",
                                        i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                        BNO08xAccuracy_to_str(data_quat.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_GAME_ROTATION_VECTOR:
                                data_available_rv_game = true;
                                data_quat = imu->rpt.rv_game.get_quat();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: RV Game: [n/a] real: %.2f i: %.2f "
                                        "j: %.2f k: %.2f accuracy: %s ",
                                        i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                        BNO08xAccuracy_to_str(data_quat.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                                data_available_rv_geomagnetic = true;
                                data_quat = imu->rpt.rv_geomagnetic.get_quat();
                                sprintf(msg_buff,
                                        "Rx Data Trial %d Success: RV Geomagnetic: [n/a] real: %.2f i: "
                                        "%.2f j: %.2f k: %.2f accuracy: %s ",
                                        i, data_quat.real, data_quat.i, data_quat.j, data_quat.k,
                                        BNO08xAccuracy_to_str(data_quat.accuracy));
                                BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                break;

                            default:
                                break;
                        }
                    }));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD_US));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD_US));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD_US));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD_US));

    // 7.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD_US));

    // 8.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD_US));

    // 9.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD_US));

    // 10.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 12.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 13.
    TEST_ASSERT_EQUAL(true, data_available_accel);
    TEST_ASSERT_EQUAL(true, data_available_lin_accel);
    TEST_ASSERT_EQUAL(true, data_available_grav);
    TEST_ASSERT_EQUAL(true, data_available_cal_gyro);
    TEST_ASSERT_EQUAL(true, data_available_cal_magnetometer);
    TEST_ASSERT_EQUAL(true, data_available_rv);
    TEST_ASSERT_EQUAL(true, data_available_rv_game);
    TEST_ASSERT_EQUAL(true, data_available_rv_geomagnetic);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Read Octo Report Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackAllReportIDInputParam] - Read Octo Report Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackAllReportIDInputParam] - Read Octo Report Test.
 *
 */
TEST_CASE("Driver Cleanup for Read Octo Report", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Read Octo Report";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for Read Single Report Test
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackSpecificReportVoidInputParam] - Read
 * Single Report Test
 *
 * 1. Create a test IMU instance for use in Read Single Report Test.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for Read Single Report Test", "[CallbackSpecificReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for Read Single Report Test";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // 2.
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Read Single Report
 *
 * This test validates the basic functionality of the void input parameter specific report callback flavor,
 * (one registered to a specific report).
 *
 * 1. Attempt to register a callback to accelerometer reports and assert that it happens successfully.
 *
 * 2. Attempt to enable accelerometer reports and assert that it happens successfully.
 *
 * 3. Block until test lock is released by callback or timeout occurs.
 *
 * 4. Assert that callback releases test lock successfully after callback invoked RX_REPORT_TRIAL_COUNT times.
 *
 * 5. Attempt to disable all reports.
 *
 * 6. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Read Single Report", "[CallbackSpecificReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Read Single Report";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD_US = 60000UL; // 60ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD_US / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

    BNO08x* imu = nullptr;
    char msg_buff[200] = {};
    bool data_available_accel = false;

    bno08x_accel_t data_accel;

    SemaphoreHandle_t test_lock = xSemaphoreCreateBinary();

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);
    imu = BNO08xTestHelper::get_test_imu();

    // 1.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.register_cb(
                                    [&imu, &data_available_accel, &data_accel, &msg_buff, &test_lock]()
                                    {
                                        static int i = 0;
                                        static bool lock_release = false;

                                        i++;

                                        if (i > RX_REPORT_TRIAL_CNT && !lock_release)
                                        {
                                            // 4.
                                            TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreGive(test_lock));
                                            lock_release = true;
                                            return;
                                        }

                                        data_available_accel = true;
                                        data_accel = imu->rpt.accelerometer.get();
                                        sprintf(msg_buff,
                                                "Rx Data Trial %d Success: Accel: [m/s^2] x: %.2f y: %.2f z: %.2f "
                                                "accuracy: %s ",
                                                i, data_accel.x, data_accel.y, data_accel.z,
                                                BNO08xAccuracy_to_str(data_accel.accuracy));
                                        BNO08xTestHelper::print_test_msg(TEST_TAG, msg_buff);
                                    }));

    // 2.
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD_US));

    // 3.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->disable_all_reports());

    // 6.
    TEST_ASSERT_EQUAL(true, data_available_accel);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for Read Single Report Test
 *
 * This test destroys the BNO08x IMU driver object that was used in the [CallbackSpecificReportVoidInputParam] - Read Single
 * Report Test.
 *
 * 1. Destroys the test IMU instance that was used with the [CallbackSpecificReportVoidInputParam] - Read Single Report Test.
 *
 */
TEST_CASE("Driver Cleanup for Read Single Report Test", "[CallbackSpecificReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for Read Single Report Test";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}
