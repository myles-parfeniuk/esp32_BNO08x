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
 * @test Driver Creation for [CallbackAllReportVoidInputParam] Tests
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] test group.
 *
 * 1. Create a test IMU instance for use in [CallbackAllReportVoidInputParam] test group.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for [CallbackAllReportVoidInputParam] Tests", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackAllReportVoidInputParam] Tests";

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
 * @test Void Input Param Flavor Cb
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
 * 11. Attempt to disable all reports after callback called RX_REPORT_TRIAL_COUNT times and release test lock.
 *
 * 12. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Void Input Param Flavor Cb", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Void Input Param Flavor Cb";
    constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

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

                        i++;

                        if (i > RX_REPORT_TRIAL_CNT)
                        {
                            // 11.
                            TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                            xSemaphoreGive(test_lock);
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
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD));

    // 7.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD));

    // 8.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD));

    // 9.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD));

    // 10.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));

    // 12.
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
 * @test Driver Cleanup for [CallbackAllReportVoidInputParam] Tests"
 *
 * This test destroys the BNO08x IMU driver object for use in the [CallbackAllReportVoidInputParam] test group.
 *
 * 1. Destroys the test IMU instance that was used with the CallbackAllReportVoidInputParam test group.
 *
 */
TEST_CASE("Driver Cleanup for [CallbackAllReportVoidInputParam] Tests", "[CallbackAllReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackAllReportVoidInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for [CallbackAllReportIDInputParam] Tests
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackAllReportIDInputParam] test group.
 *
 * 1. Create a test IMU instance for use in [CallbackAllReportIDInputParam] test group.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for [CallbackAllReportIDInputParam] Tests", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackAllReportIDInputParam] Tests";

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
 * @test Report ID Input Param Flavor Cb
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
 * 11. Attempt to disable all reports after callback called RX_REPORT_TRIAL_COUNT times and release test lock.
 *
 * 12. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Report ID Input Param Flavor Cb", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Report ID Input Param Flavor Cb";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 8;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 100000UL; // 100ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

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

                        i++;

                        if (i > RX_REPORT_TRIAL_CNT)
                        {
                            // 11. 
                            TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                            xSemaphoreGive(test_lock);
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
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));

    // 3.
    TEST_ASSERT_EQUAL(true, imu->rpt.linear_accelerometer.enable(REPORT_PERIOD));

    // 4.
    TEST_ASSERT_EQUAL(true, imu->rpt.gravity.enable(REPORT_PERIOD));

    // 5.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_gyro.enable(REPORT_PERIOD));

    // 6.
    TEST_ASSERT_EQUAL(true, imu->rpt.cal_magnetometer.enable(REPORT_PERIOD));

    // 7.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv.enable(REPORT_PERIOD));

    // 8.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_game.enable(REPORT_PERIOD));

    // 9.
    TEST_ASSERT_EQUAL(true, imu->rpt.rv_geomagnetic.enable(REPORT_PERIOD));
    
    // 10.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));
    
    // 12.
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
 * @test Driver Cleanup for [CallbackAllReportIDInputParam] Tests"
 *
 * This test destroys the BNO08x IMU driver object for use in the [CallbackAllReportIDInputParam] test group.
 *
 * 1. Destroys the test IMU instance that was used with the CallbackAllReportIDInputParam test group.
 *
 */
TEST_CASE("Driver Cleanup for [CallbackAllReportIDInputParam] Tests", "[CallbackAllReportIDInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackAllReportIDInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Creation for [CallbackSingleReportVoidInputParam] Tests
 *
 * This test creates and initializes a BNO08x IMU driver object for use in the [CallbackSingleReportVoidInputParam] test group.
 *
 * 1. Create a test IMU instance for use in [CallbackSingleReportVoidInputParam] test group.
 *
 * 2. Call the public BNO08x::initialize() function and assert it returns without fail.
 *
 */
TEST_CASE("Driver Creation for [CallbackSingleReportVoidInputParam] Tests", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Creation for [CallbackSingleReportVoidInputParam] Tests";

    BNO08x* imu = nullptr;

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    BNO08xTestHelper::print_test_msg(TEST_TAG, "Creating & initializing BNO08x driver.");
    BNO08xTestHelper::create_test_imu();
    imu = BNO08xTestHelper::get_test_imu();

    // ensure IMU initialized successfully
    TEST_ASSERT_EQUAL(true, imu->initialize());
    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Single Report Void Input Param Flavor Cb
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
 * 4. Attempt to disable all reports after callback called RX_REPORT_TRIAL_COUNT times and release test lock.
 *
 * 5. Assert that report data was received for all enabled reports.
 *
 */
TEST_CASE("Single Report Void Input Param Flavor Cb", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Single Report Void Input Param Flavor Cb";
    static const constexpr uint8_t ENABLED_REPORT_COUNT = 1;
    static const constexpr uint8_t RX_REPORT_TRIAL_CNT = ENABLED_REPORT_COUNT * 5;
    constexpr uint32_t REPORT_PERIOD = 60000UL; // 60ms
    constexpr uint32_t CB_EXECUTION_TIMEOUT_MS = (REPORT_PERIOD / 1000UL) * RX_REPORT_TRIAL_CNT * 2;

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

                                        i++;

                                        if (i > RX_REPORT_TRIAL_CNT)
                                        {
                                            // 4.
                                            TEST_ASSERT_EQUAL(true, imu->disable_all_reports());
                                            xSemaphoreGive(test_lock);
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
    TEST_ASSERT_EQUAL(true, imu->rpt.accelerometer.enable(REPORT_PERIOD));
    
    // 3.
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(test_lock, CB_EXECUTION_TIMEOUT_MS / portTICK_PERIOD_MS));
    
    // 5.
    TEST_ASSERT_EQUAL(true, data_available_accel);

    vSemaphoreDelete(test_lock);

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}

/**
 * @test Driver Cleanup for [CallbackSingleReportVoidInputParam] Tests"
 *
 * This test destroys the BNO08x IMU driver object for use in the [CallbackSingleReportVoidInputParam] test group.
 *
 * 1. Destroys the test IMU instance that was used with the CallbackSingleReportVoidInputParam test group.
 *
 */
TEST_CASE("Driver Cleanup for [CallbackSingleReportVoidInputParam] Tests", "[CallbackSingleReportVoidInputParam]")
{
    const constexpr char* TEST_TAG = "Driver Cleanup for [CallbackSingleReportVoidInputParam] Tests";

    BNO08xTestHelper::print_test_start_banner(TEST_TAG);

    // 1.
    BNO08xTestHelper::print_test_msg(TEST_TAG, "Destroying BNO08x Driver.");
    BNO08xTestHelper::destroy_test_imu();

    BNO08xTestHelper::print_test_end_banner(TEST_TAG);
}
